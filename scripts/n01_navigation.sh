#!/usr/bin/env bash
# Unified lifecycle entry point for the N-01 live SCAN navigation stack.
# Usage: scripts/n01_navigation.sh start|stop|status
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$ROOT/scripts/lib/ros_env.sh"
RUNTIME_ROOT="${N01_RUNTIME_ROOT:-$HOME/lyy/n01_runtime}"
RUN_DIR="$RUNTIME_ROOT/runs/navigation"
LOG_DIR="$RUNTIME_ROOT/logs"
PID_FILE="$RUN_DIR/live_scan_launch.pid"
DEFAULT_PCD="/opt/slam3d_release/resource/mapping_lite.pcd"
PCD="${N01_NAV_PCD:-$DEFAULT_PCD}"
NAV_GROUND_Z="${N01_NAV_GROUND_Z:-0.139}"
# RViz needs a real X display. Headless/remote GUI uses Foxglove or the GUI's
# standalone RViz button; do not let launch kill a dead rviz2 process.
NAV_RVIZ="${N01_NAV_RVIZ:-false}"
SLAM_CONFIG="/opt/slam3d_release/install/hivecore_slam3d_lite_localization/share/hivecore_slam3d_lite_localization/config/localization_lite.yaml"
mkdir -p "$RUN_DIR" "$LOG_DIR"

usage() {
  echo "Usage: $0 {start|stop|status}"
}

source_ros() {
  # ROS/colcon setup files read optional variables, so they must not be sourced
  # under nounset (set -u). n01_source_setup temporarily disables only nounset
  # and restores the caller's setting.
  n01_source_setup /opt/ros/humble/setup.bash || return $?
  if [[ ! -r "$ROOT/install/setup.bash" ]]; then
    echo "ERROR: $ROOT/install/setup.bash is missing; build the workspace first." >&2
    return 2
  fi
  n01_source_setup "$ROOT/install/setup.bash" || return $?
}

managed_nodes_absent() {
  local nodes="$1" node
  local managed=(/n01_static_map /n01_live_scan_bridge /n01_live_runtime_supervisor
    /n01_live_safety_supervisor /n01_live_dynamic_obstacle_tracker
    /n01_hive2_trajectory_monitor /n01_live_scan_goal_gate /scan_planner_node
    /n01_live_navigation_rviz)
  for node in "${managed[@]}"; do
    if grep -Fxq "$node" <<<"$nodes"; then return 1; fi
  done
  return 0
}

refresh_graph() {
  # The ROS CLI daemon is only a discovery cache; restarting it does not stop
  # ROS nodes or affect the vendor SLAM / sensor processes.
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
}

stop_stack() {
  source_ros
  echo "===== N-01 NAVIGATION STOP ====="
  local rc=0
  if [[ -s "$PID_FILE" ]]; then
    local launch_pid
    launch_pid="$(cat "$PID_FILE")"
    if ! python3 "$ROOT/scripts/n01_stop_live_stack.py" --launch-pid "$launch_pid"; then
      rc=1
    fi
  else
    echo "No active launch PID file; checking legacy N-01 orphans."
    if ! python3 "$ROOT/scripts/n01_stop_live_stack.py"; then rc=1; fi
  fi

  if (( rc == 0 )); then
    rm -f "$PID_FILE"
  else
    echo "ERROR: cleanup incomplete; preserving PID file for another stop attempt."
    return 3
  fi

  refresh_graph
  local clean=0 nodes
  for _ in $(seq 1 12); do
    nodes="$(ros2 node list 2>/dev/null || true)"
    if managed_nodes_absent "$nodes"; then clean=1; break; fi
    sleep 1
  done
  if (( clean == 0 )); then
    echo "ERROR: ROS graph still reports managed N-01 node names after process-group shutdown:"
    grep -E '^/(n01_static_map|n01_live_scan_bridge|n01_live_runtime_supervisor|n01_live_safety_supervisor|n01_live_dynamic_obstacle_tracker|n01_hive2_trajectory_monitor|n01_live_scan_goal_gate|scan_planner_node|n01_live_navigation_rviz)$' <<<"$nodes" || true
    echo "No unrelated ROS nodes were signalled. Inspect these names before restarting."
    return 4
  fi
  echo "PASS: all N-01 navigation processes stopped and managed ROS graph entries cleared."
  echo "Preserved: Lite SLAM, independently managed localization adapter, Livox, IMU and robot drivers."
}

start_stack() {
  source_ros
  cd "$ROOT"

  if [[ ! -r "$PCD" ]]; then
    echo "ERROR: navigation PCD is not readable: $PCD"
    return 2
  fi
  if [[ -z "${N01_NAV_PCD:-}" ]] && { [[ ! -r "$SLAM_CONFIG" ]] || ! grep -Fq "map_path: $PCD" "$SLAM_CONFIG"; }; then
    echo "ERROR: active Lite localization config does not confirm $PCD."
    return 2
  fi
  if [[ -n "${N01_NAV_PCD:-}" ]] && ! grep -Fq "map_path: $PCD" "$SLAM_CONFIG" 2>/dev/null; then
    echo "WARN: explicit N01_NAV_PCD differs from the static Lite YAML; using explicit PCD."
  fi
  echo "navigation PCD: $PCD"
  echo "navigation ground_z_map_m: $NAV_GROUND_Z"
  echo "navigation rviz: $NAV_RVIZ"

  # Idempotent start: first close any previous managed run and legacy orphans.
  stop_stack

  local nodes adapter_count start_adapter launch_log launch_pid ready current missing node
  nodes="$(ros2 node list 2>/dev/null || true)"
  adapter_count="$(grep -Fxc "/n01_localization_adapter" <<<"$nodes" || true)"
  if [[ "${adapter_count:-0}" -gt 1 ]]; then
    echo "ERROR: duplicate localization adapters; refusing to start."
    return 3
  elif [[ "${adapter_count:-0}" -eq 1 ]]; then
    start_adapter=false
    echo "Reusing independent /n01_localization_adapter."
  else
    start_adapter=true
    if [[ ! -r /opt/slam3d_release/setup_runtime.sh ]]; then
      echo "ERROR: Lite runtime setup is unavailable."
      return 3
    fi
    set +u
    source /opt/slam3d_release/setup_runtime.sh
    source "$ROOT/install/setup.bash"
    set -u
    ros2 interface show hivecore_slam3d_lite_localization/msg/LocStatus >/dev/null
  fi

  launch_log="$LOG_DIR/n01_navigation_launch_$(date +%Y%m%d_%H%M%S).log"
  echo "===== N-01 NAVIGATION START ====="
  nohup setsid ros2 launch n01_live_navigation live_scan.launch.py \
    pcd_path:="$PCD" \
    pcd_map_frame_asserted:=true \
    start_localization_adapter:="$start_adapter" \
    ground_z_map_m:="$NAV_GROUND_Z" \
    rviz:="$NAV_RVIZ" \
    >"$launch_log" 2>&1 </dev/null &
  launch_pid=$!
  echo "$launch_pid" > "$PID_FILE"
  echo "launch_pid=$launch_pid"
  echo "launch_log=$launch_log"

  local required=(/n01_static_map /n01_live_scan_bridge /n01_live_runtime_supervisor
    /n01_live_safety_supervisor /n01_live_dynamic_obstacle_tracker
    /n01_hive2_trajectory_monitor /n01_live_scan_goal_gate /scan_planner_node)
  ready=0
  for _ in $(seq 1 45); do
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      echo "ERROR: launch exited before navigation became ready."
      tail -100 "$launch_log" || true
      stop_stack || true
      return 5
    fi
    current="$(ros2 node list 2>/dev/null || true)"
    missing=0
    for node in "${required[@]}"; do
      [[ "$(grep -Fxc "$node" <<<"$current" || true)" -eq 1 ]] || missing=1
    done
    if (( missing == 0 )); then ready=1; break; fi
    sleep 1
  done
  if (( ready == 0 )); then
    echo "ERROR: navigation graph did not become ready within 45 seconds."
    tail -100 "$launch_log" || true
    stop_stack || true
    return 5
  fi
  echo "PASS: one N-01 live navigation stack is ready."
  echo "Use '$0 stop' to close it and remove all navigation processes."
}

status_stack() {
  source_ros
  local nodes
  nodes="$(ros2 node list 2>/dev/null || true)"
  echo "===== N-01 NAVIGATION STATUS ====="
  if [[ -s "$PID_FILE" ]]; then
    local pid
    pid="$(cat "$PID_FILE")"
    if kill -0 "$pid" 2>/dev/null; then
      echo "launch_pid=$pid (running)"
    else
      echo "launch_pid=$pid (stale PID file)"
    fi
  else
    echo "No managed launch PID file."
  fi
  grep -E '^/(n01_static_map|n01_live_scan_bridge|n01_live_runtime_supervisor|n01_live_safety_supervisor|n01_live_dynamic_obstacle_tracker|n01_hive2_trajectory_monitor|n01_live_scan_goal_gate|scan_planner_node|n01_live_navigation_rviz|n01_localization_adapter)$' <<<"$nodes" || true
}

case "${1:-}" in
  start) start_stack ;;
  stop) stop_stack ;;
  status) status_stack ;;
  *) usage; exit 2 ;;
esac
