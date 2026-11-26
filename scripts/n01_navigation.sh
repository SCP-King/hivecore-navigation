#!/usr/bin/env bash
# Unified lifecycle entry point for the N-01 live SCAN navigation stack.
# Usage: scripts/n01_navigation.sh start|stop|status
set -Eeuo pipefail
export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-ASYNCHRONOUS}"
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OMP_NUM_THREADS=1
export OMP_WAIT_POLICY=PASSIVE

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$ROOT/scripts/lib/slam_runtime.sh"
RUNTIME_ROOT="${N01_RUNTIME_ROOT:-$HOME/lyy/n01_runtime}"
RUN_DIR="$RUNTIME_ROOT/runs/navigation"
LOG_DIR="$RUNTIME_ROOT/logs"
PID_FILE="$RUN_DIR/live_scan_launch.pid"
DEFAULT_PCD="/home/demo/lyy/slam_data/maps/mapping_lite_20261109_140536_087.pcd"
PCD="${N01_NAV_PCD:-$DEFAULT_PCD}"
NAV_GROUND_Z="${N01_NAV_GROUND_Z:-}"
NAV_GROUND_A="${N01_NAV_GROUND_A:-0.0}"
NAV_GROUND_B="${N01_NAV_GROUND_B:-0.0}"
# RViz needs a real X display. Headless/remote GUI uses Foxglove or the GUI's
# standalone RViz button; do not let launch kill a dead rviz2 process.
NAV_RVIZ="${N01_NAV_RVIZ:-false}"
NAV_MAP_MODE="${N01_NAV_MAP_MODE:-offline}"
NAV_REFERENCE_TOPIC="${N01_NAV_REALTIME_REFERENCE_TOPIC:-}"
NAV_FOOTPRINT_MODEL="${N01_NAV_FOOTPRINT_MODEL:-urdf}"
SLAM_CONFIG="$ROOT/apps/n01_slam_gui/config/n01_localization_lite.yaml"
# ROS launch writes ~/.ros/log by default. In this deployment that path can be
# mounted read-only; fall back to a writable /tmp directory so the stack starts
# atomically instead of dying after static_map/scan_planner were spawned.
if [[ ! -w "$HOME/.ros/log" ]]; then
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/n01_ros_log}"
  export ROS_HOME="${ROS_HOME:-/tmp/n01_ros_home}"
  mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
  echo "WARN: $HOME/.ros/log is not writable; using ROS_LOG_DIR=$ROS_LOG_DIR"
fi
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
  n01_source_slam_runtime || return $?
  n01_source_setup "$ROOT/install/local_setup.bash" || return $?
  source "$ROOT/scripts/lib/ros_transport.sh"
  n01_configure_project_transport || return $?
}

managed_nodes_absent() {
  local nodes="$1" node
  local managed=(/n01_robot_footprint /n01_trajectory_display /n01_static_map /n01_realtime_reference /n01_environment_display /n01_live_runtime_supervisor
    /n01_live_safety_supervisor /n01_live_dynamic_obstacle_tracker
    /n01_hive2_trajectory_monitor /n01_live_scan_goal_gate /n01_run_cmd_logger /scan_planner_node
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
    nodes="$(ros2 node list --no-daemon --spin-time 1 2>/dev/null || true)"
    if managed_nodes_absent "$nodes"; then clean=1; break; fi
    sleep 1
  done
  if (( clean == 0 )); then
    echo "ERROR: ROS graph still reports managed N-01 node names after process-group shutdown:"
    grep -E '^/(n01_environment_display|n01_realtime_reference|n01_static_map|n01_live_runtime_supervisor|n01_live_safety_supervisor|n01_live_dynamic_obstacle_tracker|n01_hive2_trajectory_monitor|n01_live_scan_goal_gate|scan_planner_node|n01_live_navigation_rviz)$' <<<"$nodes" || true
    echo "No unrelated ROS nodes were signalled. Inspect these names before restarting."
    return 4
  fi
  echo "PASS: all N-01 navigation processes stopped and managed ROS graph entries cleared."
  echo "Preserved: Lite SLAM, independently managed localization adapter, Livox, IMU and robot drivers."
}

start_stack() {
  source_ros
  cd "$ROOT"

  if [[ "$NAV_MAP_MODE" != "offline" && "$NAV_MAP_MODE" != "realtime" ]]; then
    echo "ERROR: N01_NAV_MAP_MODE must be offline or realtime"
    return 2
  fi
  if [[ "$NAV_MAP_MODE" == "offline" ]]; then
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
  else
    echo "navigation map: realtime SLAM + SCAN live occupancy; reference=${NAV_REFERENCE_TOPIC:-none} (no historical PCD)"
  fi
  if [[ -z "$NAV_GROUND_Z" ]]; then
    if [[ "$NAV_MAP_MODE" == "realtime" ]]; then
      echo "ERROR: realtime mapping requires an explicitly synchronized ground plane; offline map profiles are not reused."
      return 2
    fi
    local saved_ground_values
    if ! saved_ground_values="$(N01_NAV_REPO="$ROOT" python3 "$ROOT/src/n01_live_navigation/n01_live_navigation/calibration_store.py" "$PCD")"; then
      echo "ERROR: no matching persisted ground; synchronize ground in GUI or pass explicit ground parameters."
      return 2
    fi
    read -r NAV_GROUND_Z NAV_GROUND_A NAV_GROUND_B <<< "$saved_ground_values"
  fi
  echo "navigation map mode: $NAV_MAP_MODE"
  echo "navigation ground_z_map_m: $NAV_GROUND_Z"
  echo "navigation rviz: $NAV_RVIZ"

  # Idempotent start: first close any previous managed run and legacy orphans.
  stop_stack

  local nodes adapter_count start_adapter launch_log launch_pid ready current missing node attempt
  # A just-restarted CLI daemon can return a partial graph. Use bounded fresh
  # DDS discovery before declaring the independently owned adapter missing.
  for attempt in 1 2 3; do
    if ! nodes="$(ros2 node list --no-daemon --spin-time 2)"; then
      echo "ERROR: ROS graph discovery failed; refusing navigation startup."
      return 3
    fi
    adapter_count="$(grep -Fxc "/n01_localization_adapter" <<<"$nodes" || true)"
    if [[ "${adapter_count:-0}" -ne 0 ]]; then break; fi
    echo "Waiting for localization adapter discovery ($attempt/3)."
  done

  if [[ "${adapter_count:-0}" -gt 1 ]]; then
    echo "ERROR: duplicate localization adapters; refusing to start."
    return 3
  elif [[ "${adapter_count:-0}" -eq 0 ]]; then
    echo "ERROR: /n01_localization_adapter is missing; start the localization data chain first."
    echo "Navigation only consumes the localization adapter and scan bridge."
    return 3
  else
    start_adapter=false
    echo "Reusing localization-owned /n01_localization_adapter."
  fi

  # The scan bridge is part of the navigation stack, but a previous stack (or a
  # separately managed bridge) may already own it. Reuse it when present, start
  # it otherwise; requiring it while passing start_scan_bridge:=false deadlocked
  # every start with "navigation nodes missing".
  bridge_count="$(grep -Fxc "/n01_live_scan_bridge" <<<"$nodes" || true)"
  if [[ "${bridge_count:-0}" -gt 1 ]]; then
    echo "ERROR: duplicate /n01_live_scan_bridge nodes; refusing to start."
    return 3
  elif [[ "${bridge_count:-0}" -eq 1 ]]; then
    start_bridge=false
    echo "Reusing existing /n01_live_scan_bridge."
  else
    start_bridge=true
    echo "Starting /n01_live_scan_bridge with the navigation stack."
  fi

  launch_log="$LOG_DIR/n01_navigation_launch_$(date +%Y%m%d_%H%M%S).log"
  echo "===== N-01 NAVIGATION START ====="
  launch_args=(
    map_mode:="$NAV_MAP_MODE"
    pcd_path:="$PCD"
    pcd_map_frame_asserted:="$([[ "$NAV_MAP_MODE" == "offline" ]] && echo true || echo false)"
    start_localization_adapter:="$start_adapter"
    start_scan_bridge:=$start_bridge
    navigation_speed_mps:="${N01_NAV_SPEED:-0.4}"
    ground_z_map_m:="$NAV_GROUND_Z"
    ground_slope_x:="$NAV_GROUND_A"
    ground_slope_y:="$NAV_GROUND_B"
    rviz:="$NAV_RVIZ"
    use_urdf_footprint:="$([[ "$NAV_FOOTPRINT_MODEL" == "double_cylinder" ]] && echo false || echo true)"
    double_cylinder_radius:="${N01_NAV_DC_RADIUS:-0.25}"
    double_cylinder_offset:="${N01_NAV_DC_OFFSET:-0.18}"
    environment_margin:="${N01_NAV_ENV_MARGIN:-$([[ "$NAV_FOOTPRINT_MODEL" == "double_cylinder" ]] && echo 0.0 || echo 0.06)}"
  )
  if [[ -n "${N01_NAV_MAP_ID:-}" ]]; then
    launch_args+=(map_id:="$N01_NAV_MAP_ID")
  fi
  if [[ -n "$NAV_REFERENCE_TOPIC" ]]; then
    launch_args+=(realtime_reference_topic:="$NAV_REFERENCE_TOPIC")
  fi
  if [[ -z "${N01_NAV_CPU_SET:-}" ]] && [[ "$(nproc)" -ge 8 ]]; then
    N01_NAV_CPU_SET="3-5"
  fi
  if [[ -n "${N01_NAV_CPU_SET:-}" ]] && command -v taskset >/dev/null 2>&1; then
    nohup setsid taskset -c "$N01_NAV_CPU_SET" ros2 launch n01_live_navigation live_scan.launch.py \
      "${launch_args[@]}" \
      >"$launch_log" 2>&1 </dev/null &
  else
    nohup setsid ros2 launch n01_live_navigation live_scan.launch.py \
      "${launch_args[@]}" \
      >"$launch_log" 2>&1 </dev/null &
  fi
  launch_pid=$!
  echo "$launch_pid" > "$PID_FILE"
  echo "launch_pid=$launch_pid"
  echo "launch_log=$launch_log"

  local required=(/n01_robot_footprint /n01_trajectory_display /n01_live_scan_bridge /n01_live_runtime_supervisor
    /n01_live_safety_supervisor /n01_live_dynamic_obstacle_tracker
    /n01_hive2_trajectory_monitor /n01_live_scan_goal_gate /scan_planner_node)
  if [[ "$NAV_MAP_MODE" == "offline" ]]; then required=(/n01_static_map "${required[@]}"); fi
  if [[ "$NAV_MAP_MODE" == "realtime" && -n "$NAV_REFERENCE_TOPIC" ]]; then required=(/n01_realtime_reference "${required[@]}"); fi
  # One persistent DDS participant avoids repeated discovery/teardown. The old
  # 45 x (node-list discovery + sleep) loop exceeded the helper's 90s budget.
  if ! python3 "$ROOT/scripts/n01_wait_navigation_nodes.py" --launch-pid "$launch_pid" \
       --timeout 35 "${required[@]}"; then
    tail -40 "$launch_log" || true
    stop_stack || true
    return 5
  fi
  echo "PASS: one N-01 live navigation stack is ready."
  echo "Use '$0 stop' to close it and remove all navigation processes."
}

status_stack() {
  source_ros
  local nodes
  nodes="$(ros2 node list --no-daemon --spin-time 1 2>/dev/null || true)"
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
  grep -E '^/(n01_environment_display|n01_realtime_reference|n01_static_map|n01_live_scan_bridge|n01_live_runtime_supervisor|n01_live_safety_supervisor|n01_live_dynamic_obstacle_tracker|n01_hive2_trajectory_monitor|n01_live_scan_goal_gate|scan_planner_node|n01_live_navigation_rviz|n01_localization_adapter)$' <<<"$nodes" || true
}

case "${1:-}" in
  start) start_stack ;;
  stop) stop_stack ;;
  status) status_stack ;;
  *) usage; exit 2 ;;
esac
