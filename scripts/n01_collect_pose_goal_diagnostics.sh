#!/usr/bin/env bash
# N-01 focused diagnosis: /robot_pose source/timestamps -> goal gate -> SCAN output.
# READ ONLY. Does not publish goals, change parameters, invoke services/actions,
# stop/restart nodes, build packages, or authorize actuator output.

umask 077
WS_ROOT="$(cd "$(dirname "$0")/.." 2>/dev/null && pwd)"
OUT_DIR="$HOME/lyy/n01_runtime/runs/pose_goal_diagnostics"
STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT_DIR" || exit 2
LOG_FILE="$OUT_DIR/n01_pose_goal_diagnostic_$STAMP.log"
exec > >(tee -a "$LOG_FILE") 2>&1

finish() {
  printf '\n============================================================\n'
  printf 'LOG_FILE: %s\n' "$LOG_FILE"
  printf 'READ_ONLY: no goal published; no parameter writes; no service/action calls; no node restart\n'
  printf '============================================================\n'
}
trap finish EXIT

run_cmd() {
  seconds="$1"; shift
  printf '\n$'; printf ' %q' "$@"; printf '\n'
  timeout --signal=INT --kill-after=2s "${seconds}s" "$@"
  rc=$?
  printf '[exit_code=%s]\n' "$rc"
  return 0
}

section() {
  printf '\n\n============================================================\n%s\n============================================================\n' "$1"
}

has_topic() {
  printf '%s\n' "$TOPIC_LIST" | grep -Fxq "$1"
}

topic_info() {
  topic="$1"
  if has_topic "$topic"; then
    run_cmd 8 ros2 topic info "$topic" --verbose
  else
    printf '\nTOPIC_MISSING: %s\n' "$topic"
  fi
}

topic_echo() {
  topic="$1"; qos="$2"
  if has_topic "$topic"; then
    if [ "$qos" = "sensor" ]; then
      run_cmd 6 ros2 topic echo "$topic" --once --qos-reliability best_effort
    else
      run_cmd 6 ros2 topic echo "$topic" --once
    fi
  else
    printf '\nTOPIC_MISSING: %s\n' "$topic"
  fi
}

topic_hz() {
  topic="$1"; qos="$2"
  if has_topic "$topic"; then
    if [ "$qos" = "sensor" ]; then
      run_cmd 5 ros2 topic hz "$topic" --qos-reliability best_effort
    else
      run_cmd 5 ros2 topic hz "$topic"
    fi
  else
    printf '\nTOPIC_MISSING: %s\n' "$topic"
  fi
}

section "0. Machine, clock, branch and installed workspace"
printf 'time: %s\nhost: %s\nworkspace: %s\n' "$(date --iso-8601=ns)" "$(hostname)" "$WS_ROOT"
run_cmd 5 git -C "$WS_ROOT" branch --show-current
run_cmd 5 git -C "$WS_ROOT" rev-parse HEAD
run_cmd 5 git -C "$WS_ROOT" status --short --branch

section "1. Source ROS Humble and this workspace"
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  printf 'SOURCED: /opt/ros/humble/setup.bash\n'
else
  printf 'ERROR: ROS Humble setup missing\n'
fi
if [ -f "$WS_ROOT/install/local_setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/local_setup.bash"
  printf 'SOURCED: %s/install/local_setup.bash\n' "$WS_ROOT"
else
  printf 'WARNING: workspace install/local_setup.bash missing\n'
fi
printf 'ROS_DISTRO=%s ROS_DOMAIN_ID=%s RMW_IMPLEMENTATION=%s\n' "${ROS_DISTRO:-unset}" "${ROS_DOMAIN_ID:-unset}" "${RMW_IMPLEMENTATION:-default}"

section "2. ROS graph"
NODE_LIST="$(timeout 10s ros2 node list 2>&1)"
TOPIC_LIST="$(timeout 10s ros2 topic list 2>&1)"
printf '\nNODES:\n%s\n' "$NODE_LIST"
printf '\nTOPICS:\n'
timeout 10s ros2 topic list -t 2>&1

section "3. Exact /robot_pose publisher identities and full bridge diagnostic"
topic_info /robot_pose
topic_echo /robot_pose sensor
topic_info /n01/live_navigation/readiness
topic_echo /n01/live_navigation/readiness reliable

section "4. Supervisor and Goal Gate decision"
topic_info /n01/live_navigation/system_health
topic_echo /n01/live_navigation/system_health reliable
topic_info /n01/live_navigation/goal_gate_status
topic_echo /n01/live_navigation/goal_gate_status reliable

section "5. Goal input, gated output and planner trajectory wiring"
topic_info /move_base_simple/goal
topic_info /n01/live_navigation/goal_gated
topic_info /planning/bspline
topic_echo /planning/bspline reliable

section "6. Timestamp and rate samples"
topic_echo /odom sensor
topic_echo /body_pose sensor
topic_hz /robot_pose sensor
topic_hz /odom sensor
topic_hz /body_pose sensor
topic_hz /livox/lidar/geo_filtered sensor
topic_hz /cloud sensor
topic_hz /planning/bspline reliable

section "7. Relevant node wiring"
for node in /slam3d_node /n01_live_scan_bridge /n01_live_runtime_supervisor /n01_live_scan_goal_gate /scan_planner_node; do
  if printf '%s\n' "$NODE_LIST" | grep -Fxq "$node"; then
    run_cmd 8 ros2 node info "$node"
  else
    printf '\nNODE_NOT_RUNNING: %s\n' "$node"
  fi
done

section "8. Interpretation keys"
printf '%s\n' \
  'robot_pose_publishers in /n01/live_navigation/readiness identifies each /robot_pose publisher node.' \
  'pose_duplicate_conflict / pose_out_of_order / pose_stamp_advance counters reveal multi-publisher timestamp conflicts.' \
  'pose_scan_skew_s must remain within robot_pose_anchor_max_delta_s.' \
  'Goal Gate accepted=true means RViz goal was forwarded; accepted=false gives the exact rejection reason.' \
  'A received /planning/bspline message proves SCAN produced a trajectory; no message alone does not prove why.' \
  'This script never publishes a goal or enables robot motion.'
