#!/usr/bin/env bash
# N-01 one-shot full runtime diagnostics for A/B/C/D.
# READ ONLY: no command publication, ROS service calls, parameter writes, builds, tests, or node restarts.

set -u
umask 077

LYY_ROOT="$HOME/lyy"
WS_ROOT="$(cd "$(dirname "$0")/.." 2>/dev/null && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LYY_ROOT/n01_runtime/runs/full_diagnostics"
mkdir -p "$LOG_DIR" || exit 2
LOG_FILE="$LOG_DIR/n01_full_runtime_diagnostic_$STAMP.log"

exec > >(tee -a "$LOG_FILE") 2>&1

finish() {
  printf '\n============================================================\n'
  printf 'N-01 full runtime diagnostic finished: %s\n' "$(date --iso-8601=seconds)"
  printf 'LOG_FILE: %s\n' "$LOG_FILE"
  printf 'READ_ONLY: no command publication, no service calls, no parameter writes\n'
  printf '============================================================\n'
}
trap finish EXIT

section() {
  printf '\n\n============================================================\n'
  printf '%s\n' "$1"
  printf '============================================================\n'
}

run_cmd() {
  seconds="$1"
  shift
  printf '\n$'
  printf ' %q' "$@"
  printf '\n'
  timeout --signal=INT --kill-after=2s "$seconds"s "$@"
  rc=$?
  printf '[exit_code=%s]\n' "$rc"
  return 0
}

has_topic() {
  printf '%s\n' "$TOPIC_LIST" | grep -Fxq "$1"
}

has_node() {
  printf '%s\n' "$NODE_LIST" | grep -Fxq "$1"
}

topic_info() {
  topic="$1"
  if has_topic "$topic"; then
    run_cmd 9 ros2 topic info "$topic" --verbose
  else
    printf '\nTOPIC_MISSING: %s\n' "$topic"
  fi
}

topic_echo() {
  topic="$1"
  shift
  if has_topic "$topic"; then
    run_cmd 8 ros2 topic echo "$topic" --once "$@"
  else
    printf '\nTOPIC_MISSING: %s\n' "$topic"
  fi
}

topic_hz() {
  topic="$1"
  if has_topic "$topic"; then
    run_cmd 6 ros2 topic hz "$topic"
  else
    printf '\nTOPIC_MISSING: %s\n' "$topic"
  fi
}

node_info() {
  node="$1"
  if has_node "$node"; then
    run_cmd 10 ros2 node info "$node"
  else
    printf '\nNODE_NOT_RUNNING: %s\n' "$node"
  fi
}

node_params() {
  node="$1"
  if has_node "$node"; then
    run_cmd 12 ros2 param dump "$node"
  else
    printf '\nNODE_NOT_RUNNING_SKIP_PARAMS: %s\n' "$node"
  fi
}

section "0. Run identity, machine, clock and ROS environment"
printf 'timestamp: %s\n' "$(date --iso-8601=ns)"
printf 'hostname: %s\n' "$(hostname)"
printf 'workspace: %s\n' "$WS_ROOT"
printf 'log_file: %s\n' "$LOG_FILE"
printf 'user: %s\n' "$(id)"
run_cmd 5 uname -a
run_cmd 6 timedatectl status
printf '\nROS_DISTRO=%s\n' "$(printenv ROS_DISTRO 2>/dev/null || echo unset)"
printf 'ROS_DOMAIN_ID=%s\n' "$(printenv ROS_DOMAIN_ID 2>/dev/null || echo unset)"
printf 'RMW_IMPLEMENTATION=%s\n' "$(printenv RMW_IMPLEMENTATION 2>/dev/null || echo default)"
printf 'ROS_LOCALHOST_ONLY=%s\n' "$(printenv ROS_LOCALHOST_ONLY 2>/dev/null || echo unset)"
printf 'AMENT_PREFIX_PATH=%s\n' "$(printenv AMENT_PREFIX_PATH 2>/dev/null || echo unset)"

section "1. Source ROS and this workspace without reloading stale underlays"
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  printf 'SOURCED: /opt/ros/humble/setup.bash\n'
else
  printf 'ERROR: /opt/ros/humble/setup.bash missing\n'
fi
if [ -f "$WS_ROOT/install/local_setup.bash" ]; then
  # Use local_setup after the ROS underlay to avoid obsolete underlay paths
  # recorded in install/setup.bash (for example ~/FROZEN).
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/local_setup.bash"
  printf 'SOURCED: %s/install/local_setup.bash\n' "$WS_ROOT"
else
  printf 'WARNING: workspace install/local_setup.bash missing; ROS packages may be unavailable\n'
fi
printf 'ROS_DISTRO_AFTER_SOURCE=%s\n' "$(printenv ROS_DISTRO 2>/dev/null || echo unset)"
printf 'AMENT_PREFIX_PATH_AFTER_SOURCE=%s\n' "$(printenv AMENT_PREFIX_PATH 2>/dev/null || echo unset)"

section "2. Git, workspace and installed package identity"
run_cmd 5 git -C "$WS_ROOT" branch --show-current
run_cmd 5 git -C "$WS_ROOT" rev-parse HEAD
run_cmd 5 git -C "$WS_ROOT" status --short --branch
run_cmd 5 git -C "$WS_ROOT" log -1 --oneline --decorate
run_cmd 8 ros2 pkg prefix n01_live_navigation
run_cmd 8 ros2 pkg prefix scan_planner
run_cmd 8 ros2 pkg prefix scan_planner_msgs
run_cmd 8 ros2 pkg prefix n01_static_map

section "3. Processes, launch arguments, PCD paths and duplicate processes"
run_cmd 8 bash -lc "ps -eo pid,ppid,etimes,args | grep -E '[s]lam3d_node|[s]can_planner_node|n01_live_|[r]os2 launch|[r]viz2|[s]tatic_map_node'"
printf '\nPotential obsolete FROZEN path references in active process arguments:\n'
ps -eo args | grep -E '[F]ROZEN|[s]lam3d_node|[s]can_planner_node|n01_live_' || true
printf '\nPCD files available under ~/lyy/slam_data/maps:\n'
if [ -d "$LYY_ROOT/slam_data/maps" ]; then
  find "$LYY_ROOT/slam_data/maps" -maxdepth 1 -type f -name '*.pcd' -printf '%p | %s bytes | %TY-%Tm-%Td %TH:%TM\n' 2>&1
else
  printf 'MAP_DIRECTORY_MISSING: %s/slam_data/maps\n' "$LYY_ROOT"
fi
printf '\nBag directories available under ~/lyy/slam_data/bags:\n'
if [ -d "$LYY_ROOT/slam_data/bags" ]; then
  find "$LYY_ROOT/slam_data/bags" -mindepth 1 -maxdepth 1 -type d -printf '%p\n' 2>&1
fi

section "4. ROS graph snapshot"
NODE_LIST="$(timeout 12s ros2 node list 2>&1)"
printf '%s\n' "$NODE_LIST"
printf '\nDuplicate node names:\n'
printf '%s\n' "$NODE_LIST" | grep '^/' | sort | uniq -c | awk '$1 > 1 {print}' || true

TOPIC_LIST="$(timeout 12s ros2 topic list 2>&1)"
TOPIC_LIST_TYPES="$(timeout 12s ros2 topic list -t 2>&1)"
printf '\nAll topics and types:\n%s\n' "$TOPIC_LIST_TYPES"

printf '\nRelevant topic names (localization / mapping / planning / safety / chassis):\n'
printf '%s\n' "$TOPIC_LIST_TYPES" | grep -Ei 'pose|odom|imu|livox|cloud|grid_map|occupancy|dynamic|obstacle|planning|bspline|trajectory|feasibility|navigation|readiness|health|safety|operator|goal|cmd|chassis|wheel|steer|joint_states' || true

section "5. ROS services and actions (listed only; no service/action is invoked)"
run_cmd 10 ros2 service list -t
run_cmd 8 ros2 action list -t

section "6. A - localization, IMU/LiDAR time alignment and body pose"
printf 'Expected A-chain: SLAM /robot_pose -> live scan bridge -> /body_pose -> goal gate\n'
node_info /slam3d_node
node_info /n01_live_scan_bridge
node_info /n01_live_scan_goal_gate

printf '\nA-chain topic endpoint details:\n'
topic_info /localization_status
topic_info /robot_pose
topic_info /body_pose
topic_info /odom
topic_info /imu/data
topic_info /livox/lidar/geo_filtered
topic_info /livox/lidar/geo_filter_azimuth
topic_info /tf
topic_info /tf_static

printf '\nA-chain one-shot messages:\n'
topic_echo /localization_status
topic_echo /robot_pose
topic_echo /body_pose
topic_echo /odom
topic_echo /imu/data
topic_echo /n01/live_navigation/readiness
topic_echo /n01/live_navigation/navigation_ready
topic_echo /n01/live_navigation/system_health

printf '\nA-chain frequency samples:\n'
topic_hz /robot_pose
topic_hz /localization_status
topic_hz /body_pose
topic_hz /imu/data
topic_hz /livox/lidar/geo_filtered

section "7. B - static PCD overlay, dynamic obstacles and native SCAN planner"
node_info /n01_live_static_pcd
node_info /scan_planner_node
node_info /n01_live_dynamic_obstacle_tracker
printf '\nB-chain topic endpoint details:\n'
topic_info /grid_map/static_occupancy
topic_info /grid_map/static_overlay_status
topic_info /grid_map/static_overlay_ready
topic_info /n01/live_navigation/dynamic_obstacle_status
topic_info /planning/bspline
topic_info /n01/live_navigation/scan_status
topic_info /n01/live_navigation/planner_status

printf '\nB-chain one-shot messages:\n'
topic_echo /grid_map/static_overlay_status --qos-durability transient_local
topic_echo /grid_map/static_overlay_ready --qos-durability transient_local
topic_echo /n01/live_navigation/dynamic_obstacle_status
topic_echo /n01/live_navigation/scan_status
topic_echo /n01/live_navigation/planner_status

printf '\nNative planner output sample:\n'
topic_echo /planning/bspline
printf '\nNative planner frequency sample:\n'
topic_hz /planning/bspline

section "8. C - Hive2 four-wheel-steering kinematics and trajectory feasibility"
node_info /n01_hive2_trajectory_monitor
printf '\nC-chain relevant topic endpoints:\n'
topic_info /n01/live_navigation/hive2_feasibility
topic_info /n01/live_navigation/trajectory_status
topic_info /planning/bspline
topic_info /joint_states
topic_info /chassis_controller/rt_command
topic_info /cmd_vel
topic_info /cmd_wheel
printf '\nC-chain one-shot messages:\n'
topic_echo /n01/live_navigation/hive2_feasibility
topic_echo /n01/live_navigation/trajectory_status
topic_echo /joint_states

section "9. D - safety state machine, operator hold and actuator fail-closed chain"
node_info /n01_live_safety_supervisor
node_info /n01_live_runtime_supervisor
node_info /n01_live_scan_goal_gate
printf '\nD-chain one-shot messages:\n'
topic_echo /n01/live_navigation/safety_status
topic_echo /n01/live_navigation/navigation_ready
topic_echo /n01/live_navigation/system_health
printf '\nControl-chain endpoint topology (inspection only):\n'
topic_info /cmd_vel
topic_info /cmd_wheel
topic_info /chassis_controller/rt_command
topic_info /chassis_controller/command
topic_info /rt_command

section "10. Relevant runtime parameters (read only)"
node_params /slam3d_node
node_params /n01_live_scan_bridge
node_params /n01_live_runtime_supervisor
node_params /n01_live_safety_supervisor
node_params /n01_live_dynamic_obstacle_tracker
node_params /n01_hive2_trajectory_monitor
node_params /n01_live_scan_goal_gate
node_params /n01_live_static_pcd
node_params /scan_planner_node

section "11. Message interface contracts"
run_cmd 8 ros2 interface show hivecore_slam3d/msg/LocalizationStatus
run_cmd 8 ros2 interface show scan_planner_msgs/msg/Bspline
run_cmd 8 ros2 interface show interfaces/msg/ChassisRtCommand
run_cmd 8 ros2 interface show nav_msgs/msg/Odometry

section "12. TF lookup samples (read only)"
run_cmd 5 ros2 run tf2_ros tf2_echo map base_footprint
run_cmd 5 ros2 run tf2_ros tf2_echo base_footprint lidar_link

section "13. Relevant source-code references in this checkout"
if [ -d "$WS_ROOT/src" ]; then
  grep -RInE 'pose_publish_period|imu_cloud_max_stamp_delta_sec|fusion_path_imu_sync_ok|body_pose|static_overlay_status|dynamic_obstacle_status|hive2_feasibility|operator_hold|navigation_ready|subscriptions' \
    "$WS_ROOT/src/n01_live_navigation" "$WS_ROOT/src/n01_static_map" "$WS_ROOT/src/scan_planner" \
    --include='*.py' --include='*.cpp' --include='*.hpp' --include='*.h' --include='*.yaml' --include='*.yml' \
    2>/dev/null | head -n 240 || true
else
  printf 'SOURCE_DIRECTORY_MISSING: %s/src\n' "$WS_ROOT"
fi

section "14. Vendor SLAM source archive inspection"
SLAM_ARCHIVE="$LYY_ROOT/hive2_slam_sources_20261030_155649.tar.gz"
if [ -f "$SLAM_ARCHIVE" ]; then
  printf 'archive: %s\n' "$SLAM_ARCHIVE"
  printf '\nRelevant source/config member names:\n'
  MEMBERS="$(tar -tzf "$SLAM_ARCHIVE" 2>/dev/null | grep -Ei '(slam3d|localiz|mola|pose|fusion).*\.(cpp|cc|cxx|hpp|h|yaml|yml)$' | head -n 100)"
  printf '%s\n' "$MEMBERS"
  printf '\nSource matches for pose publishing and IMU synchronization:\n'
  while IFS= read -r member; do
    [ -z "$member" ] && continue
    matches="$(tar -xOzf "$SLAM_ARCHIVE" "$member" 2>/dev/null | grep -n -C 3 -E 'pose_publish_period|publish_pose|pose_topic|fusion_path_imu_sync_ok|imu_cloud_max_stamp_delta_sec|imu_cloud_stamp_delta' | head -n 100 || true)"
    if [ -n "$matches" ]; then
      printf '\n--- %s ---\n%s\n' "$member" "$matches"
    fi
  done <<< "$MEMBERS"
else
  printf 'SLAM_SOURCE_ARCHIVE_MISSING: %s\n' "$SLAM_ARCHIVE"
fi

section "15. Existing N01 diagnostic and runbook artifacts"
for file in \
  "$WS_ROOT/scripts/n01_live_stack_preflight.py" \
  "$WS_ROOT/scripts/n01_live_localization_readonly_audit.py" \
  "$WS_ROOT/docs/n01_four_stage_navigation_delivery.md" \
  "$WS_ROOT/docs/n01_live_navigation_full_stack.md" \
  "$WS_ROOT/docs/n01_g415_native_scan_static_pcd_integration.md"
do
  if [ -f "$file" ]; then
    ls -lh "$file"
  else
    printf 'MISSING: %s\n' "$file"
  fi
done

section "16. Diagnostic completeness checklist"
printf 'Collected: host/time/ROS env, git revision, active processes, ROS nodes/topics/services,\n'
printf 'A localization+IMU+body pose, B static map+dynamic obstacle+SCAN, C Hive2 feasibility,\n'
printf 'D safety/control topology, live params, message contracts, TF samples, local source refs,\n'
printf 'and vendor SLAM archive source matches when the archive is present.\n'
printf 'No ROS command topic was published. No ROS service/action was called.\n'
printf 'No ROS parameter was changed. No node was stopped or restarted. No build/test was run.\n'
