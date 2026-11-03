#!/usr/bin/env bash
# N-01 comprehensive planning-readiness audit.
# READ ONLY: subscribes/inspects only; no goal, motion command, parameter write,
# service/action invocation, node restart, launch, build, or controller operation.

umask 077
set +e

WS_ROOT="$(cd "$(dirname "$0")/.." 2>/dev/null && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$HOME/lyy/n01_runtime/runs/planning_readiness_audit"
mkdir -p "$OUT_DIR" || exit 2
LOG_FILE="$OUT_DIR/n01_planning_readiness_audit_$STAMP.log"
PROBE_JSON="$OUT_DIR/n01_planning_ros_probe_$STAMP.json"
BUNDLE_FILE="$OUT_DIR/n01_planning_readiness_bundle_$STAMP.tar.gz"
exec > >(tee -a "$LOG_FILE") 2>&1

section() {
  printf '\n\n================================================================\n%s\n================================================================\n' "$1"
}

run_cmd() {
  seconds="$1"
  shift
  printf '\n$'
  printf ' %q' "$@"
  printf '\n'
  timeout --signal=INT --kill-after=2s "${seconds}s" "$@"
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
  if ! has_topic "$topic"; then
    printf '\nTOPIC_MISSING: %s\n' "$topic"
    return 0
  fi
  printf '\n$ ros2 topic info %s --verbose\n' "$topic"
  output="$(timeout --signal=INT --kill-after=2s 7s ros2 topic info "$topic" --verbose 2>&1)"
  rc=$?
  printf '%s\n' "$output"
  pubs="$(printf '%s\n' "$output" | grep -c 'Endpoint type: PUBLISHER' 2>/dev/null)"
  subs="$(printf '%s\n' "$output" | grep -c 'Endpoint type: SUBSCRIPTION' 2>/dev/null)"
  printf 'ENDPOINT_COUNT publisher=%s subscription=%s exit_code=%s\n' "${pubs:-0}" "${subs:-0}" "$rc"
}

topic_echo() {
  topic="$1"
  qos="$2"
  if ! has_topic "$topic"; then
    printf '\nTOPIC_MISSING: %s\n' "$topic"
    return 0
  fi
  case "$qos" in
    sensor)
      run_cmd 4 ros2 topic echo "$topic" --once --qos-reliability best_effort
      ;;
    latched)
      run_cmd 4 ros2 topic echo "$topic" --once --qos-durability transient_local
      ;;
    *)
      run_cmd 4 ros2 topic echo "$topic" --once
      ;;
  esac
}

topic_hz() {
  topic="$1"
  qos="$2"
  if ! has_topic "$topic"; then
    printf '\nTOPIC_MISSING: %s\n' "$topic"
    return 0
  fi
  if [ "$qos" = "sensor" ]; then
    run_cmd 4 ros2 topic hz "$topic" --qos-reliability best_effort
  else
    run_cmd 4 ros2 topic hz "$topic"
  fi
}

node_info() {
  node="$1"
  if has_node "$node"; then
    run_cmd 8 ros2 node info "$node"
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

param_value() {
  node="$1"
  name="$2"
  raw="$(timeout 6s ros2 param get "$node" "$name" 2>&1)"
  printf '%s\n' "$raw" | sed -n 's/^[^:]* value is: //p' | tail -n 1
}

finish() {
  printf '\n================================================================\n'
  printf 'N-01 planning-readiness audit finished: %s\n' "$(date --iso-8601=seconds)"
  printf 'LOG_FILE: %s\n' "$LOG_FILE"
  if [ -f "$PROBE_JSON" ]; then
    printf 'ROS_PROBE_JSON: %s\n' "$PROBE_JSON"
    tar -czf "$BUNDLE_FILE" -C "$OUT_DIR" "$(basename "$LOG_FILE")" "$(basename "$PROBE_JSON")" 2>/dev/null
    if [ -f "$BUNDLE_FILE" ]; then
      printf 'BUNDLE_FILE: %s\n' "$BUNDLE_FILE"
    fi
  fi
  printf 'READ_ONLY: no goal published; no actuator command published; no parameter writes; no service/action calls; no node restarts\n'
  printf '================================================================\n'
}
trap finish EXIT

section "0. Audit identity, host clock, load and storage"
printf 'timestamp: %s\n' "$(date --iso-8601=ns)"
printf 'hostname: %s\n' "$(hostname)"
printf 'workspace: %s\n' "$WS_ROOT"
printf 'user: %s\n' "$(id)"
printf 'ROS_DISTRO(before source)=%s\n' "$(printenv ROS_DISTRO 2>/dev/null || echo unset)"
printf 'ROS_DOMAIN_ID(before source)=%s\n' "$(printenv ROS_DOMAIN_ID 2>/dev/null || echo unset)"
printf 'RMW_IMPLEMENTATION(before source)=%s\n' "$(printenv RMW_IMPLEMENTATION 2>/dev/null || echo default)"
printf 'ROS_LOCALHOST_ONLY(before source)=%s\n' "$(printenv ROS_LOCALHOST_ONLY 2>/dev/null || echo unset)"
run_cmd 5 uname -a
run_cmd 6 timedatectl status
run_cmd 5 uptime
run_cmd 5 free -h
run_cmd 5 df -h "$HOME"
if command -v chronyc >/dev/null 2>&1; then
  run_cmd 5 chronyc tracking
fi

section "1. Source ROS Humble and this exact workspace overlay"
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  printf 'SOURCED: /opt/ros/humble/setup.bash\n'
else
  printf 'ERROR: /opt/ros/humble/setup.bash missing\n'
fi
if [ -f "$WS_ROOT/install/local_setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/local_setup.bash"
  printf 'SOURCED: %s/install/local_setup.bash\n' "$WS_ROOT"
else
  printf 'WARNING: install/local_setup.bash missing\n'
fi
printf 'ROS_DISTRO(after source)=%s\n' "$(printenv ROS_DISTRO 2>/dev/null || echo unset)"
printf 'ROS_DOMAIN_ID(after source)=%s\n' "$(printenv ROS_DOMAIN_ID 2>/dev/null || echo unset)"
printf 'RMW_IMPLEMENTATION(after source)=%s\n' "$(printenv RMW_IMPLEMENTATION 2>/dev/null || echo default)"
printf 'ROS_LOCALHOST_ONLY(after source)=%s\n' "$(printenv ROS_LOCALHOST_ONLY 2>/dev/null || echo unset)"
printf 'AMENT_PREFIX_PATH=%s\n' "$(printenv AMENT_PREFIX_PATH 2>/dev/null || echo unset)"
run_cmd 8 git -C "$WS_ROOT" branch --show-current
run_cmd 8 git -C "$WS_ROOT" rev-parse HEAD
run_cmd 8 git -C "$WS_ROOT" status --short --branch
run_cmd 8 git -C "$WS_ROOT" log -1 --oneline --decorate
for pkg in n01_live_navigation n01_static_map scan_planner scan_planner_msgs n01_scan_dynacbf_bridge; do
  run_cmd 6 ros2 pkg prefix "$pkg"
done
run_cmd 8 ros2 pkg executables n01_live_navigation
run_cmd 8 ros2 pkg executables n01_static_map
run_cmd 8 ros2 pkg executables scan_planner

section "2. Running processes, duplicate launch stacks and workspace paths"
run_cmd 8 bash -c "ps -eo pid,ppid,etimes,args | grep -E '[s]lam3d_node|[s]can_planner_node|n01_live_|n01_hive2_|[r]os2 launch|[r]viz2'"
printf '\nPotential stale workspace path references in active process command lines:\n'
ps -eo pid,args | grep -E '[F]ROZEN|[h]ivecore-navigation|[s]lam3d_node|[s]can_planner_node|n01_live_' || true

section "3. ROS graph snapshot: nodes, topics, services and actions (listed only)"
sleep 2
NODE_LIST="$(timeout 12s ros2 node list 2>&1)"
TOPIC_LIST="$(timeout 12s ros2 topic list 2>&1)"
printf 'NODES:\n%s\n' "$NODE_LIST"
printf '\nDUPLICATE NODE NAMES:\n'
printf '%s\n' "$NODE_LIST" | grep '^/' | sort | uniq -c | awk '$1 > 1 {print}' || true
printf '\nTOPICS AND TYPES:\n'
timeout 12s ros2 topic list -t 2>&1
printf '\nSERVICES (list only):\n'
timeout 10s ros2 service list -t 2>&1
printf '\nACTIONS (list only):\n'
timeout 10s ros2 action list -t 2>&1
run_cmd 15 ros2 doctor --report

section "4. Required node and topic presence matrix"
for node in \
  /slam3d_node \
  /n01_live_scan_bridge \
  /n01_live_static_pcd \
  /n01_live_runtime_supervisor \
  /n01_live_dynamic_obstacle_tracker \
  /n01_hive2_trajectory_monitor \
  /n01_live_scan_goal_gate \
  /n01_live_safety_supervisor \
  /scan_planner_node
do
  if has_node "$node"; then printf 'NODE_PRESENT %s\n' "$node"; else printf 'NODE_MISSING %s\n' "$node"; fi
done
for topic in \
  /localization_status /robot_pose /odom /imu/data /livox/lidar/geo_filtered \
  /cloud /sensor_pose /body_pose /n01/live_navigation/scan_body_pose \
  /grid_map/static_occupancy /grid_map/static_overlay_ready /grid_map/static_overlay_status \
  /grid_map/occupancy /grid_map/occupancy_inflate /grid_map/unknown /grid_map/sliding_map_bbox /mapping_processing_timings /n01/live_navigation/navigation_ready \
  /n01/live_navigation/readiness /n01/live_navigation/system_health \
  /n01/live_navigation/goal_gate_status /move_base_simple/goal \
  /n01/live_navigation/goal_gated /planning/bspline /planning/data_display \
  /n01/live_navigation/dynamic_obstacle_status \
  /n01/live_navigation/dynamic_obstacle_candidates /n01/live_navigation/dynamic_obstacle_tracks \
  /n01/live_navigation/hive2_feasibility /n01/live_navigation/hive2_candidate_path \
  /n01/live_navigation/hive2_wheel_kinematics /joint_states /planning/go2_execution_frozen \
  /n01/live_navigation/operator_hold /n01/live_navigation/safety_status
do
  if has_topic "$topic"; then printf 'TOPIC_PRESENT %s\n' "$topic"; else printf 'TOPIC_MISSING %s\n' "$topic"; fi
done

section "5. Publisher/subscriber endpoint topology and QoS"
for topic in \
  /localization_status /robot_pose /odom /imu/data /livox/lidar/geo_filtered \
  /cloud /sensor_pose /body_pose /n01/live_navigation/scan_body_pose \
  /tf /tf_static \
  /grid_map/static_occupancy /grid_map/static_overlay_ready /grid_map/static_overlay_status \
  /grid_map/occupancy /grid_map/occupancy_inflate /grid_map/unknown \
  /mapping_processing_timings /n01/live_navigation/navigation_ready \
  /move_base_simple/goal /n01/live_navigation/goal_gated \
  /planning/bspline /planning/data_display \
  /n01/live_navigation/dynamic_obstacle_status \
  /n01/live_navigation/dynamic_obstacle_candidates \
  /n01/live_navigation/hive2_feasibility /n01/live_navigation/hive2_candidate_path /n01/live_navigation/hive2_wheel_kinematics /joint_states /planning/go2_execution_frozen \
  /n01/live_navigation/system_health /n01/live_navigation/goal_gate_status \
  /n01/live_navigation/operator_hold /n01/live_navigation/safety_status \
  /cmd_vel /cmd_wheel /chassis_controller/rt_command \
  /chassis_controller/command /rt_command
do
  topic_info "$topic"
done

section "6. Latest status snapshots (read only; no point-cloud payload dump)"
topic_echo /localization_status sensor
topic_echo /n01/live_navigation/readiness reliable
topic_echo /n01/live_navigation/navigation_ready reliable
topic_echo /mapping_processing_timings reliable
topic_echo /planning/go2_execution_frozen reliable
topic_echo /grid_map/static_overlay_ready latched
topic_echo /grid_map/static_overlay_status latched
topic_echo /n01/live_navigation/dynamic_obstacle_status reliable
topic_echo /n01/live_navigation/system_health reliable
topic_echo /n01/live_navigation/goal_gate_status reliable
topic_echo /n01/live_navigation/hive2_feasibility reliable
topic_echo /n01/live_navigation/operator_hold reliable
topic_echo /n01/live_navigation/safety_status reliable
topic_echo /planning/bspline reliable
topic_echo /planning/data_display reliable

section "7. Relevant node wiring"
for node in \
  /slam3d_node \
  /n01_live_scan_bridge \
  /n01_live_static_pcd \
  /scan_planner_node \
  /n01_live_runtime_supervisor \
  /n01_live_dynamic_obstacle_tracker \
  /n01_hive2_trajectory_monitor \
  /n01_live_scan_goal_gate \
  /n01_live_safety_supervisor
do
  node_info "$node"
done

section "8. Runtime parameters for every node in the planning chain"
for node in \
  /slam3d_node \
  /n01_live_scan_bridge \
  /n01_live_static_pcd \
  /scan_planner_node \
  /n01_live_runtime_supervisor \
  /n01_live_dynamic_obstacle_tracker \
  /n01_hive2_trajectory_monitor \
  /n01_live_scan_goal_gate \
  /n01_live_safety_supervisor
do
  node_params "$node"
done

section "9. Static PCD source file: actual runtime path, parser contract and bounds"
PCD_PATH="$(param_value /n01_live_static_pcd pcd_path)"
PCD_RES="$(param_value /n01_live_static_pcd resolution)"
PCD_MIN_Z="$(param_value /n01_live_static_pcd min_z)"
PCD_MAX_Z="$(param_value /n01_live_static_pcd max_z)"
if [ -z "$PCD_PATH" ] && [ -f "$WS_ROOT/src/n01_static_map/config/static_map.yaml" ]; then
  PCD_PATH="$(sed -n 's/^[[:space:]]*pcd_path:[[:space:]]*"\{0,1\}\([^"]*\)"\{0,1\}[[:space:]]*$/\1/p' "$WS_ROOT/src/n01_static_map/config/static_map.yaml" | head -n 1)"
fi
printf 'RUNTIME_PCD_PATH=%s\n' "${PCD_PATH:-NOT_REPORTED}"
printf 'RUNTIME_PCD_RESOLUTION=%s\n' "${PCD_RES:-NOT_REPORTED}"
printf 'RUNTIME_PCD_MIN_Z=%s\n' "${PCD_MIN_Z:-NOT_REPORTED}"
printf 'RUNTIME_PCD_MAX_Z=%s\n' "${PCD_MAX_Z:-NOT_REPORTED}"
if [ -n "$PCD_PATH" ]; then
  run_cmd 5 file "$PCD_PATH"
  run_cmd 5 stat "$PCD_PATH"
  run_cmd 8 sha256sum "$PCD_PATH"
  PCD_PATH="$PCD_PATH" PCD_RES="$PCD_RES" PCD_MIN_Z="$PCD_MIN_Z" PCD_MAX_Z="$PCD_MAX_Z" python3 - <<'PY'
import os, sys
path = os.environ.get("PCD_PATH", "")
print("PCD_AUDIT_PATH:", path)
if not path or not os.path.isfile(path):
    print("PCD_FILE_EXISTS: false")
    raise SystemExit(0)
print("PCD_FILE_EXISTS: true")
try:
    import numpy as np
except Exception as exc:
    print("PCD_NUMPY_IMPORT_ERROR:", exc)
    raise SystemExit(0)
header = {}
data_kind = ""
with open(path, "rb") as stream:
    while True:
        raw = stream.readline()
        if not raw:
            break
        line = raw.decode("ascii", errors="replace").strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        header[parts[0].upper()] = parts[1:]
        if parts[0].upper() == "DATA":
            data_kind = parts[1].lower() if len(parts) > 1 else ""
            break
    data_offset = stream.tell()
    fields = header.get("FIELDS", [])
    sizes = [int(x) for x in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(x) for x in header.get("COUNT", ["1"] * len(fields))]
    points = int(header.get("POINTS", ["0"])[0])
    width = int(header.get("WIDTH", [str(points)])[0])
    height = int(header.get("HEIGHT", ["1"])[0])
    file_size = os.path.getsize(path)
    print("PCD_HEADER:", {k: v for k, v in header.items() if k in ("VERSION","FIELDS","SIZE","TYPE","COUNT","WIDTH","HEIGHT","POINTS","DATA")})
    print("PCD_DATA_KIND:", data_kind)
    print("PCD_DATA_OFFSET_BYTES:", data_offset)
    print("PCD_FILE_SIZE_BYTES:", file_size)
    print("PCD_HEADER_POINTS:", points)
    print("PCD_XYZ_BINARY_CONTRACT:", bool(
        data_kind == "binary" and fields == ["x","y","z"]
        and sizes == [4,4,4] and types == ["F","F","F"] and counts == [1,1,1]
    ))
    if not (data_kind == "binary" and fields == ["x","y","z"]
            and sizes == [4,4,4] and types == ["F","F","F"] and counts == [1,1,1]):
        print("PCD_PARSE_COMPATIBLE_WITH_n01_static_map: false")
        raise SystemExit(0)
    expected = points * 12
    available = max(0, file_size - data_offset)
    print("PCD_EXPECTED_PAYLOAD_BYTES:", expected)
    print("PCD_AVAILABLE_PAYLOAD_BYTES:", available)
    if points <= 0 or available < expected:
        print("PCD_PAYLOAD_COMPLETE: false")
        raise SystemExit(0)
    finite_count = 0
    accepted_z_count = 0
    mins = np.array([np.inf, np.inf, np.inf], dtype=np.float64)
    maxs = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64)
    zmin = float(os.environ["PCD_MIN_Z"]) if os.environ.get("PCD_MIN_Z") else None
    zmax = float(os.environ["PCD_MAX_Z"]) if os.environ.get("PCD_MAX_Z") else None
    remaining = points
    while remaining:
        count = min(250000, remaining)
        xyz = np.fromfile(stream, dtype="<f4", count=count * 3)
        if xyz.size != count * 3:
            break
        xyz = xyz.reshape((-1,3))
        finite = np.isfinite(xyz).all(axis=1)
        good = xyz[finite]
        finite_count += int(len(good))
        if len(good):
            mins = np.minimum(mins, good.min(axis=0))
            maxs = np.maximum(maxs, good.max(axis=0))
            if zmin is not None and zmax is not None:
                accepted_z_count += int(((good[:,2] >= zmin) & (good[:,2] <= zmax)).sum())
        remaining -= count
    print("PCD_PAYLOAD_COMPLETE:", remaining == 0)
    print("PCD_FINITE_POINTS:", finite_count)
    if finite_count:
        print("PCD_BOUNDS_MIN_XYZ:", mins.tolist())
        print("PCD_BOUNDS_MAX_XYZ:", maxs.tolist())
    if zmin is not None and zmax is not None:
        print("PCD_CONFIGURED_Z_WINDOW:", [zmin, zmax])
        print("PCD_FINITE_POINTS_IN_Z_WINDOW:", accepted_z_count)
        print("PCD_FINITE_POINTS_OUTSIDE_Z_WINDOW:", finite_count - accepted_z_count)
    print("PCD_PARSE_COMPATIBLE_WITH_n01_static_map: true")
    print("PCD_FRAME_ALIGNMENT_PROOF: NOT_ENCODED_IN_STANDARD_PCD_HEADER; compare runtime map status and SLAM map contract")
PY
else
  printf 'PCD_PATH_UNAVAILABLE: static map node parameter and local YAML did not provide a path\n'
fi
printf '\nCandidate map files:\n'
if [ -d "$HOME/lyy/slam_data/maps" ]; then
  find "$HOME/lyy/slam_data/maps" -maxdepth 1 -type f -name '*.pcd' -printf '%TY-%Tm-%Td %TH:%TM:%TS | %s bytes | %p\n' | sort -r | head -n 20
else
  printf 'MAP_DIRECTORY_MISSING\n'
fi

section "10. TF frame-chain checks (direct map->base may be optional; sensor mount is required)"
run_cmd 5 ros2 run tf2_ros tf2_echo base_footprint lidar_link
run_cmd 5 ros2 run tf2_ros tf2_echo odom base_link
run_cmd 5 ros2 run tf2_ros tf2_echo map odom
run_cmd 5 ros2 run tf2_ros tf2_echo map base_footprint
if ros2 pkg prefix tf2_tools >/dev/null 2>&1; then
  cd "$OUT_DIR"
  run_cmd 9 ros2 run tf2_tools view_frames
  cd "$WS_ROOT"
else
  printf 'tf2_tools package unavailable; skipped frame graph PDF\n'
fi

section "11. Live sensor, cloud, pose, map and trajectory rates"
topic_hz /robot_pose sensor
topic_hz /odom sensor
topic_hz /imu/data sensor
topic_hz /livox/lidar/geo_filtered sensor
topic_hz /cloud sensor
topic_hz /sensor_pose sensor
topic_hz /body_pose sensor
topic_hz /grid_map/occupancy sensor
topic_hz /grid_map/occupancy_inflate sensor
topic_hz /planning/bspline reliable
topic_hz /joint_states sensor

section "12. Structured ROS live-data probe: cloud geometry, pose frames, IMU and exact stamp alignment"
if [ -f "$WS_ROOT/scripts/n01_live_planning_ros_probe.py" ]; then
  run_cmd 30 python3 "$WS_ROOT/scripts/n01_live_planning_ros_probe.py" --duration 8 --output "$PROBE_JSON"
else
  printf 'ROS_PROBE_SCRIPT_MISSING: %s/scripts/n01_live_planning_ros_probe.py\n' "$WS_ROOT"
fi

section "13. Native SCAN source contract and live launch/config consistency"
for file in \
  "$WS_ROOT/src/n01_live_navigation/launch/live_scan.launch.py" \
  "$WS_ROOT/src/n01_live_navigation/config/live_scan.yaml" \
  "$WS_ROOT/src/n01_live_navigation/n01_live_navigation/live_scan_bridge.py" \
  "$WS_ROOT/src/n01_live_navigation/n01_live_navigation/runtime_supervisor.py" \
  "$WS_ROOT/src/n01_live_navigation/n01_live_navigation/dynamic_obstacle_tracker.py" \
  "$WS_ROOT/src/n01_live_navigation/n01_live_navigation/trajectory_monitor.py" \
  "$WS_ROOT/src/n01_live_navigation/n01_live_navigation/goal_gate.py" \
  "$WS_ROOT/src/n01_live_navigation/n01_live_navigation/safety_supervisor.py" \
  "$WS_ROOT/src/n01_static_map/config/static_map.yaml" \
  "$WS_ROOT/src/scan_planner/plan_manage/config/planner.yaml" \
  "$WS_ROOT/src/scan_planner/plan_manage/src/scan_replan_fsm.cpp" \
  "$WS_ROOT/src/scan_planner/plan_env/src/grid_map.cpp"
do
  if [ -f "$file" ]; then printf '\n--- %s ---\n' "$file"; fi
done
grep -nE 'pcd_map_frame_asserted|static_cloud_enabled|static_cloud_topic|cloud_is_world|frame_id|body_pose|sensor_pose|goal_gated|move_base_simple/goal|planning/bspline|dynamic.*visualization_only|planner_input|operator_hold|actuator_output|safety_authorization|fusion_path_imu_sync_ok|pose_publish' \
  "$WS_ROOT/src/n01_live_navigation/launch/live_scan.launch.py" \
  "$WS_ROOT/src/n01_live_navigation/config/live_scan.yaml" \
  "$WS_ROOT/src/n01_live_navigation/n01_live_navigation/live_scan_bridge.py" \
  "$WS_ROOT/src/n01_live_navigation/n01_live_navigation/runtime_supervisor.py" \
  "$WS_ROOT/src/n01_live_navigation/n01_live_navigation/dynamic_obstacle_tracker.py" \
  "$WS_ROOT/src/n01_live_navigation/n01_live_navigation/trajectory_monitor.py" \
  "$WS_ROOT/src/n01_live_navigation/n01_live_navigation/goal_gate.py" \
  "$WS_ROOT/src/n01_live_navigation/n01_live_navigation/safety_supervisor.py" \
  "$WS_ROOT/src/n01_static_map/config/static_map.yaml" \
  "$WS_ROOT/src/scan_planner/plan_manage/config/planner.yaml" \
  "$WS_ROOT/src/scan_planner/plan_manage/src/scan_replan_fsm.cpp" \
  "$WS_ROOT/src/scan_planner/plan_env/src/grid_map.cpp" 2>/dev/null | head -n 260

section "14. ROS interface contracts"
run_cmd 6 ros2 interface show hivecore_slam3d/msg/LocalizationStatus
run_cmd 6 ros2 interface show scan_planner_msgs/msg/Bspline
run_cmd 6 ros2 interface show scan_planner_msgs/msg/DataDisp
run_cmd 6 ros2 interface show nav_msgs/msg/Odometry
run_cmd 6 ros2 interface show sensor_msgs/msg/PointCloud2
run_cmd 6 ros2 interface show sensor_msgs/msg/JointState

section "15. Existing bag evidence and vendor SLAM source archive"
if [ -d "$HOME/lyy/slam_data/bags" ]; then
  printf 'Available bag directories (newest first):\n'
  find "$HOME/lyy/slam_data/bags" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -nr | head -n 10
  LATEST_BAG="$(find "$HOME/lyy/slam_data/bags" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-)"
  if [ -n "$LATEST_BAG" ]; then
    printf '\nLatest bag metadata (read only): %s\n' "$LATEST_BAG"
    run_cmd 12 ros2 bag info "$LATEST_BAG"
  fi
else
  printf 'BAG_DIRECTORY_MISSING: %s\n' "$HOME/lyy/slam_data/bags"
fi
SLAM_ARCHIVE="$HOME/lyy/hive2_slam_sources_20261030_155649.tar.gz"
if [ -f "$SLAM_ARCHIVE" ]; then
  printf '\nVendor SLAM source archive present: %s\n' "$SLAM_ARCHIVE"
  run_cmd 8 bash -c "tar -tzf '$SLAM_ARCHIVE' | grep -Ei '(slam3d|localiz|mola|pose|fusion).*(cpp|cc|cxx|hpp|h|yaml|yml)$' | head -n 100"
else
  printf '\nVendor SLAM source archive not found at expected path: %s\n' "$SLAM_ARCHIVE"
fi

section "16. Planning-stage interpretation and completeness"
printf '%s\n' \
  'P0 planning prerequisites: localization/scan compensation, fresh /body_pose, exact /cloud + /sensor_pose synchronization, static PCD overlay in the active SCAN window, planner subscriptions and goal-gate health.' \
  'P1 candidate review: /planning/bspline validity, fresh /joint_states, Hive2 kinematic feasibility and dynamic-scene diagnostics.' \
  'P2 physical execution: separate acceptance only; this audit never authorizes execution and the safety supervisor is expected to remain fail-closed.' \
  'Important: /planning/bspline may be absent before a valid goal is sent; that alone is not a planner fault.' \
  'Important: the dynamic-obstacle tracker is visualization/diagnostics only; native SCAN consumes the map-frame /cloud directly plus the static PCD overlay.' \
  'Important: standard PCD files do not prove their coordinate registration; the launch map-frame assertion and runtime overlay bounds must be checked against the live SLAM map.' \
  'This audit intentionally does not send a test goal. Once this report is reviewed, the next step can be a single controlled planning-only goal with actuator outputs still disabled.'

printf '\nOUTPUTS:\n'
printf 'LOG_FILE=%s\n' "$LOG_FILE"
printf 'ROS_PROBE_JSON=%s\n' "$PROBE_JSON"
printf 'The script has not published /move_base_simple/goal, /goal_gated, /cmd_vel, /cmd_wheel or any chassis command.\n'
