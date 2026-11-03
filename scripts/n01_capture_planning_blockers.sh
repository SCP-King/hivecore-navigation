#!/usr/bin/env bash
# Focused, read-only capture for the first N-01 planning blockers.
set -o pipefail

WS_ROOT="${WS_ROOT:-$HOME/lyy/hivecore-navigation}"
PCD_PATH="${PCD_PATH:-$HOME/lyy/slam_data/maps/cloud_map_20261027_225549.pcd}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BASE_DIR="$HOME/lyy/n01_runtime/runs"
OUT_DIR="$BASE_DIR/planning_blockers_$STAMP"
BUNDLE="$BASE_DIR/n01_planning_blockers_$STAMP.tar.gz"
mkdir -p "$OUT_DIR" || exit 1

exec 3>&1 4>&2
exec > "$OUT_DIR/capture.log" 2>&1

echo "N-01 focused planning blocker capture"
echo "timestamp=$(date --iso-8601=ns)"
echo "workspace=$WS_ROOT"
echo "pcd_path=$PCD_PATH"
echo "output_dir=$OUT_DIR"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/setup.bash"
fi

# Prefer the exact PCD path used by the running static-map node over the
# historical default. The parameter read is diagnostic-only.
if command -v ros2 >/dev/null 2>&1; then
  LIVE_PCD_PATH="$(timeout 5s ros2 param get /n01_live_static_pcd pcd_path 2>/dev/null | sed -n 's/^String value is: //p' | tail -n 1)"
  if [ -n "$LIVE_PCD_PATH" ]; then
    PCD_PATH="$LIVE_PCD_PATH"
  fi
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 command unavailable after sourcing ROS/workspace setup"
else
  capture() {
    local name="$1"
    shift
    {
      echo "### command: $*"
      timeout --signal=INT --kill-after=2s 12s "$@"
      local rc=$?
      echo
      echo "### exit_code=$rc"
    } > "$OUT_DIR/$name.txt" 2>&1
    echo "SAVED: $OUT_DIR/$name.txt"
  }

  capture nodes ros2 node list
  capture topics ros2 topic list -t
  capture static_overlay_status ros2 topic echo /grid_map/static_overlay_status --once --qos-durability transient_local
  capture static_overlay_ready ros2 topic echo /grid_map/static_overlay_ready --once --qos-durability transient_local
  capture system_health ros2 topic echo /n01/live_navigation/system_health --once
  capture bridge_readiness ros2 topic echo /n01/live_navigation/readiness --once
  capture localization_status ros2 topic echo /localization_status --once
  capture robot_pose ros2 topic echo /robot_pose --once --qos-reliability best_effort
  capture body_pose ros2 topic echo /body_pose --once --qos-reliability best_effort
  capture scan_body_pose ros2 topic echo /n01/live_navigation/scan_body_pose --once --qos-reliability best_effort
  capture sensor_pose ros2 topic echo /sensor_pose --once --qos-reliability best_effort

  capture static_map_node_info ros2 node info /n01_live_static_pcd
  capture scan_planner_node_info ros2 node info /scan_planner_node
  capture bridge_node_info ros2 node info /n01_live_scan_bridge
  capture runtime_supervisor_node_info ros2 node info /n01_live_runtime_supervisor
  capture goal_gate_node_info ros2 node info /n01_live_scan_goal_gate
  capture static_cloud_endpoints ros2 topic info -v /grid_map/static_occupancy
  capture planner_cloud_endpoints ros2 topic info -v /cloud
  capture sensor_pose_endpoints ros2 topic info -v /sensor_pose
  capture scan_body_pose_endpoints ros2 topic info -v /n01/live_navigation/scan_body_pose
  capture static_status_endpoints ros2 topic info -v /grid_map/static_overlay_status
  capture robot_pose_endpoints ros2 topic info -v /robot_pose
  capture body_pose_endpoints ros2 topic info -v /body_pose
  capture scan_cloud_endpoints ros2 topic info -v /livox/lidar/geo_filtered
  capture planner_params ros2 param dump /scan_planner_node
  capture static_map_params ros2 param dump /n01_live_static_pcd
  capture bridge_params ros2 param dump /n01_live_scan_bridge
fi

{
  echo "### active processes"
  ps -eo pid,ppid,etimes,args | grep -E '[s]can_planner_node|n01_live_|[s]lam3d_node|[r]os2 launch' || true
} > "$OUT_DIR/processes.txt"
echo "SAVED: $OUT_DIR/processes.txt"

{
  echo "### PCD file"
  if [ -r "$PCD_PATH" ]; then
    stat "$PCD_PATH"
    sha256sum "$PCD_PATH"
    file "$PCD_PATH"
    python3 - "$PCD_PATH" <<'PY'
import sys
path = sys.argv[1]
print("### PCD header")
with open(path, "rb") as f:
    for raw in f:
        line = raw.decode("latin-1", "replace").rstrip("\r\n")
        print(line)
        if line.strip().startswith("DATA"):
            break
PY
  else
    echo "PCD_NOT_READABLE_OR_MISSING: $PCD_PATH"
  fi
} > "$OUT_DIR/pcd_file.txt" 2>&1
echo "SAVED: $OUT_DIR/pcd_file.txt"

{
  echo "### workspace revision"
  git -C "$WS_ROOT" branch --show-current 2>&1 || true
  git -C "$WS_ROOT" rev-parse HEAD 2>&1 || true
  git -C "$WS_ROOT" status --short --branch 2>&1 || true
} > "$OUT_DIR/workspace_revision.txt"
echo "SAVED: $OUT_DIR/workspace_revision.txt"

# Close the capture log before archiving so tar receives a complete,
# stable file rather than a log that is still being appended.
exec 1>&3 2>&4
exec 3>&- 4>&-
if tar -czf "$BUNDLE" -C "$BASE_DIR" "$(basename "$OUT_DIR")"; then
  printf "BUNDLE_FILE: %s\n" "$BUNDLE"
  sha256sum "$BUNDLE"
else
  printf "ERROR: failed to create bundle\n" >&2
  exit 1
fi
