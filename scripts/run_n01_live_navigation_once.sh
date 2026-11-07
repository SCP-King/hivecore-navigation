#!/usr/bin/env bash
# One-command N-01 live SCAN bring-up + read-only acceptance.
# Starts the static PCD layer and planning/perception stack, then audits live data.
# Never publishes goals or actuator commands; planner remains non-actuating.
set -Eeo pipefail
# ROS setup scripts reference optional unset variables; enable nounset only afterwards.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
RUNTIME_ROOT="${N01_RUNTIME_ROOT:-$HOME/lyy/n01_runtime}"
LOG_DIR="$RUNTIME_ROOT/logs"
RUN_DIR="$RUNTIME_ROOT/runs/live_navigation"
STAMP="$(date +%Y%m%d_%H%M%S)"
PCD="/opt/slam3d_release/resource/mapping_lite.pcd"
SLAM_CONFIG="/opt/slam3d_release/install/hivecore_slam3d_lite_localization/share/hivecore_slam3d_lite_localization/config/localization_lite.yaml"
BUILD_LOG="$LOG_DIR/n01_live_navigation_build_$STAMP.log"
LAUNCH_LOG="$LOG_DIR/n01_live_navigation_launch_$STAMP.log"
TF_LOG="$LOG_DIR/n01_live_navigation_tf_$STAMP.log"
REPORT="$RUN_DIR/n01_live_stack_preflight_$STAMP.json"
PID_FILE="$RUN_DIR/live_scan_launch.pid"

mkdir -p "$LOG_DIR" "$RUN_DIR"
cd "$ROOT"

source /opt/ros/humble/setup.bash

if [[ ! -f "$ROOT/install/setup.bash" ]]; then
  echo "ERROR: workspace install/setup.bash missing; build the workspace first."
  exit 2
fi
source "$ROOT/install/setup.bash"
set -u

if [[ ! -f "$PCD" || ! -r "$PCD" ]]; then
  echo "ERROR: active Lite PCD not readable: $PCD"
  exit 2
fi
if [[ ! -r "$SLAM_CONFIG" ]] || ! grep -Fq "map_path: $PCD" "$SLAM_CONFIG"; then
  echo "ERROR: cannot verify that $PCD is the map configured for the active Lite localization."
  echo "Config checked: $SLAM_CONFIG"
  exit 2
fi

echo "===== 1. REPOSITORY ====="
git branch --show-current
git log -1 --oneline
git status -sb

if [[ "$(git branch --show-current)" != "dev/n01-current" ]]; then
  echo "ERROR: switch to canonical development branch dev/n01-current first."
  exit 2
fi

echo
echo "===== 2. BUILD ====="
if ! colcon build --symlink-install \
    --packages-up-to n01_live_navigation scan_planner n01_static_map \
    --event-handlers console_direct+ 2>&1 | tee "$BUILD_LOG"; then
  echo "BUILD FAILED: $BUILD_LOG"
  exit 2
fi
# ROS/colcon setup scripts read optional variables that may be unset.
set +u
source "$ROOT/install/setup.bash"
set -u

echo
echo "===== 3. AUTOMATED TESTS ====="
colcon test --packages-select n01_live_navigation scan_planner n01_static_map 2>&1 | tee -a "$BUILD_LOG"
colcon test-result --verbose 2>&1 | tee -a "$BUILD_LOG"

echo
echo "===== 4. CHECK EXISTING NAVIGATION INSTANCES ====="
NODES="$(ros2 node list 2>&1)"
for node in /n01_live_scan_bridge /scan_planner_node /n01_static_map /n01_live_runtime_supervisor; do
  if grep -Fxq "$node" <<<"$NODES"; then
    echo "ERROR: $node already exists. Stop the previous N-01 live stack before retrying."
    exit 3
  fi
done

if grep -Fxq "/n01_localization_adapter" <<<"$NODES"; then
  START_ADAPTER=false
  echo "Existing /n01_localization_adapter detected: reuse it; do not create a duplicate."
else
  START_ADAPTER=true
  echo "No adapter detected: launch will start one."
fi

RSP_COUNT="$(grep -Fxc "/robot_state_publisher" <<<"$NODES" || true)"
if [[ "${RSP_COUNT:-0}" -gt 1 ]]; then
  echo "WARNING: $RSP_COUNT /robot_state_publisher nodes exist. They are not restarted by this launch; inspect TF publishers in the final report."
fi

echo
echo "===== 5. CHECK BODY-TO-LIDAR TF ====="
timeout 4 ros2 run tf2_ros tf2_echo base_footprint lidar_link >"$TF_LOG" 2>&1 || true
if grep -q "At time" "$TF_LOG"; then
  echo "PASS: base_footprint -> lidar_link transform observed."
else
  echo "WARNING: body-to-LiDAR TF not confirmed yet; Bridge diagnostics will report the missing transform."
  tail -20 "$TF_LOG" || true
fi

echo
echo "===== 6. START LIVE SCAN + STATIC MAP + SCAN PLANNER ====="
echo "PCD: $PCD"
echo "Adapter already running: start_localization_adapter=$START_ADAPTER"
nohup ros2 launch n01_live_navigation live_scan.launch.py \
  pcd_path:="$PCD" \
  pcd_map_frame_asserted:=true \
  start_localization_adapter:="$START_ADAPTER" \
  rviz:=true \
  >"$LAUNCH_LOG" 2>&1 </dev/null &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$PID_FILE"
echo "launch_pid=$LAUNCH_PID"
echo "launch_log=$LAUNCH_LOG"

echo
echo "===== 7. WAIT FOR STACK DISCOVERY ====="
for _ in $(seq 1 30); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ERROR: launch process exited early."
    tail -100 "$LAUNCH_LOG" || true
    exit 4
  fi
  if ros2 node list 2>/dev/null | grep -Fxq "/scan_planner_node"; then
    break
  fi
  sleep 1
done

echo
echo "===== 8. READ-ONLY LIVE DATA ACCEPTANCE (20 s) ====="
if /usr/bin/python3 "$ROOT/scripts/n01_live_stack_preflight.py" --duration 20 >"$REPORT" 2>&1; then
  AUDIT_RC=0
else
  AUDIT_RC=$?
fi
echo "audit_exit_code=$AUDIT_RC"
echo "report=$REPORT"
cat "$REPORT"

echo
echo "===== 9. LAUNCH ERRORS / WARNINGS ====="
grep -Ei "error|fatal|exception|failed|warning|PCD 加载成功|体素化完成" "$LAUNCH_LOG" | tail -100 || true

echo
echo "===== 10. FINAL NODES ====="
ros2 node list | sort

echo
echo "===== COMPLETE ====="
echo "Build/test log: $BUILD_LOG"
echo "Launch log:     $LAUNCH_LOG"
echo "TF log:         $TF_LOG"
echo "Audit report:   $REPORT"
echo "Live stack remains running; stop it with Ctrl-C in its launch process or: kill -INT $LAUNCH_PID"
echo "ACTUATOR_OUTPUT=DISABLED"
echo "EXECUTION_AUTHORIZED=false"
echo "SAFETY_AUTHORIZATION=NEVER"
exit "$AUDIT_RC"
