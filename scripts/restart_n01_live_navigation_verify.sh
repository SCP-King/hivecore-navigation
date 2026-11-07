#!/usr/bin/env bash
# Restart only the N-01 live navigation stack and verify runtime data flow.
# Does not restart Lite SLAM, Livox, chassis controller, or publish actuator commands.
set -Eeo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
RUNTIME_ROOT="${N01_RUNTIME_ROOT:-$HOME/lyy/n01_runtime}"
LOG_DIR="$RUNTIME_ROOT/logs"
RUN_DIR="$RUNTIME_ROOT/runs/live_navigation"
STAMP="$(date +%Y%m%d_%H%M%S)"
PCD="/opt/slam3d_release/resource/mapping_lite.pcd"
PID_FILE="$RUN_DIR/live_scan_launch.pid"
BUILD_LOG="$LOG_DIR/n01_tf_tolerance_build_$STAMP.log"
LAUNCH_LOG="$LOG_DIR/n01_tf_tolerance_launch_$STAMP.log"
FLOW_LOG="$LOG_DIR/n01_tf_tolerance_flow_$STAMP.log"

mkdir -p "$LOG_DIR" "$RUN_DIR"
cd "$ROOT"

# ROS and colcon setup scripts may read optional unset variables.
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

echo "===== BUILD UPDATED LIVE BRIDGE ====="
colcon build --symlink-install --packages-select n01_live_navigation \
  --event-handlers console_direct+ 2>&1 | tee "$BUILD_LOG"

echo
echo "===== UNIT TESTS ====="
colcon test --packages-select n01_live_navigation 2>&1 | tee -a "$BUILD_LOG"
colcon test-result --verbose 2>&1 | tee -a "$BUILD_LOG"

# Re-source after build without nounset; preserve the environment as ROS expects.
source "$ROOT/install/setup.bash"

echo
echo "===== STOP PREVIOUS N-01 LIVE LAUNCH ONLY ====="
if [[ -s "$PID_FILE" ]]; then
  OLD_PID="$(cat "$PID_FILE")"
  if [[ "$OLD_PID" =~ ^[0-9]+$ ]] && kill -0 "$OLD_PID" 2>/dev/null; then
    echo "Sending SIGINT to previous ros2 launch PID $OLD_PID"
    kill -INT "$OLD_PID" 2>/dev/null || true
    for _ in $(seq 1 10); do
      kill -0 "$OLD_PID" 2>/dev/null || break
      sleep 1
    done
    if kill -0 "$OLD_PID" 2>/dev/null; then
      echo "Previous launch did not exit after SIGINT; sending SIGTERM."
      kill -TERM "$OLD_PID" 2>/dev/null || true
      sleep 2
    fi
  else
    echo "Previous launch PID is not running."
  fi
else
  echo "No previous launch PID file."
fi

echo
echo "===== REUSE EXISTING LOCALIZATION ADAPTER IF PRESENT ====="
NODES="$(ros2 node list 2>&1 || true)"
if grep -Fxq "/n01_localization_adapter" <<<"$NODES"; then
  START_ADAPTER=false
  echo "Existing adapter retained; no duplicate adapter will be started."
else
  START_ADAPTER=true
  echo "Adapter absent; launch will start the configured adapter."
fi

echo
echo "===== START LIVE SCAN / STATIC MAP / SCAN PLANNER ====="
nohup ros2 launch n01_live_navigation live_scan.launch.py \
  pcd_path:="$PCD" \
  pcd_map_frame_asserted:=true \
  start_localization_adapter:="$START_ADAPTER" \
  rviz:=true >"$LAUNCH_LOG" 2>&1 </dev/null &
NEW_PID=$!
echo "$NEW_PID" > "$PID_FILE"
echo "launch_pid=$NEW_PID"
echo "launch_log=$LAUNCH_LOG"

sleep 8
if ! kill -0 "$NEW_PID" 2>/dev/null; then
  echo "ERROR: live launch exited. Last launch output:"
  tail -100 "$LAUNCH_LOG"
  exit 4
fi

echo
echo "===== CONTINUOUS DATA-FLOW VERIFICATION ====="
{
  echo "timestamp=$(date --iso-8601=seconds)"
  echo "launch_pid=$NEW_PID"
  echo
  for topic in /n01/localization/pose /odom /livox/lidar/geo_filtered \
      /cloud /sensor_pose /body_pose /n01/live_navigation/scan_body_pose; do
    echo "----- RATE: $topic -----"
    timeout 12 ros2 topic hz "$topic" || true
    echo
  done

  echo "----- BRIDGE READINESS SNAPSHOT -----"
  timeout 5 ros2 topic echo /n01/live_navigation/readiness --once || true
  echo

  echo "----- BRIDGE NODE GRAPH -----"
  timeout 5 ros2 node info /n01_live_scan_bridge || true
  echo

  echo "----- SCAN PLANNER NODE GRAPH -----"
  timeout 5 ros2 node info /scan_planner_node || true
  echo

  echo "----- LIVE READINESS STREAM (30 s) -----"
  timeout 30 ros2 topic echo /n01/live_navigation/readiness || true
  echo

  echo "----- RELEVANT LAUNCH WARNINGS / ERRORS -----"
  grep -Ei "error|fatal|exception|failed|warning|tf|body_pose|static map|PCD" \
    "$LAUNCH_LOG" | tail -120 || true

  echo
  echo "===== END ====="
  date
} 2>&1 | tee "$FLOW_LOG"

echo
echo "Navigation launch remains running."
echo "Build log:  $BUILD_LOG"
echo "Launch log: $LAUNCH_LOG"
echo "Flow log:   $FLOW_LOG"
echo "Actuator output remains DISABLED; no cmd_vel or chassis command is published."
