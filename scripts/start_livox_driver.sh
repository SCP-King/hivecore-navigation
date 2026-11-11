#!/usr/bin/env bash
# Start the vendor Livox MID360 driver with the hive2-motion overlay ahead of
# slam3d_release so the launch/node resolve to the copy that has the driver.
set -eo pipefail
# ROS setup files reference optional variables; do not source under nounset.
set +u
source /opt/ros/humble/setup.bash
source /opt/slam3d_release/setup_runtime.sh
source /opt/hivecore/hive2-motion/1.0.0/install/setup.bash
# Both /opt trees ship a livox_ros_driver2 package. Only hive2-motion has the
# launch + node; put it first explicitly because slam3d_release's setup_runtime
# can leave its own prefix earlier in AMENT_PREFIX_PATH.
export AMENT_PREFIX_PATH=/opt/hivecore/hive2-motion/1.0.0/install:${AMENT_PREFIX_PATH:-}
# /home/demo/.ros/log is read-only in this deployment.
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/n01_ros_livox_log}"
export ROS_HOME="${ROS_HOME:-/tmp/n01_ros_livox_home}"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
if python3 "$SCRIPT_DIR/livox_start_preflight.py"; then
  exit 0
else
  preflight_rc=$?
  if [[ "$preflight_rc" -ne 10 ]]; then exit "$preflight_rc"; fi
fi
exec ros2 launch livox_ros_driver2 msg_MID360_launch.py
