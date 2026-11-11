#!/usr/bin/env bash
# GUI lifecycle: source the N-01 workspace before ros2 run, otherwise the GUI
# child environment (vendor SLAM only) cannot find n01_live_navigation.
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /opt/slam3d_release/setup_runtime.sh
source /home/demo/lyy/hivecore-navigation/install/setup.bash
if [[ ! -w "$HOME/.ros/log" ]]; then
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/n01_ros_slow_log}"
  export ROS_HOME="${ROS_HOME:-/tmp/n01_ros_slow_home}"
  mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
fi
exec ros2 run n01_live_navigation n01_slow_motion_executor "$@"
