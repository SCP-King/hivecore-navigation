#!/usr/bin/env bash
# GUI lifecycle helper: source ROS + vendor SLAM + N-01 workspace, then start
# the unified localization stack. Keeps desk_guard's environment honest.
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /opt/slam3d_release/setup_runtime.sh
source /home/demo/lyy/hivecore-navigation/install/setup.bash
if [[ ! -w "$HOME/.ros/log" ]]; then
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/n01_ros_loc_log}"
  export ROS_HOME="${ROS_HOME:-/tmp/n01_ros_loc_home}"
  mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
fi
exec ros2 launch n01_live_navigation n01_localization_stack.launch.py "$@"
