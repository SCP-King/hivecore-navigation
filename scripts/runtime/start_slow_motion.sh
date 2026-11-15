#!/usr/bin/env bash
# Internal GUI lifecycle: source the N-01 workspace before ros2 run, otherwise the GUI
# child environment (vendor SLAM only) cannot find n01_live_navigation.
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
N01_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$N01_PROJECT_ROOT/scripts/lib/slam_runtime.sh"
n01_source_slam_runtime
source "$N01_PROJECT_ROOT/install/local_setup.bash"
if [[ ! -w "$HOME/.ros/log" ]]; then
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/n01_ros_slow_log}"
  export ROS_HOME="${ROS_HOME:-/tmp/n01_ros_slow_home}"
  mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
fi
export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-ASYNCHRONOUS}"
# Hardware RPC and publisher-GID ownership stay on the verified control transport.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec ros2 run n01_live_navigation n01_slow_motion_executor "$@"
