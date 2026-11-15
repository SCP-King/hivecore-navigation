#!/usr/bin/env bash
# Internal GUI lifecycle helper: source ROS + vendor SLAM + N-01 workspace, then start
# the unified localization stack. Keeps desk_guard's environment honest.
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
N01_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$N01_PROJECT_ROOT/scripts/lib/slam_runtime.sh"
n01_source_slam_runtime
source "$N01_PROJECT_ROOT/install/local_setup.bash"
if [[ ! -w "$HOME/.ros/log" ]]; then
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/n01_ros_loc_log}"
  export ROS_HOME="${ROS_HOME:-/tmp/n01_ros_loc_home}"
  mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
fi
# Keep large map/TF publication from blocking ROS callback threads.
export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-ASYNCHRONOUS}"
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OMP_NUM_THREADS=2
export OMP_WAIT_POLICY=PASSIVE
source /home/demo/lyy/hivecore-navigation/scripts/lib/ros_transport.sh
n01_configure_project_transport
exec ros2 launch n01_live_navigation n01_localization_stack.launch.py "$@"
