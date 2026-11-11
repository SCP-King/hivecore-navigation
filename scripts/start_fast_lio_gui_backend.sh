#!/usr/bin/env bash
# GUI lifecycle helper for the repository-owned FAST-LIO2 backend.
set -eo pipefail
set +u

MODE="${1:-}"
shift || true

source /opt/ros/humble/setup.bash
source /opt/slam3d_release/setup_runtime.sh
source /home/demo/lyy/hivecore-navigation/install/setup.bash

if [[ ! -w "${HOME}/.ros/log" ]]; then
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/n01_ros_fast_lio_log}"
  export ROS_HOME="${ROS_HOME:-/tmp/n01_ros_fast_lio_home}"
  mkdir -p "${ROS_LOG_DIR}" "${ROS_HOME}"
fi

case "${MODE}" in
  localization)
    exec ros2 launch n01_fast_lio fast_lio_nav_bridge.launch.py "$@"
    ;;
  mapping)
    exec ros2 launch n01_fast_lio fast_lio_mapping.launch.py "$@"
    ;;
  realtime)
    exec ros2 launch n01_fast_lio fast_lio_realtime_nav_bridge.launch.py "$@"
    ;;
  *)
    echo "usage: $0 {localization|mapping|realtime} [launch arguments...]" >&2
    exit 2
    ;;
esac
