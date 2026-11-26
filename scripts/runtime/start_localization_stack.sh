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
# Localization is the latency-critical producer. On an 8-core host reserve
# 0-2 for it, 3-5 for the navigation stack and 6 for the controller; CPU7 is
# intentionally left untouched. Override with N01_LOC_CPU_SET=... .
if [[ -z "${N01_LOC_CPU_SET:-}" ]] && [[ "$(nproc)" -ge 8 ]]; then
  N01_LOC_CPU_SET="0-2"
fi
if [[ -n "${N01_LOC_CPU_SET:-}" ]] && command -v taskset >/dev/null 2>&1; then
  if [[ -n "${N01_LOC_RT_PRIO:-}" ]] && command -v chrt >/dev/null 2>&1; then
    exec chrt -f "$N01_LOC_RT_PRIO" taskset -c "$N01_LOC_CPU_SET" ros2 launch n01_live_navigation n01_localization_stack.launch.py "$@"
  fi
  exec taskset -c "$N01_LOC_CPU_SET" ros2 launch n01_live_navigation n01_localization_stack.launch.py "$@"
fi
exec ros2 launch n01_live_navigation n01_localization_stack.launch.py "$@"
