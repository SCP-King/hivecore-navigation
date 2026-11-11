#!/usr/bin/env bash
# GUI lifecycle: Hive2 chassis driver + steering supervisor.
# person_follow_controller is intentionally NOT started: it publishes /cmd_vel
# and would race the N-01 slow executor.
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /opt/hivecore/hive2-motion/1.0.0/install/setup.bash
source /home/demo/lyy/hivecore-navigation/install/setup.bash
if [[ ! -w "$HOME/.ros/log" ]]; then
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/n01_ros_chassis_log}"
  export ROS_HOME="${ROS_HOME:-/tmp/n01_ros_chassis_home}"
  mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
fi

# Start from a single clean chassis graph; stale old Hive2 drivers are the
# usual source of duplicate steer/preprocessor/RSP nodes.
bash /home/demo/lyy/hivecore-navigation/scripts/stop_chassis.sh || true

PARAMS="$(ros2 pkg prefix wheel_controller_refactor)/share/wheel_controller_refactor/config/hive2_chassis_params.yaml"
echo "chassis params: $PARAMS"
ros2 launch hive2_driver hive2_driver.launch.py imu_enabled:=${HIVE2_IMU_ENABLED:-false} &
DRIVER=$!
ros2 run wheel_controller_refactor steer_drive_node --ros-args --params-file "$PARAMS" &
STEER=$!
ros2 run wheel_controller_refactor cmd_wheel_preprocessor_node --ros-args --params-file "$PARAMS" &
PREP=$!

cleanup() {
  trap - INT TERM EXIT
  kill -INT "$DRIVER" "$STEER" "$PREP" 2>/dev/null || true
  sleep 1
  kill -TERM "$DRIVER" "$STEER" "$PREP" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT
wait
