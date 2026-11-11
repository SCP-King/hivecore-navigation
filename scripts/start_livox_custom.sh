#!/usr/bin/env bash
# FAST-LIO2 needs livox CustomMsg (xfer_format=1). Copy the vendor launch so the
# vendor file stays untouched; only the per-process copy is switched.
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /opt/slam3d_release/setup_runtime.sh
source /opt/hivecore/hive2-motion/1.0.0/install/setup.bash
export AMENT_PREFIX_PATH=/opt/hivecore/hive2-motion/1.0.0/install:${AMENT_PREFIX_PATH:-}
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/n01_ros_livox_custom_log}"
export ROS_HOME="${ROS_HOME:-/tmp/n01_ros_livox_custom_home}"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
VENDOR=/opt/hivecore/hive2-motion/1.0.0/install/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
VENDOR_CONFIG=/opt/hivecore/hive2-motion/1.0.0/install/share/livox_ros_driver2/config/MID360_config.json
TMP_DIR="${ROS_LOG_DIR}/livox_custom_launch"
rm -rf "$TMP_DIR"
mkdir -p "$TMP_DIR/launch" "$TMP_DIR/config"
cp -f "$VENDOR" "$TMP_DIR/launch/msg_MID360_custom.launch.py"
cp -f "$VENDOR_CONFIG" "$TMP_DIR/config/MID360_config.json"
sed -i 's/^xfer_format   = 0/xfer_format   = 1/' "$TMP_DIR/launch/msg_MID360_custom.launch.py"
grep -n '^xfer_format' "$TMP_DIR/launch/msg_MID360_custom.launch.py"
exec ros2 launch "$TMP_DIR/launch/msg_MID360_custom.launch.py"
