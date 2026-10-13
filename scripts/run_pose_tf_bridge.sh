#!/bin/bash
# =============================================================================
# run_pose_tf_bridge.sh —— 单独拉起 pose_tf_bridge（不重启整栈）
# =============================================================================
# 作用：订阅 /robot_pose，补出 map→base_footprint 的 TF，修复以下断链
#       {map → sliding_map}  ×  {base_footprint → base_link → 腿/雷达/IMU}
#       （甲方 slam3d_node 的位姿 TF 实测不可靠，新起的 tf2 订阅者取不到）
#
# 用法（前台看日志）:
#   bash ~/lyy/hivecore-navigation/scripts/run_pose_tf_bridge.sh
# 后台跑:
#   nohup bash ~/lyy/hivecore-navigation/scripts/run_pose_tf_bridge.sh \
#       > /tmp/pose_tf_bridge.log 2>&1 &
#
# 关掉（甲方侧 TF 修好后）:
#   pkill -f 'dynacbf_gate/pose_tf_bridge'
#   或把 enable 置 false 后重启本脚本
# =============================================================================
set -e

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=55
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /home/demo/lyy/hivecore-navigation/install/setup.bash

exec ros2 run dynacbf_gate pose_tf_bridge --ros-args \
  -p enable:=true \
  -p pose_topic:=/robot_pose \
  -p parent_frame:=map \
  -p child_frame:=base_footprint \
  -p detect_sec:=3.0 \
  -p pose_timeout_sec:=2.0
