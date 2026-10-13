#!/bin/bash
# 干跑验证：整链照常出解（MPC/规划/门控全跑），但 gate 的速度出口指向
# /cmd_vel_dryrun，不接到底盘 —— 用于零风险验证控制序列推进效果。
# M1 系配置（pose_topic=/odom nav_frame=odom pose_child_frame=base_footprint
#   gate_require_slam=false）——不依赖甲方 SLAM / 不锁门控。
export ROS_DOMAIN_ID=55
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
source /home/demo/lyy/hivecore-navigation/install/setup.bash
cd /home/demo/lyy/hivecore-navigation
exec ros2 launch dynacbf_bringup dynacbf_real.launch.py \
  navi_mode:=1 \
  pose_topic:=/odom \
  nav_frame:=odom \
  pose_child_frame:=base_footprint \
  gate_require_slam:=false \
  bridge_detect_sec:=0.0 \
  gate_cmd_out:=/cmd_vel_dryrun \
  speed_limit:=0.3 \
  omega_limit:=0.4 \
  lidar_topic:=/livox/lidar \
  rviz:=false
