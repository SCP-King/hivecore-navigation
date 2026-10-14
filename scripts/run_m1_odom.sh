#!/bin/bash
# =============================================================================
# run_m1_odom.sh —— M1 链路：不依赖甲方 SLAM，整链在底盘里程计系 odom 跑
# =============================================================================
# 用途：甲方 SLAM 的 /robot_pose 与 map 系位姿 TF 不可用时，先用底盘 /odom 把
#       感知 → 跟踪 → 规划 → MPC-D-CBF → 定位门控 → 底盘 全链路跑通
#       （即 SOP 里的 M1：相对起点验证）。
#
# 与真机模式的区别（全部通过 launch 参数传入，不改任何代码）：
#   pose_topic:=/odom            位姿源换成底盘里程计（约 50 Hz）
#   nav_frame:=odom              全局 frame 一起切换：
#                                local_map.map_frame / grid_map.frame_id /
#                                controller.global_frame / bspline.frame_id
#   pose_child_frame:=base_footprint /odom 的 child_frame 是 base_link，但乙方桥必须
#                               挂甲方地面系 base_footprint（与真机/甲方 RSP
#                               base_footprint→base_link 一致）：挂 base_link 会双 parent
#                               分裂 TF 树；挂 base_footprint 则整链 odom→base_footprint→
#                               base_link→lidar_link 连通，bridge 只转发 /odom xy/yaw、z=0
#   gate_require_slam:=false     门控不再依赖甲方 LocalizationStatus（否则一直拦住）
#   speed_limit / omega_limit    线/角速度上限，默认 0.3 m/s / 0.4 rad/s（试跑限速）
#   （真机模式对照：pose_topic:=/robot_pose nav_frame:=map
#                  pose_child_frame:=base_footprint gate_require_slam:=true
#                  speed_limit:=1.0 omega_limit:=1.25）
#   RViz：launch 按 nav_frame=odom 自动加载 dynacbf_m1.rviz（Fixed Frame=odom）
#
# 用法:
#   bash scripts/run_m1_odom.sh
#   nohup env PYTHONUNBUFFERED=1 bash scripts/run_m1_odom.sh > /tmp/dynacbf_m1.log 2>&1 &
#   RVIZ:=true bash scripts/run_m1_odom.sh        # 需要 RViz 下发目标时
#
# 注意:
#   1) 本模式**不加载离线 .pcd**（planner 用 cloud_is_world 的实时点云），
#      目标是"验证链路"，不是"在离线图上导航"。
#   2) 目标点要在 RViz 的 odom 系下下发；**未下发目标前机器人不会动**。
#   3) ⚠️ gate_require_slam:=false 意味着门控是放行的：一旦下发目标，
#      /cmd_vel 会真的到底盘。测试前务必确认场地安全（或把车架起来）。
#   4) 恢复真机模式：停掉本进程后改用
#        ros2 launch dynacbf_bringup dynacbf_real.launch.py rviz:=false navi_mode:=1
#      （默认即 map 系 + /robot_pose + gate 依赖甲方定位）
# =============================================================================
set -e

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=55
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /home/demo/lyy/hivecore-navigation/install/setup.bash

exec ros2 launch dynacbf_bringup dynacbf_real.launch.py \
  rviz:=${RVIZ:-false} \
  navi_mode:=${NAVI_MODE:-1} \
  pose_topic:=/odom \
  nav_frame:=odom \
  pose_child_frame:=base_footprint \
  gate_require_slam:=false \
  bridge_detect_sec:=0.0 \
  speed_limit:=${SPEED_LIMIT:-0.3} \
  omega_limit:=${OMEGA_LIMIT:-0.4} \
  lidar_topic:=/livox/lidar/geo_filtered
