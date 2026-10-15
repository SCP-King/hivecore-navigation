#!/bin/bash
# run_localmap_only.sh —— 仓库内单节点 local_map（不依赖 /tmp，重启后仍在仓库里）
# 用法: bash scripts/run_localmap_only.sh  或  直接执行
#
# 说明:
#   * 仅启动感知节点 local_map（dynacbf_perception），不启动规划/控制/门控。
#   * M1 参数: map_frame=odom（整链跑在底盘里程计系，无需甲方 SLAM），
#     odom_topic=/odom, lidar_topic=/livox/lidar/geo_filtered（甲方过滤点云）。
#   * base_link 参数在 local_map.cpp 中声明但 updateTF() 未使用（实际取 /odom 的
#     child_frame_id），此处传默认 base_link 仅为对齐 launch 声明，不影响行为。
#   * 用位置确定仓库根目录，不依赖执行时的当前目录。
set -eo pipefail

echo "===== run_localmap_only.sh (仓库内单节点 local_map) ====="

REPO_ROOT=$(dirname "$(dirname "$(readlink -f "$0")")")
echo "repo root: ${REPO_ROOT}"

# 环境：ROS 2 Humble + 乙方 install
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/install/setup.bash"
export ROS_DOMAIN_ID=55
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_LOCALHOST_ONLY=0

echo "===== 启动 local_map (M1 参数) ====="
exec ros2 run dynacbf_perception local_map --ros-args \
  -p lidar_topic:=/livox/lidar/geo_filtered \
  -p odom_topic:=/odom \
  -p map_frame:=odom \
  -p base_link:=base_link