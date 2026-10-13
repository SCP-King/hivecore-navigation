#!/bin/bash
# m1_preflight_and_run.sh —— 编译 + 前置检查 + 启动 M1 (乙方, by 乙方)
# 用法: bash scripts/m1_preflight_and_run.sh
# ⚠️ pose_child_frame 必须挂 base_footprint(不挂 base_link):
#    甲方 RSP 静态边 base_footprint→base_link 已在 /tf_static,若桥再发 odom→base_link,
#    base_link 双 parent → TF 树分裂 → wait for lidar 死循环(审计 2026-10-13 确认)。
set -euo pipefail

echo "===== ① 清场: 杀乙方进程(避免残留 TF/话题双发) ====="
pkill -9 -f 'dynacbf_real.launch.py' 2>/dev/null || true
pkill -9 -f 'dynacbf' 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
cwd="/home/demo/lyy/hivecore-navigation"

echo "===== ② 确认甲方数据源已起 ====="
ros2 topic info /odom        # 应显示 'Publisher count: 1'
ros2 topic info /livox/lidar # 同上
echo "  [ok] 若上面两项 Publisher count 都是 0,请先起甲方运动栈再回来。"
sleep 2

echo "===== ③ 编译需要重编的 3 个 C++ 包 ====="
cd "$cwd"
source install/setup.bash 2>/dev/null || true
colcon build --symlink-install \
  --packages-select dynacbf_perception dynacbf_planner dynacbf_tracking \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -20
echo "  [ok] 编译完成(3 包 Finished,CGAL warning 可忽略)。"

echo "===== ④ 启动 M1 ====="
export ROS_DOMAIN_ID=55
export ROS_LOCALHOST_ONLY=0
source "$cwd/install/setup.bash"
ros2 launch dynacbf_bringup dynacbf_real.launch.py \
  rviz:=true \
  navi_mode:=1 \
  pose_topic:=/odom \
  nav_frame:=odom \
  pose_child_frame:=base_footprint \
  gate_require_slam:=false \
  bridge_detect_sec:=0.0 \
  lidar_topic:=/livox/lidar \
  speed_limit:=0.3 \
  omega_limit:=0.4