#!/usr/bin/env bash
# 隔离域 bag 回放:N-01 map_localized 链路对甲方 bag 数据的离线验证。
#
# 安全边界(严格):
#   - ROS_DOMAIN_ID=88(隔离),ROS_LOCALHOST_ONLY=1 —— 不碰真机 55 域。
#   - 只回放导航所需话题:/livox/lidar/geo_filtered /tf /tf_static(挂 --clock)。
#   - 不启动任何甲方 SLAM/Console/GUI;不发布 /cmd_vel;不回放 /odom /robot_pose /cloud_map。
#   - 乙方感知+scan 以 use_sim_time=true 运行(消费 /clock)。
#   - 全程只读 bag 文件,不修改、不裁剪。
#   - 结束自动关闭后台进程;超时强杀,不留残留。
set -eo pipefail

DOMAIN=88
BAG=/home/demo/lyy/slam_data/bags/bag_20261027_225100
TOPICS="/livox/lidar/geo_filtered /tf /tf_static"
OUT=/home/demo/lyy/hivecore-navigation/n01_static_map/output

export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1
# 沙箱下 ~/.ros 只读:C++/Python 日志都改到可写目录
export ROS_HOME="$OUT/.ros_home" ROS_LOG_DIR="$OUT/.ros_home/log"
source /opt/ros/humble/setup.bash
source /home/demo/lyy/hivecore-navigation/install/setup.bash
mkdir -p "$OUT" "$ROS_HOME"

run_bg() { local name=$1; shift
    "$@" >"$OUT/$name.log" 2>&1 & echo $!; }

echo "[replay] domain=$DOMAIN localhost-only  topics=$TOPICS"
echo "[replay] starting perception_node (use_sim_time)"
P1=$(run_bg perception_node ros2 run n01_perception perception_node \
    --ros-args -p use_sim_time:=true \
    -p input_topic:=/livox/lidar/geo_filtered -p target_frame:=map)
sleep 1
echo "[replay] starting scan_grid_map (map_localized, use_sim_time)"
P2=$(run_bg scan_grid_map ros2 run n01_scan scan_grid_map \
    --ros-args -p use_sim_time:=true \
    -p mode:=map_localized -p map_frame:=map -p lidar_frame:=lidar_link \
    -p input_topic:=/local_pcd)
sleep 1
echo "[replay] starting bag play (clock + nav topics)"
P3=$(run_bg bag_play ros2 bag play "$BAG" --clock --topics $TOPICS)

# 在回放期间观测占用输出(后台 echo,回放结束即停)
# 用 wall-clock 观测:回放 283s 太长,这里只等 occupancy 首次产出即收束。
echo "[replay] starting occupancy echo (in time window)"
POCC=$(run_bg topic_echo_occ \
    sh -c "for i in \$(seq 1 300); do ros2 topic echo /grid_map/occupancy --once 2>/dev/null && break; sleep 1; done")
PINFL=$(run_bg topic_echo_infl \
    sh -c "for i in \$(seq 1 300); do ros2 topic echo /grid_map/occupancy_inflate --once 2>/dev/null && break; sleep 1; done")

echo "[replay] waiting up to 300s for occupancy output..."
end=$((SECONDS+300))
seen=0
while [ $SECONDS -lt $end ]; do
    sleep 5
    if grep -q 'S4 update\[acc\]' "$OUT/scan_grid_map.log" 2>/dev/null; then seen=1; break; fi
    if ! kill -0 $P3 2>/dev/null; then break; fi
done
sleep 2
if [ "$seen" = 1 ]; then
    echo "[replay] RESULT: scan_grid_map 成功处理帧并产出占用 (见日志)"
else
    echo "[replay] RESULT: 未见 'S4 update[acc]' 日志"
fi

echo "[replay] === scan_grid_map 日志尾部 ==="
tail -n 25 "$OUT/scan_grid_map.log" 2>/dev/null || true
echo "[replay] === perception_node 日志尾部 ==="
tail -n 12 "$OUT/perception_node.log" 2>/dev/null || true
echo "[replay] === /grid_map/occupancy echo (前几行) ==="
[ -s "$OUT/topic_echo_occ.log" ] && head -50 "$OUT/topic_echo_occ.log" || echo "  (未捕获到占用话题)"
echo "[replay] === /grid_map/occupancy_inflate echo (前几行) ==="
[ -s "$OUT/topic_echo_infl.log" ] && head -30 "$OUT/topic_echo_infl.log" || echo "  (未捕获到膨胀占用话题)"

# 清理
echo "[replay] cleaning up..."
kill $P1 $P2 $P3 $POCC $PINFL 2>/dev/null || true
sleep 1
kill -9 $P1 $P2 $P3 $POCC $PINFL 2>/dev/null || true

echo "[replay] done (exit $?)"
