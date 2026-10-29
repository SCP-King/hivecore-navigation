#!/usr/bin/env bash
# Phase 5 隔离域回放编排:静态 PCD 静态供给 + 实时障碍(回放 bag)融合 + A* 规划。
#
# 安全边界(严格,与 n01_static_map/replay_nav_chain.sh 一致):
#   - ROS_DOMAIN_ID=88 + ROS_LOCALHOST_ONLY=1,不碰真机 55 域。
#   - 只回放 /livox/lidar/geo_filtered /tf /tf_static(挂 --clock)。
#   - 不启动甲方 SLAM/Console/GUI;不发布 /cmd_vel*;不回放 /odom /robot_pose /cloud_map。
#   - 乙方感知/scan/静态/融合/A* 均 use_sim_time=true。
#   - 全程只读 bag 与 PCD;结束自动清理,不留残留进程。
set -eo pipefail

DOMAIN=88
BAG=/home/demo/lyy/slam_data/bags/bag_20261027_225100
PCD=/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd
TOPICS="/livox/lidar/geo_filtered /tf /tf_static"
RATE=2.0
OUT=/home/demo/lyy/hivecore-navigation/src/n01_map_fusion/output
PARAMS=/home/demo/lyy/hivecore-navigation/src/n01_map_fusion/config/fusion_planner.yaml
VERIFY=/home/demo/lyy/hivecore-navigation/src/n01_map_fusion/scripts/verify_fusion_a_star.py
START="12.35 2.15"; GOAL="11.75 12.95"; SNAP=1.5
IDLE=15
WALL_TIMEOUT=360

export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1
export ROS_HOME="$OUT/.ros_home" ROS_LOG_DIR="$OUT/.ros_home/log"
source /opt/ros/humble/setup.bash
source /home/demo/lyy/hivecore-navigation/install/setup.bash
mkdir -p "$OUT" "$ROS_HOME"

run_bg() { local name=$1; shift
    "$@" >"$OUT/$name.log" 2>&1 & echo $!; }

echo "[phase5] domain=$DOMAIN localhost-only rate=$RATE sub-region S=($START) G=($GOAL)"
echo "[phase5] 只回放 $TOPICS + clock;不启动甲方 SLAM/底盘"

# ---- 数据链:静态供给 + 实时障碍(回放) + 融合 + A* ----
P1=$(run_bg static_map ros2 run n01_static_map static_map_node \
    --ros-args -p pcd_path:="$PCD" -p use_sim_time:=true)
sleep 1
P2=$(run_bg perception ros2 run n01_perception perception_node \
    --ros-args -p use_sim_time:=true \
    -p input_topic:=/livox/lidar/geo_filtered -p target_frame:=map)
sleep 1
P3=$(run_bg scan ros2 run n01_scan scan_grid_map \
    --ros-args -p use_sim_time:=true \
    -p mode:=map_localized -p map_frame:=map -p lidar_frame:=lidar_link \
    -p input_topic:=/local_pcd)
sleep 1
P4=$(run_bg fusion ros2 run n01_map_fusion fusion_node \
    --ros-args -p use_sim_time:=true --params-file "$PARAMS")
sleep 1
P5=$(run_bg planner ros2 run n01_map_fusion planner_node \
    --ros-args -p use_sim_time:=true --params-file "$PARAMS")
sleep 1

echo "[phase5] starting verifier (only-read subscriber)"
P6=$(run_bg verifier python3 -u "$VERIFY" \
    --start $START --goal $GOAL --snap-radius "$SNAP" --idle-seconds "$IDLE" \
    --output "$OUT/verifier_report.json")

echo "[phase5] starting bag play (clock + nav topics)"
P7=$(run_bg bag_play ros2 bag play "$BAG" --clock --rate "$RATE" --topics $TOPICS)
BAG_START_SEC=$SECONDS

# ---- 等待:bag 结束或整体超时----
end=$((SECONDS + WALL_TIMEOUT))
while kill -0 "$P7" 2>/dev/null; do
    if [ $SECONDS -ge $end ]; then
        echo "[phase5] WARNING: bag play 超时 $WALL_TIMEOUT s,强收"
        break
    fi
    sleep 2
done
echo "[phase5] bag play 结束(运行 ${BAG_START_SEC}+s;由 verifier 判定覆盖情况)"

# 等 verifier 在 idle 判定后自行写出报告(最多 WAIT_VERIFIER s)
WAIT_VERIFIER=60
v_end=$((SECONDS + WAIT_VERIFIER))
while [ $SECONDS -lt $v_end ] && ! [ -s "$OUT/verifier_report.json" ]; do sleep 2; done
if ! [ -s "$OUT/verifier_report.json" ]; then
    echo "[phase5] verifier 未在 ${WAIT_VERIFIER}s 内写出报告,发送 SIGTERM"
    kill -TERM "$P6" 2>/dev/null || true
    sleep 5
fi

echo "[phase5] === verifier 输出 ==="
cat "$OUT/verifier_report.json" 2>/dev/null || echo "(无报告)"

echo "[phase5] === scan_grid_map 日志尾部(TF 丢失/复位证据) ==="
grep -nE "LOST|reset|tf|TF|占位|occupancy|S4" "$OUT/scan.log" 2>/dev/null | tail -15 || true
echo "[phase5] === planner 日志尾部 ==="
tail -n 8 "$OUT/planner.log" 2>/dev/null || true
echo "[phase5] === fusion 日志尾部 ==="
tail -n 6 "$OUT/fusion.log" 2>/dev/null || true

# ---- 清理 ----
echo "[phase5] cleaning up (只杀本脚本启动的 PID)..."
kill $P1 $P2 $P3 $P4 $P5 $P6 $P7 2>/dev/null || true
sleep 1
kill -9 $P1 $P2 $P3 $P4 $P5 $P6 $P7 2>/dev/null || true

echo "[phase5] done"