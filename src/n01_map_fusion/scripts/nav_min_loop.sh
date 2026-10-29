#!/usr/bin/env bash
# Phase 5 最小闭环编排(隔离域,wall-clock,无 bag,无真机):
#   静态PCD 供给 ─> 融合(ROI) ─> A* 规划 ─> 独立模拟位姿+运动模型闭环
#   阶段A: 无动态障碍 -> 静态短路径 + 模拟闭环到达;
#   阶段B: 注入模拟动态障碍(路径中点) -> 规划器重规划绕障 / 正确拒绝。
#
# 安全边界(严格):
#   - ROS_DOMAIN_ID=88 + ROS_LOCALHOST_ONLY=1,不碰真机 55 域。
#   - 只读 PCD;不启动甲方 SLAM/Console/GUI/底盘;不发布 /cmd_vel*。
#   - 控制仅出现在隔离测试话题 /nav/test/cmd_vel(sim_motion 内部计算)。
#   - 不用 use_sim_time(无 /clock,否则 timer 不触发);wall-clock 计时。
#   - 结束时只杀本脚本启动的 PID。
set -o pipefail          # 不用 set -u:ROS setup 脚本会引用未定义变量

DOMAIN=88
PCD=/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd
OUT=/home/demo/lyy/hivecore-navigation/src/n01_map_fusion/output
PARAMS=/home/demo/lyy/hivecore-navigation/src/n01_map_fusion/config/fusion_planner.yaml
COLLECTOR=/home/demo/lyy/hivecore-navigation/src/n01_map_fusion/n01_map_fusion/loop_collector.py

PHASE_A_S=16     # 阶段A时长
PHASE_B_S=16     # 阶段B时长
COLLECT_S=13     # collector 采集时长(< 阶段时长,先写盘再被 kill,避免竞态)
REPORT_A=$OUT/loop_report_a.json
REPORT_B=$OUT/loop_report_b.json

export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1
export ROS_HOME="$OUT/.ros_home" ROS_LOG_DIR="$OUT/.ros_home/log"
# set -u 下 source ROS setup 需要这些先定义(避免 unbound variable)
export COLCON_CURRENT_PREFIX="${COLCON_CURRENT_PREFIX:-}" AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}" \
       AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-$(which python3)}" \
       ROS_DISTRO="${ROS_DISTRO:-humble}" ROS_VERSION="${ROS_VERSION:-2}"
source /opt/ros/humble/setup.bash
source /home/demo/lyy/hivecore-navigation/install/setup.bash
mkdir -p "$OUT" "$ROS_HOME"

run_bg() { local name=$1; shift
    setsid "$@" >"$OUT/$name.log" 2>&1 & echo $!; }
# 杀整个进程组(-PGID,ros2 run 外壳+node 子进程),避免孤儿
kill_pids() { local p; for p in "$@"; do
        kill -- "-$p" 2>/dev/null || kill "$p" 2>/dev/null || true
    done
    sleep 1; for p in "$@"; do
        kill -9 -- "-$p" 2>/dev/null || kill -9 "$p" 2>/dev/null || true
    done; }

echo "[nav-min-loop] domain=$DOMAIN localhost-only (wall-clock,无bag,无真机)"
echo "[nav-min-loop] ROI= x[9.5,12.6] y[5.4,9.0] start=(11.05,7.95) goal=(11.05,6.45)"

# ---- 常驻数据链(阶段A就起,阶段B保持) ----
P1=$(run_bg static_map ros2 run n01_static_map static_map_node \
    --ros-args -p pcd_path:="$PCD")
sleep 1
P2=$(run_bg fusion ros2 run n01_map_fusion fusion_node \
    --ros-args --params-file "$PARAMS")
sleep 1
P3=$(run_bg planner ros2 run n01_map_fusion planner_node \
    --ros-args --params-file "$PARAMS")
sleep 1

# ============ 阶段 A:静态短路径 + 模拟定位/运动闭环 ============
echo "[nav-min-loop] 阶段A: 静态 PCD 短路径 + 模拟运动闭环 (${PHASE_A_S}s) —— 无动态障碍"
P4=$(run_bg sim_motion ros2 run n01_map_fusion sim_motion \
    --ros-args -p start_x:=11.05 -p start_y:=7.95 -p start_yaw:=1.5708)
P5=$(run_bg collector_a python3 -u "$COLLECTOR" --output "$REPORT_A" --quit-after "$COLLECT_S")
sleep "$PHASE_A_S"
echo "[nav-min-loop] 阶段A 结束"

kill_pids "$P4" "$P5"
# ============ 阶段 B:注入动态障碍验证重规划/拒绝 ============
echo "[nav-min-loop] 阶段B: 注入模拟动态障碍(路径中点)验证重规划 (${PHASE_B_S}s)"
# 先起 mock(注入持续发布),等 3s 让 fusion 消化动态层并触发 planner 重规划;
# 再起 sim_motion 从起点试图跟踪——若规划器已重规划绕障,sim 走绕行路径;若双方
# 拒绝(无绕行),sim 保持 S0 并提示不可达。
P6=$(run_bg mock_obstacle ros2 run n01_map_fusion mock_obstacle \
    --ros-args -p center_x:=11.05 -p center_y:=7.20 -p radius:=0.15 \
    -p publish_after_s:=1.0 -p rate:=2.0)
sleep 3
P4=$(run_bg sim_motion2 ros2 run n01_map_fusion sim_motion \
    --ros-args -p start_x:=11.05 -p start_y:=7.95 -p start_yaw:=1.5708)
P5=$(run_bg collector_b python3 -u "$COLLECTOR" --output "$REPORT_B" --quit-after "$COLLECT_S")
sleep "$PHASE_B_S"
echo "[nav-min-loop] 阶段B 结束"

kill_pids "$P1" "$P2" "$P3" "$P4" "$P5" "$P6"

echo "[nav-min-loop] === 阶段A 报告 ==="
python3 - "$REPORT_A" <<'EOF'
import json, sys
r = json.load(open(sys.argv[1]))
print('fused_grid_msgs=%d  events=%d' % (r.get('fused_grid_messages',0), len(r.get('events',[]))))
for e in r.get('events',[]):
    print('  t=%6.2f %-10s %s' % (e['t'], e['kind'], str(e['data'])[:110]))
EOF
echo "[nav-min-loop] === 阶段B 报告 ==="
python3 - "$REPORT_B" <<'EOF'
import json, sys
r = json.load(open(sys.argv[1]))
print('fused_grid_msgs=%d  events=%d' % (r.get('fused_grid_messages',0), len(r.get('events',[]))))
for e in r.get('events',[]):
    print('  t=%6.2f %-10s %s' % (e['t'], e['kind'], str(e['data'])[:110]))
EOF

echo "[nav-min-loop] === 日志尾部 ==="
for f in static_map fusion planner mock_obstacle sim_motion; do
    echo "--- $f ---"; tail -n 5 "$OUT/$f.log" 2>/dev/null || true
done
echo "[nav-min-loop] done"