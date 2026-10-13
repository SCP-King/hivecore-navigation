#!/usr/bin/env bash
# =============================================================================
# start_hivecore_deps.sh —— 甲方依赖启动/检查助手（默认只检查，不擅动甲方状态）
# =============================================================================
# 原则（乙方铁律）：
#   * 默认【只检测 + 提示】：列出每个甲方依赖当前状态、缺什么、怎么启动。
#     除非显式传 --start，否则不运行任何甲方 launch/脚本，不改甲方运行状态。
#   * 不修改甲方任何启动脚本/URDF/SLAM 参数/底盘驱动。
#   * 检测已运行实例：若 GUI-SLAM / 运动栈已在跑，直接报告"已运行"，绝不重复启动。
#   * 一键启动（--start）必须显式授权；启动失败有恢复提示。
#
# 用法：
#   bash scripts/start_hivecore_deps.sh                    # 只检查（推荐，默认）
#   bash scripts/start_hivecore_deps.sh --status           # 同上，别名
#   bash scripts/start_hivecore_deps.sh --start            # 显式授权后：启动缺失的甲方依赖
#   bash scripts/start_hivecore_deps.sh --dryrun-commands  # 只打印"若需启动会执行什么"，不运行
#
# 甲方启动方式（已核实，与桌面图标一致）：
#   运动栈 : start_hive2_motion.sh（PID 已在现场运行，通常无需重起）
#   GUI-SLAM: 双击「GUI-SLAM启动」或
#             source /home/demo/release_slam_3d/setup_runtime.sh
#             ros2 launch hivecore_console_slam3d gui_slam_console.launch.py use_sim_time:=false
# =============================================================================
set -uo pipefail

ACTION="${1:-status}"     # status | start | dryrun-commands
case "${1:-}" in
  --status) ACTION=status ;;
  --start)  ACTION=start ;;
  --dryrun-commands) ACTION=dryrun ;;
esac

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

# 显式加载 ROS 环境（非交互 shell 的 .bashrc 可能不生效，须保证
# LD_LIBRARY_PATH 含 /opt/ros/humble/lib，否则 ros2 动态库会 ImportError）
if [ -f /home/demo/release_slam_3d/setup_runtime.sh ]; then
  # shellcheck disable=SC1091
  source /home/demo/release_slam_3d/setup_runtime.sh
elif [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if ! command -v ros2 >/dev/null 2>&1; then
  echo "[FAIL] 找不到 ROS 2 环境（ros2 不在 PATH）"
  exit 2
fi

run() { echo "  $*" >&2; "$@" >/dev/null 2>&1; }
have() { command -v "$1" >/dev/null 2>&1; }

RED='\033[31m'; GRN='\033[32m'; YEL='\033[33m'; CYN='\033[36m'; NC='\033[0m'
ok()   { printf "${GRN}[OK]  ${NC}%s\n" "$*"; }
miss() { printf "${YEL}[缺]  ${NC}%s\n" "$*"; }
err()  { printf "${RED}[ERR] ${NC}%s\n" "$*"; }
note() { printf "${CYN}[--]  ${NC}%s\n" "$*"; }

echo "${CYN}===== HiveCore 甲方依赖状态检查（ROS_DOMAIN_ID=${ROS_DOMAIN_ID}） =====${NC}"

# ---------------- 1. 运动栈 ----------------
echo ""
note "1/ 运动栈（底盘 + Livox + /odom）"
MOTION_RUNNING=0
for pat in "hive2_driver.launch.py" "hive2_controller.launch.py" "livox_ros_driver2"; do
  if pgrep -f "$pat" >/dev/null 2>&1; then
    MOTION_RUNNING=1
    ok "已在运行：$pat"
  else
    miss "未运行：$pat"
  fi
done
if [ "$MOTION_RUNNING" = "0" ]; then
  note "启动命令：bash /home/demo/release_slam_3d/scripts/start_hive2_motion.sh（或甲方运动栈 SOP）"
fi

# ---------------- 2. GUI-SLAM ----------------
echo ""
note "2/ GUI-SLAM（geo_filtered + livox_frame TF）"
GUI_UP=0
for pat in "gui_slam_console.launch.py" "lidar_cloud_preprocess_node" "static_tf_publisher_node"; do
  if pgrep -f "$pat" >/dev/null 2>&1; then
    GUI_UP=1
    ok "已在运行：$pat"
  else
    miss "未运行：$pat"
  fi
done
if [ "$GUI_UP" = "0" ]; then
  note "启动命令（与桌面图标一致，需已起运动栈）："
  note "  source /home/demo/release_slam_3d/setup_runtime.sh"
  note "  ros2 launch hivecore_console_slam3d gui_slam_console.launch.py use_sim_time:=false"
fi

# ---------------- 3. 必需话题 ----------------
echo ""
note "3/ 必需话题（实际订阅取样，非只看名字）"
check_topic() { # check_topic <name> <hz_hint>
  local t="$1" hz="$2" n
  if ros2 topic list 2>/dev/null | grep -x "$t" >/dev/null; then
    n=$(timeout 3.5 ros2 topic echo "$t" --field header.stamp 2>/dev/null | grep -c "sec: [0-9]" || true)
    if [ "$n" -ge 1 ]; then
      ok "$t 实际有数据（~${n} 帧/3.5s）"
    else
      miss "$t 存在但 3.5s 无数据（需真订阅者 / GUI 操作 / enable_forward）"
    fi
  else
    miss "$t 不存在"
  fi
}
check_topic /odom 50
check_topic /livox/lidar 10

# ---------------- 4. geo_filtered 与 GUI 前置提示 ----------------
echo ""
note "4/ GUI 前置数据（geo_filtered / robot_pose / localization_status）"
if ros2 topic list 2>/dev/null | grep -x /livox/lidar/geo_filtered >/dev/null; then
  n=$(timeout 3.5 ros2 topic echo /livox/lidar/geo_filtered --field header.stamp 2>/dev/null | grep -c "sec: [0-9]" || true)
  [ "$n" -ge 1 ] && ok "/livox/lidar/geo_filtered 有数据（${n} 帧/3.5s）" \
    || miss "/livox/lidar/geo_filtered 存在但无数据（publish_only_when_subscribed，需真实订阅者）"
else
  miss "/livox/lidar/geo_filtered 不存在（GUI-SLAM 未运行）"
fi
for t in /robot_pose /localization_status; do
  if ros2 topic list 2>/dev/null | grep -x "$t" >/dev/null; then
    n=$(timeout 3.5 ros2 topic echo "$t" --field header.stamp 2>/dev/null | grep -c "sec: [0-9]" || true)
    [ "$n" -ge 1 ] && ok "$t 有数据" || miss "$t 存在但无数据（需 GUI Localization: Load map → Session Start）"
  else
    miss "$t 不存在"
  fi
done

# ---------------- 5. 双 RSP 冲突提示 ----------------
echo ""
note "5/ 双 RSP 冲突检测"
DUP_PUBS=$(ros2 topic info /tf_static -v 2>&1 | grep -c "Endpoint type: PUBLISHER" || true)
ALT_Z=""
for i in 1 2 3 4 5 6; do
  z=$(timeout 3 ros2 run tf2_ros tf2_echo base_footprint base_link 2>/dev/null | grep -m1 "Translation: " | grep -oE "\[[-0-9., ]+\]" | tr -d '[]' | cut -d',' -f3)
  [ -n "$z" ] && ALT_Z="$ALT_Z $z"
  sleep 0.2
done
UNIQ=$(echo "$ALT_Z" | tr ' ' '\n' | sort -u | grep -v '^$')
CNT=$(printf '%s\n' "$UNIQ" | grep -c . || true)

if [ "$DUP_PUBS" -ge 2 ] || { [ -n "$UNIQ" ] && [ "$CNT" -ge 2 ]; }; then
  err "检测到 base_footprint→base_link 多定义（/tf_static 发布者=${DUP_PUBS}，z 集合：$(echo $UNIQ | tr '\n' '/')）"
  err "  → 来源①运动栈 URDF z=0.9：hive2.config.xacro（/opt/hivecore/hive2-motion/1.0.0/install/share/hive2_driver/description/）"
  err "  → 来源②SLAM 栈 URDF z=0.3：hivecore_robot.urdf.xacro footprint_xyz 默认（/home/demo/release_slam_3d/install/hivecore_robot_description/share/hivecore_robot_description/urdf/）"
  err "  → 需甲方确认正确几何（现场实测 z=0.9）并停用其一；乙方不代做、不掩盖（禁止新增第三条 TF / 写死高度）"
elif [ -z "$UNIQ" ]; then
  miss "base_footprint→base_link 无法采样 z 值（跳过双 RSP 判定）"
else
  ok "base_footprint→base_link 唯一（发布者=${DUP_PUBS}，z=$(echo $UNIQ | tr '\n' '/')）"
fi

# ---------------- 汇总 & 动作 ----------------
echo ""
if [ "$ACTION" = "start" ]; then
  echo "${CYN}--start 已授权：尝试启动缺失的甲方依赖（不重复已运行的）${NC}"
  if [ "$MOTION_RUNNING" = "0" ]; then
    note "启动运动栈：bash /home/demo/release_slam_3d/scripts/start_hive2_motion.sh"
    bash /home/demo/release_slam_3d/scripts/start_hive2_motion.sh >/tmp/hive2_motion_start.log 2>&1 \
      || err "运动栈启动失败，详见 /tmp/hive2_motion_start.log；请检查电源/底盘连接后重试。"
  fi
  if [ "$GUI_UP" = "0" ]; then
    note "启动 GUI-SLAM（与桌面一致）……"
    cd /home/demo/release_slam_3d || exit 1
    # shellcheck disable=SC1091
    source setup_runtime.sh
    nohup ros2 launch hivecore_console_slam3d gui_slam_console.launch.py use_sim_time:=false \
      >/tmp/gui_slam_start.log 2>&1 &
    echo "  [GUI-SLAM] 已后台启动，日志 /tmp/gui_slam_start.log（等待 ~15s 注册）"
    sleep 15
    pgrep -f "gui_slam_console.launch.py" >/dev/null 2>&1 \
      && ok "GUI-SLAM 启动成功" \
      || err "GUI-SLAM 启动疑似失败，查看 /tmp/gui_slam_start.log"
  fi
elif [ "$ACTION" = "dryrun" ]; then
  echo "${CYN}--dryrun-commands 打印将执行的命令（未执行任何东西）${NC}"
  echo "  运动栈未运行时： bash /home/demo/release_slam_3d/scripts/start_hive2_motion.sh"
  echo "  GUI-SLAM 未运行时："
  echo "    source /home/demo/release_slam_3d/setup_runtime.sh"
  echo "    ros2 launch hivecore_console_slam3d gui_slam_console.launch.py use_sim_time:=false"
fi

echo ""
echo "${CYN}检查完成。运行乙方导航前建议执行： bash scripts/preflight_hivecore.sh ${NC}"