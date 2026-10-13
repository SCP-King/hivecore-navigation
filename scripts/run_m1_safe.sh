#!/usr/bin/env bash
# =============================================================================
# run_m1_safe.sh —— M1 安全启动（先 preflight，后干跑；不驱动底盘）
# =============================================================================
# 职责：
#   1. 先跑 scripts/preflight_hivecore.sh 做全部甲方数据自检；
#   2. preflight 全部 PASS → 以 gate_cmd_out:=/cmd_vel_dryrun 干跑导航整链
#      （MPC/规划/门控全跑，但速度发到 /cmd_vel_dryrun，绝不接到底盘 /cmd_vel）；
#   3. preflight 有 FAIL → 拒绝启动导航，打印缺失项与甲方启动命令/处理建议。
#
# 边界（开发运行准则 + 现场约束）：
#   * 默认【干跑】：gate_cmd_out:=/cmd_vel_dryrun。真机接底盘必须显式传
#     --real-cmd 且确认场地安全（双 RSP 冲突未消除前一律拒绝接底盘）。
#   * 不伪造任何 odom/定位/TF 来绕过 preflight。
#   * 不改甲方启动脚本/URDF/SLAM 参数/底盘驱动。
#
# 用法：
#   bash scripts/run_m1_safe.sh                 # preflight → 干跑（推荐）
#   bash scripts/run_m1_safe.sh --skip-preflight # 直接干跑（调试时）
#   bash scripts/run_m1_safe.sh --real-cmd      # 【高危】preflight 通过后接 /cmd_vel
#   bash scripts/run_m1_safe.sh --print        # 只打印将执行的命令，不运行
# =============================================================================
set -uo pipefail

REAL_CMD=0
SKIP_PRE=0
PRINT_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --real-cmd) REAL_CMD=1 ;;
    --skip-preflight) SKIP_PRE=1 ;;
    --print) PRINT_ONLY=1 ;;
    *) echo "未知参数: $arg"; exit 1 ;;
  esac
done

ROOT="/home/demo/lyy/hivecore-navigation"
PRE="$ROOT/scripts/preflight_hivecore.sh"

if [ "$PRINT_ONLY" = "1" ]; then
  cat <<'EOF'
将执行（--print，未实际运行）：
  1) bash scripts/preflight_hivecore.sh        # 甲方数据自检
  2) ros2 launch dynacbf_bringup dynacbf_real.launch.py \\
       navi_mode:=1 pose_topic:=/odom nav_frame:=odom \\
       pose_child_frame:=base_footprint gate_require_slam:=false \\
       bridge_detect_sec:=0.0 gate_cmd_out:=/cmd_vel_dryrun \\
       speed_limit:=0.3 omega_limit:=0.4 lidar_topic:=/livox/lidar \\
       rviz:=false
EOF
  exit 0
fi

# ---------- 1. preflight ----------
if [ "$SKIP_PRE" = "1" ]; then
  echo "[run_m1_safe] --skip-preflight：跳过 preflight，直接干跑（调试用）"
else
  echo "[run_m1_safe] 第 1 步：甲方数据自检（preflight_hivecore.sh）……"
  if ! bash "$PRE" 2>&1 | tee /tmp/m1_preflight_$(date +%H%M%S).log \
        | grep -q "全部必查项通过"; then
    echo ""
    echo "[run_m1_safe] preflight 有 FAIL —— 拒绝启动导航。"
    echo "  按 preflight 输出处理缺失项（甲方启动命令/双 RSP 处理建议见输出），"
    echo "  修复后重跑；或在确认场地安全后强制干跑（--skip-preflight，仅调试）。"
    exit 1
  fi
  echo "[run_m1_safe] preflight 通过。"
fi

# ---------- 2. 真机底盘门禁 ----------
if [ "$REAL_CMD" = "1" ]; then
  echo ""
  echo "[run_m1_safe] 你选择了 --real-cmd（速度接到底盘 /cmd_vel）。"
  echo "  警告：这会让机器人真实移动。请确认："
  echo "    ① 双 RSP 冲突已由甲方消除（否则 TF 抖动可能导致高度错误）；"
  echo "    ② 场地安全 / 车架起来 / 有人随时急停。"
  read -r -p "确认输入 YES 继续（否则 Ctrl-C）：" ans
  [ "$ans" = "YES" ] || { echo "已取消。"; exit 0; }
  GATE_OUT="/cmd_vel"
else
  GATE_OUT="/cmd_vel_dryrun"
  echo ""
  echo "[run_m1_safe] 干跑模式：gate 速度出口 = /cmd_vel_dryrun（不驱动底盘）。"
fi

# ---------- 3. 启动 M1 ----------
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROS_LOCALHOST_ONLY=0
# Humble 的 setup.bash 在 set -u 下会报 AMENT_TRACE_SETUP_FILES unbound → 临时关闭
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
set -u

cd "$ROOT"
echo "[run_m1_safe] 启动 M1 导航（gate_cmd_out=$GATE_OUT，lidar=/livox/lidar）……"
exec ros2 launch dynacbf_bringup dynacbf_real.launch.py \
  navi_mode:=1 \
  pose_topic:=/odom \
  nav_frame:=odom \
  pose_child_frame:=base_footprint \
  gate_require_slam:=false \
  bridge_detect_sec:=0.0 \
  gate_cmd_out:="$GATE_OUT" \
  speed_limit:=${SPEED_LIMIT:-0.3} \
  omega_limit:=${OMEGA_LIMIT:-0.4} \
  lidar_topic:=${LIDAR_TOPIC:-/livox/lidar} \
  rviz:=${RVIZ:-false}