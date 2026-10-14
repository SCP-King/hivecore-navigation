#!/usr/bin/env bash
# =============================================================================
# preflight_hivecore.sh —— 乙方 hivecore-navigation M1 启动前自检（不驱动底盘）
# =============================================================================
# 功能：在启动导航前，逐项验证甲方数据源与乙方订阅/输出是否真正可用。
#       **部分检查会真实订阅话题**（甲方 publish_only_when_subscribed 的库，
#       只能靠实际订阅才拿到数据），不能只看 `ros2 topic list`。
#
# 结论类型：
#   [PASS] 满足要求
#   [FAIL] 必须解决才能进 Ready（脚本最后会汇总 FAIL，返回非零）
#   [WARN] 不阻断启动，但建议处理（如双 RSP 冲突、旧版本话题等）
#
# 用法：
#   bash scripts/preflight_hivecore.sh                  # 默认 M1 检查
#   bash scripts/preflight_hivecore.sh --mode real     # 真机(SLAM) 检查
#   bash scripts/preflight_hivecore.sh --print-only    # 只打印依赖表，不探测
#
# 设计约束（开发运行准则 §5 乙方数据来源铁律）：
#   * 乙方数据只来自甲方实际发布内容 + 参数文本.txt，不凭空生成；
#   * 用 GUI 前置话题（geo_filtered/robot_pose/localization_status）必须在
#     launch/日志/前置检查里显式提醒用户前置条件；
#   * 本脚本**不伪造** odom/定位/TF 来绕过检查。
#
# 退出码：0 = 全部必查项通过；1 = 有 FAIL；2 = 依赖缺失（ros2 等）。
# =============================================================================
set -uo pipefail

# ---------------- 配置 ----------------
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
MODE="${1:-m1}"                     # m1 | real
PRINT_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --mode=*) MODE="${arg#--mode=}" ;;
    --print-only) PRINT_ONLY=1 ;;
    -h|--help)
      grep -E '^#' "$0" | sed -n '2,40p'; exit 0 ;;
  esac
done

# M1 目标参数（与 run_m1_safe.sh / dynacbf_real.launch.py 对齐）
# 【2026-10-14 统一】默认点云 = /livox/lidar/geo_filtered（frame=lidar_link，
#  甲方 SLAM preprocessor 产物；乙方 local_map 实测读入成功，见 docs/）。
#  显式切回原始 /livox/lidar（frame=livox_frame）时 preflight 5b 会检查其
#  实际 frame 所需 TF 是否可解析 —— 不可解析即 FAIL，不做静默自动回退。
if [ "$MODE" = "real" ]; then
  POSE_TOPIC="${POSE_TOPIC:-/robot_pose}"
  LIDAR_TOPIC="${LIDAR_TOPIC:-/livox/lidar/geo_filtered}"
  NAV_FRAME="map"
  POSE_CHILD="base_footprint"
  REQUIRE_SLAM=1
else
  POSE_TOPIC="${POSE_TOPIC:-/odom}"
  LIDAR_TOPIC="${LIDAR_TOPIC:-/livox/lidar/geo_filtered}"
  NAV_FRAME="odom"
  POSE_CHILD="base_footprint"
  REQUIRE_SLAM=0
fi

# 检查阈值（与现场实测/参数文本对齐）
ODOM_MIN_HZ=20
POINTCLOUD_MIN_HZ=2
POSE_TIMEOUT_SEC=1.0
TF_TIMEOUT_SEC=1.0

# 统计
PASS=0; FAIL=0; WARN=0
FAILED_ITEMS=()

red()   { printf '\033[31m%s\033[0m\n' "$*"; }
green() { printf '\033[32m%s\033[0m\n' "$*"; }
yellow(){ printf '\033[33m%s\033[0m\n' "$*"; }

record() { # record <level> <summary>
  local level="$1"; shift
  case "$level" in
    PASS) green "[PASS] $*"; PASS=$((PASS+1)) ;;
    FAIL) red   "[FAIL] $*"; FAIL=$((FAIL+1)); FAILED_ITEMS+=("$*") ;;
    WARN) yellow "[WARN] $*"; WARN=$((WARN+1)) ;;
  esac
}

have() { command -v "$1" >/dev/null 2>&1; }

ros2_cmd() { # 统一带超时+环境，避免命令卡死
  timeout 8 ros2 "${@}" 2>&1 || true
}

hdr() { printf '\n\033[1;36m=== %s ===\033[0m\n' "$*"; }

# ---------------- 0. 环境 ----------------
if [ "$PRINT_ONLY" = "1" ]; then
  # 只打印依赖表（见下方各节的「甲方依赖」清单）
  cat <<'EOF'
=== M1 甲方数据依赖表（按需启动） ===
话题                    | 发布节点(甲方)                    | 启动依赖                    | GUI操作 | QoS匹配要求
/odom                    | chassis_controller               | start_hive2_motion.sh        | 无      | Reliable ~50Hz
/livox/lidar/geo_filtered| lidar_cloud_preprocess_node      | GUI-SLAM启动 + enable_forward:true | 无(默认开) | BestEffort, 需订阅才发
                         |                                  | ★【乙方默认输入】frame=lidar_link；base_link→lidar_link 静态边可用
/livox/lidar (原始,可选) | livox_ros_driver2_node           | start_hive2_motion.sh        | 无      | Reliable ~10Hz
                         |                                  | ★frame=livox_frame；仅显式切回，须 5b 确认其 TF 可解析
base_link→livox_frame TF | static_tf_publisher_node(/tf_static) | GUI-SLAM启动              | 无      | static（链式可达）
base_footprint→base_link | robot_state_publisher(运动栈)     | start_hive2_motion.sh        | 无      | static
[WARN风险] 双 RSP: 运动栈 z=0.9 (hive2.config.xacro) vs SLAM栈 z=0.3 (hivecore_robot.urdf.xacro)
/robot_pose              | slam3d_node                       | GUI-SLAM + GUI Localization: Load map -> Session Start | 必须点按钮 | Reliable, 需进入 localization 才有数据
/localization_status     | slam3d_node                       | 同上                       | 必须点按钮 | Reliable
map→base_footprint TF    | slam3d_node                       | 同上                       | 必须点按钮 | dynamic
EOF
  exit 0
fi

hdr "0/ 通信环境"
export ROS_DOMAIN_ID ROS_FASTDDS_BUILTIN_TRANSPORTS 2>/dev/null || export ROS_DOMAIN_ID

rc=0
# 显式加载 ROS 环境（脚本在非交互 shell 运行，.bashrc 可能不生效；
# 必须确保 LD_LIBRARY_PATH 含 /opt/ros/humble/lib，否则 ros2 的
# librcl_action.so 等会报 ImportError）
if [ -f /home/demo/release_slam_3d/setup_runtime.sh ]; then
  # shellcheck disable=SC1091
  source /home/demo/release_slam_3d/setup_runtime.sh
elif [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if ! have ros2; then
  red "[FAIL] 找不到 ROS 2 环境（ros2 不在 PATH）"; exit 2
fi
# 确保动态库路径可用
if ! LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}" timeout 5 ros2 topic list >/dev/null 2>&1; then
  red "[FAIL] ros2 无法加载动态库（librcl_action.so 缺失）——检查 setup_runtime.sh 是否生效、LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
  exit 2
fi

if [ "${ROS_DOMAIN_ID:-}" != "55" ] && [ "${ROS_DOMAIN_ID:-}" != "" ]; then
  record WARN "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}（甲方现场为 55，建议对齐）"
else
  record PASS "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-55}"
fi

# ---------------- 1. 甲方进程已运行检查 ----------------
hdr "1/ 甲方进程状态"
echo "  以下进程应存在（缺 = 对应数据源没起）："
for pat in "hive2_driver.launch.py" "hive2_controller.launch.py" "livox_ros_driver2"; do
  if pgrep -f "$pat" >/dev/null 2>&1; then
    record PASS "甲方进程在跑: $pat"
  else
    record WARN "未发现 $pat（若已用其它方式启动可忽略；否则数据源缺失）"
  fi
done
# GUI-SLAM 是否在跑（决定 geo_filtered / livox_frame TF 是否可用）
if pgrep -f "gui_slam_console.launch.py" >/dev/null 2>&1 \
   || pgrep -f "lidar_cloud_preprocess_node" >/dev/null 2>&1 \
   || pgrep -f "static_tf_publisher_node" >/dev/null 2>&1; then
  record PASS "GUI-SLAM 进程在跑（geo_filtered/livox_frame 可用）"
  GUI_SLAM_UP=1
else
  GUI_SLAM_UP=0
  if [ "$MODE" = "m1" ]; then
    record WARN "GUI-SLAM 未运行：/livox/lidar/geo_filtered 与 base_link→livox_frame TF 不可用；若用 /livox/lidar 原始点云可暂时不依赖。"
  else
    record FAIL "GUI-SLAM 未运行（/robot_pose 与 map TF 依赖它，必须启动）"
  fi
fi

# ---------------- 2. 话题真实订阅验证 ----------------
hdr "2/ 话题存在 + 真实订阅 + 频率"
echo "  甲方 publish_only_when_subscribed=true：话题名存在 ≠ 有数据，以下实际订阅取样。"

# 2a. /odom
if ros2 topic list 2>/dev/null | grep -x /odom >/dev/null; then
  PUB=$(ros2_cmd topic info /odom | grep -c "Publisher count: [1-9]")
  if [ "$PUB" -ge 1 ]; then
    # 实际订阅 ~3.5s 数帧
    N=$(timeout 3.5 ros2 topic echo /odom --qos-reliability reliable --field header.stamp 2>/dev/null | grep -c "sec: [0-9]" || true)
    if [ "$N" -ge 2 ]; then
      HZ=$(awk "BEGIN{printf \"%.0f\", $N/3.5}")
      if [ "$HZ" -ge "$ODOM_MIN_HZ" ]; then
        record PASS "/odom 实际收到 ${N} 帧/3.5s (~${HZ}Hz, >= ${ODOM_MIN_HZ}) 发布者=chassis_controller"
      else
        record FAIL "/odom 频率偏低：~${HZ}Hz < `${ODOM_MIN_HZ}`Hz"
      fi
    else
      record FAIL "/odom 有发布者但 3.5s 内未取到数据（stamp 数=$N）——有话题无数据"
    fi
  else
    record FAIL "/odom 无发布者（需甲方运动栈 start_hive2_motion.sh）"
  fi
else
  record FAIL "/odom 话题不存在（需甲方运动栈）"
fi

# 2b. 主点云 = 设定 LIDAR_TOPIC（默认 /livox/lidar/geo_filtered，与启动参数统一）
#     ★geo_filtered 是甲方 SLAM preprocessor 产物（frame=lidar_link），需 GUI-SLAM 在跑。
#     publish_only_when_subscribed=true：话题名存在 ≠ 有数据，此处真实订阅取样。
#     → 主点云话题不存在或无真实数据 = FAIL（不做静默回退到原始点云）。
pick_lidar() {
  local t="$1"
  # 真实订阅 3.5s 数帧
  local N
  N=$(timeout 3.5 ros2 topic echo "$t" --field header.stamp 2>/dev/null | grep -c "sec: [0-9]" || true)
  echo "$N"
}
LIDAR_FRAME=""
LIDAR_N=0
if ros2 topic list 2>/dev/null | grep -x "$LIDAR_TOPIC" >/dev/null; then
  N=$(pick_lidar "$LIDAR_TOPIC")
  LIDAR_N="$N"
  PF=$(timeout 6 ros2 topic echo "$LIDAR_TOPIC" --once --field header.frame_id 2>/dev/null | head -1)
  LIDAR_FRAME="$PF"
  if [ "$N" -ge 1 ]; then
    record PASS "$LIDAR_TOPIC 实际收到 ${N} 帧/3.5s，frame_id=$PF"
  else
    record FAIL "$LIDAR_TOPIC 有但 3.5s 无数据（publish_only_when_subscribed / QoS 不匹配）"
  fi
else
  if [ "$LIDAR_TOPIC" = "/livox/lidar/geo_filtered" ]; then
    record FAIL "$LIDAR_TOPIC 不存在（默认输入需 GUI-SLAM 启动 + enable_forward:true；显式切回 /livox/lidar 需运动栈）"
  else
    record FAIL "$LIDAR_TOPIC 不存在（请确认所配点云已由甲方启动）"
  fi
fi

# 2c. 备选点云（只读提示，不替代主检查；不影响默认输入）
#     默认 geo_filtered 时备选=原始 /livox/lidar；显式切回原始时备选=geo_filtered。
if [ "$LIDAR_TOPIC" = "/livox/lidar/geo_filtered" ]; then
  ALT="/livox/lidar"
elif [ "$LIDAR_TOPIC" = "/livox/lidar" ]; then
  ALT="/livox/lidar/geo_filtered"
else
  ALT=""
fi
if [ -n "$ALT" ]; then
  if ros2 topic list 2>/dev/null | grep -x "$ALT" >/dev/null; then
    AN=$(pick_lidar "$ALT")
    if [ "$AN" -ge 1 ]; then
      record PASS "备选点云 $ALT 也有数据（${AN} 帧/3.5s）；显式切回它须先在 5b 确认其 frame 对应 TF"
    else
      record WARN "备选点云 $ALT 存在但无数据（可能未订阅触发 / enable_forward 关闭）"
    fi
  else
    record WARN "备选点云 $ALT 不存在（默认输入不受影响，可忽略）"
  fi
fi

# ---------------- 3. 位姿话题（M1=/odom 已检；真机=/robot_pose） ----------------
hdr "3/ 位姿话题（$POSE_TOPIC）"
if ros2 topic list 2>/dev/null | grep -x "$POSE_TOPIC" >/dev/null; then
  N=$(timeout 3.5 ros2 topic echo "$POSE_TOPIC" --qos-reliability reliable --field header.stamp 2>/dev/null | grep -c "sec: [0-9]" || true)
  if [ "$N" -ge 2 ]; then
    record PASS "$POSE_TOPIC 实际 ${N} 帧/3.5s"
  else
    if [ "$MODE" = "real" ]; then
      record FAIL "$POSE_TOPIC 无数据（真机需 GUI Localization: Load map → Session Start）"
    else
      # M1 /odom 已在 2a 检过，这里不必重复
      record PASS "M1 $POSE_TOPIC 已在 /odom 节验证"
    fi
  fi
else
  if [ "$MODE" = "real" ]; then
    record FAIL "$POSE_TOPIC 不存在（需启动 GUI-SLAM）"
  else
    record PASS "M1 用 /odom（已在 2a 验证）"
  fi
fi

# ---------------- 4. QoS 匹配 ----------------
hdr "4/ QoS 兼容性"
# geo_filtered 是 BEST_EFFORT，乙方 local_map 用 SensorDataQoS=BestEffort → 完全匹配
# /livox/lidar 是 RELIABLE，乙方 BestEffort 订阅 → ROS2 自动降级兼容（rel→be）
# 只在实际取到点云数据时记 PASS（否则 QoS 匹配从未被证真）。
case "$LIDAR_TOPIC" in
  /livox/lidar/geo_filtered)
    if [ -n "$LIDAR_FRAME" ]; then
      record PASS "geo_filtered (BestEffort pub) vs 乙方 SensorDataQoS(BestEffort sub)：完全匹配"
    else
      record WARN "geo_filtered 未取到数据，QoS 匹配未实际验证"
    fi
    ;;
  /livox/lidar)
    if [ -n "$LIDAR_FRAME" ]; then
      record PASS "/livox/lidar (RELIABLE pub) vs 乙方 SensorDataQoS(BestEffort sub)：兼容（rel→be 自动降级）"
    else
      record WARN "/livox/lidar 未取到数据，QoS 匹配未实际验证"
    fi
    ;;
  *)
    record WARN "LIDAR_TOPIC=$LIDAR_TOPIC 不在已知 QoS 映射表，请人工确认兼容性"
    ;;
esac

# ---------------- 5. TF 链连通性 ----------------
hdr "5/ TF 链（M1 最小链）"
# tf2_echo 首次调用会先打印 "Waiting ... frame does not exist"（buffer 未就绪），
# 同一进程内后续调用才出 Translation；故每个变换轮询多次直至拿全真实值。
tf_snapshot() { # tf_snapshot <parent> <child> → "Translation: [...]"（轮询轮次累积）
  # 【2026-10-14 加固】ros2 run tf2_ros tf2_echo 冷启动在系统负载下可达 ~4s，
  # timeout 3 会偶发掐断导致"真实存在的边"被误报不可用 → 每轮放宽到 5s、轮次 5 轮。
  local p="$1" c="$2" i out
  out=""
  for i in 1 2 3 4 5; do
    out="$out $(timeout 5 ros2 run tf2_ros tf2_echo "$p" "$c" 2>/dev/null | grep -m1 "Translation: " || true)"
    [ "$(echo "$out" | grep -c "Translation:")" -ge 1 ] && break
  done
  echo "$out"
}
tf_ok() { # tf_ok <parent> <child> → 0 如果至少拿到一次 Translation
  # 【修正 2026-10-14】grep 空结果时 tf_snapshot 返回的是 4 个空格（非空字符串），
  # `[ -n "$out" ]` 会把纯空白判为存在 → 假阳性。必须确认实际含 "Translation:"。
  local out; out=$(tf_snapshot "$1" "$2")
  [[ "$out" == *"Translation:"* ]]
}
tf_zset() { # tf_zset <snapshot> → 该变换所有采到的 z 值（去重）
  echo "$1" | grep -oE "\[[-0-9., ]+\]" | tr -d '[]' | cut -d',' -f3
}

# 5a. base_footprint → base_link
BF_BL=$(tf_snapshot base_footprint base_link)
if tf_ok base_footprint base_link; then
  record PASS "TF base_footprint→base_link 可用：$(echo "$BF_BL" | grep -m1 'Translation: ')"
else
  record FAIL "TF base_footprint→base_link 不可用（需运动栈 RSP）"
fi

# 5b. 点云 frame 对应 TF（按 2b 实际 frame_id 判，不按话题字符串猜）
#     geo_filtered 点云 frame=lidar_link  → 需 base_link→lidar_link 可解析
#     原始 /livox/lidar 点云 frame=livox_frame → 需 base_link→livox_frame 可解析
#     frame_id 取不到或两者皆非 → FAIL（TF 链无法确认，不做静默猜测回退）
if [ -z "$LIDAR_FRAME" ]; then
  record FAIL "2b 未取到点云实际 frame_id，TF 链无法确认（点云不可用）"
elif [ "$LIDAR_FRAME" = "lidar_link" ]; then
  BL_LL=$(tf_snapshot base_link lidar_link)
  if tf_ok base_link lidar_link; then
    record PASS "TF base_link→lidar_link 可用（点云 frame=lidar_link 可转出）：$(echo "$BL_LL" | grep -m1 'Translation: ')"
  else
    record FAIL "TF base_link→lidar_link 不可用：点云 frame=lidar_link 无法转出。需 GUI-SLAM 的 static_tf_publisher_node 提供该静态边（运动栈 RSP 不保证发布）"
  fi
elif [ "$LIDAR_FRAME" = "livox_frame" ]; then
  BL_LV=$(tf_snapshot base_link livox_frame)
  if tf_ok base_link livox_frame; then
    record PASS "TF base_link→livox_frame 可用（点云 frame=livox_frame 可转出）：$(echo "$BL_LV" | grep -m1 'Translation: ')"
  else
    record FAIL "TF base_link→livox_frame 不可用：点云 frame=livox_frame 无法转出。需启动 GUI-SLAM（static_tf_publisher）"
  fi
else
  record FAIL "点云 frame_id=$LIDAR_FRAME 与预期(lidar_link/livox_frame)不符，TF 链无法确认"
fi

# 5c. 【双 RSP 冲突检查】base_footprint→base_link 是否多定义
hdr "5c/ TF 多定义冲突（双 RSP）"
# 判据① ：/tf_static 上 base_footprint→base_link 边的发布者数量。
#         >1 个 robot_state_publisher（运动栈 hive2 与 SLAM 栈 hivecore_robot）即冲突。
#         【修正 2026-10-14】只数 Node name 为 robot_state_publisher 的 PUBLISHER 端点
#         （预期 2 个 = 双 RSP）；static_tf_publisher_node（base_link→lidar_link/imu_link
#         静态边）不是 base_footprint→base_link 的发布者，不计入冲突数。
DUP_PUBS=$(ros2 topic info /tf_static -v 2>&1 | awk '
  /^Node name: / { n=$0; sub(/^Node name: \/?/, "", n) }
  /^Endpoint type: PUBLISHER/ && n == "robot_state_publisher" { c++ }
  END { print c+0 }' || true)
# 判据② ：跨 6 次独立进程采样 base_footprint→base_link 的 z，取集合。
ALT_Z=""
for i in 1 2 3 4 5 6; do
  z=$(timeout 3 ros2 run tf2_ros tf2_echo base_footprint base_link 2>/dev/null | grep -m1 "Translation: " | grep -oE "\[[-0-9., ]+\]" | tr -d '[]' | cut -d',' -f3)
  [ -n "$z" ] && ALT_Z="$ALT_Z $z"
  sleep 0.2
done
UNIQ=$(echo "$ALT_Z" | tr ' ' '\n' | sort -u | grep -v '^$')
CNT=$(printf '%s\n' "$UNIQ" | grep -c . || true)

if [ "$DUP_PUBS" -ge 2 ] || { [ -n "$UNIQ" ] && [ "$CNT" -ge 2 ]; }; then
  red "  ---------- 双 RSP 冲突确认（FAIL） ----------"
  [ "$DUP_PUBS" -ge 2 ] && \
    record FAIL "/tf_static 上 base_footprint→base_link 有 ${DUP_PUBS} 个发布者（双 robot_state_publisher）"
  [ -n "$UNIQ" ] && [ "$CNT" -ge 2 ] && \
    record FAIL "base_footprint→base_link 采样到 ${CNT} 种 z：z=$(echo $UNIQ | tr '\n' '/')（0.9 运动栈 / 0.3 SLAM 栈间歇覆盖）"
  record WARN "来源①运动栈 URDF z=0.9：/opt/hivecore/hive2-motion/1.0.0/install/share/hive2_driver/description/hive2.config.xacro"
  record WARN "来源②SLAM 栈 URDF z=0.3：/home/demo/release_slam_3d/install/hivecore_robot_description/share/hivecore_robot_description/urdf/hivecore_robot.urdf.xacro (footprint_xyz default)"
  record WARN "影响：任何查询 base_footprint→base_link 的地方 z 会 0.3/0.9 抖动；乙方不代做、不掩盖。"
  record WARN "乙方规避：M1 下 controller/local_map/gate 直接消费 /odom + lidar_link 静态边，不查此边；planner/可视化若用会受影响。"
  record WARN "处理建议：甲方确认正确几何（现场实测 z=0.9，参数文本§9），在 SLAM 栈 URDF/launch 停用冲突 RSP，乙方不改甲方。"
elif [ -z "$UNIQ" ]; then
  record WARN "未能采到 base_footprint→base_link z 值，跳过双 RSP 判定"
else
  record PASS "base_footprint→base_link 唯一（发布者=${DUP_PUBS}，采样 z=$(echo $UNIQ | tr '\n' '/')）——无双 RSP 冲突"
fi

# ---------------- 6. 乙方订阅/输出结构（只读 rosnode 检查） ----------------
hdr "6/ 乙方相关话题结构（只读，不启动导航）"
echo "  导航未启动时无乙方节点，跳过输出检查；仅提示乙方将发布："
echo "    /curr_state /local_plan /planning/bspline /global_path /cmd_vel_nav /for_obs_track"

# ---------------- 汇总 ----------------
printf '\n\033[1;36m===== 自检汇总 =====\033[0m\n'
printf '  PASS: %d   FAIL: %d   WARN: %d\n' "$PASS" "$FAIL" "$WARN"
if [ "$FAIL" -gt 0 ]; then
  printf '\n\033[1;31m以下必查项未通过（按顺序处理）：\033[0m\n'
  for it in "${FAILED_ITEMS[@]}"; do
    printf '  - %s\n' "$it"
  done
  exit 1
fi
printf '\n\033[1;32m全部必查项通过 → 可进入 M1 启动。\033[0m\n'
echo "提示：仍须以 gate_cmd_out:=/cmd_vel_dryrun 干跑验证（preflight 不驱动底盘）。"
exit 0