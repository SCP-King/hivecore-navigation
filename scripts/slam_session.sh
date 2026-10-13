#!/usr/bin/env bash
# ============================================================================
# slam_session.sh —— 甲方 3D SLAM 纯命令行会话脚本（建图/录包/可视化/保存）
# ============================================================================
# 目的：完全替代甲方 Tk GUI（slam3d_services_gui），全程只调甲方已有的
#       launch 与服务，不修改甲方任何参数、代码、配置。
#       每一条命令 = 一个子命令，在对应终端前台运行，职责清晰。
#
# 使用的甲方资源（仅调用）：
#   - ros2 launch hivecore_console_slam3d mapping_console.launch.py
#   - ros2 launch hivecore_slam3d includes/vis_world_viewpoint.launch.py
#   - slam3d_node 系列服务（start_mapping / set_enable_publish_cloud_map /
#     publish_cloud_map_once / save_cloud_map_once / stop_mapping ...）
#
# 默认命名：默认名 + 时间戳（跨命令一致，由本脚本状态文件保证）
#   地图默认  ~/lyy/slam_data/maps/nav_map_<时间戳>.pcd
#   包  默认  ~/lyy/slam_data/bags/livox_bag_<时间戳>
#   时间戳格式同甲方 GUI 的 {stamp}：%Y_%m_%d-%H_%M_%S
# 自定义：子命令可带 [MAP_NAME] / [BAG_NAME]，或用环境变量覆盖，见各子命令。
#
# 用法（四个终端，按标注跑对应命令）：
#   bash scripts/slam_session.sh help
#
#   [终端A] 启动建图节点（常驻，结束时 Ctrl+C）
#     bash scripts/slam_session.sh launch
#
#   [终端B] 开始建图（等节点就绪后自动 start_mapping + 开启地图发布）
#     bash scripts/slam_session.sh start                    # 默认名+时间戳
#     bash scripts/slam_session.sh start 楼栋_a             # 自定义地图名
#     env MAP_NAME=site2 bash scripts/slam_session.sh start
#
#   [终端C] 开始/取消录包（常驻，脚本会自动记录 PID 供 finish 结束）
#     bash scripts/slam_session.sh bag                      # 默认包名+时间戳
#     bash scripts/slam_session.sh bag 楼栋_a               # 自定义包名
#
#   [终端B 或新终端] 结束并保存：存地图 + 停建图 + 停录包
#     bash scripts/slam_session.sh finish
#     bash scripts/slam_session.sh finish site2_map         # 显式覆盖地图名
#
#   [终端D] 启动甲方世界视角 RViz（末尾 Ctrl+C 关闭）
#     bash scripts/slam_session.sh viz
#
# 提示：各终端都建议先 source 一遍环境；脚本内部也会 source，因此可不重复。
# ============================================================================

set -euo pipefail

# ---------------- 可配置项（环境变量可覆盖） ----------------
: "${SLAM_DIR:=$HOME/release_slam_3d}"        # 甲方 SLAM 工作区
: "${ROS_DOMAIN_ID:=55}"
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
: "${DEFAULT_MAP_DIR:=$HOME/lyy/slam_data/maps}"   # 地图输出目录（默认）
: "${DEFAULT_BAG_DIR:=$HOME/lyy/slam_data/bags}"   # 包输出目录（默认）
: "${DEFAULT_MAP_NAME:=nav_map}"               # 地图默认名
: "${DEFAULT_BAG_NAME:=livox_bag}"             # 包默认名
: "${STATE_DIR:=$HOME/lyy/slam_data/.hivecore_slam_session}"   # 跨命令会话状态

# ---------------- 工具函数 ----------------
_log()   { printf '[slam] %s\n' "$*"; }
_err()   { printf '[slam] 错误: %s\n' "$*" >&2; exit 1; }

_source_env() {
  # 幂等：内部 source 一次甲方运行时，并固定 domain / rmw。
  if [ -f "$SLAM_DIR/setup_runtime.sh" ]; then
    # shellcheck disable=SC1090,SC1091
    source "$SLAM_DIR/setup_runtime.sh"
  else
    _err "找不到 $SLAM_DIR/setup_runtime.sh"
  fi
  export ROS_DOMAIN_ID RMW_IMPLEMENTATION
}

_stamp() {
  date +%Y_%m_%d-%H_%M_%S
}

_new_stamp_file() {
  local stamp map_name bag_name map_dir bag_dir
  stamp=$(_stamp)
  map_name="${1:-$DEFAULT_MAP_NAME}"
  bag_name="${2:-$DEFAULT_BAG_NAME}"
  map_dir="${MAP_DIR:-$DEFAULT_MAP_DIR}"
  bag_dir="${BAG_DIR:-$DEFAULT_BAG_DIR}"
  mkdir -p "$map_dir" "$bag_dir" "$STATE_DIR"
  # 关键：生成“本次会话”统一的名字，写状态文件，其余子命令读取。
  cat > "$STATE_DIR/latest" <<EOF
stamp=$stamp
map_name=$map_name
bag_name=$bag_name
map_dir=$map_dir
bag_dir=$bag_dir
map_pcd=$map_dir/${map_name}_$stamp.pcd
bag_uri=$bag_dir/${bag_name}_$stamp
EOF
}

_load_stamp_file() {
  # 从状态文件还原本次会话的名字；返回非零表示无现存会话。
  [ -f "$STATE_DIR/latest" ] || { _log "尚无活跃会话（请先运行 start）。"; return 1; }
  # shellcheck disable=SC1091
  source "$STATE_DIR/latest"
  true
}

_wait_slam3d() {
  _log "等待 slam3d_node 就绪 ..."
  for _ in $(seq 1 45); do
    # ros2 service list 输出形如 /slam3d_node/start_mapping（带前导斜杠），
    # 故不能用 ^ 锚定行首；用子串匹配。
    if ros2 service list 2>/dev/null | grep -q 'slam3d_node/'; then
      _log "slam3d_node 就绪。"
      return 0
    fi
    sleep 1
  done
  _err "等待超时：没发现 slam3d_node/... 服务。请确认 [终端A] 的 mapping_console 是否已启动（环境：DOMAIN=$ROS_DOMAIN_ID）。"
}

_call() { # 打印并调用 ros2 service call（带统一环境）
  local srv="$1" type="$2" args="${3:-}"
  _log "服务调用: $srv"
  # args 是一个 YAML 字符串（如 {data: true}），必须整体作为单个 argv 传给
  # ros2 service call，不能去掉引号拆词。
  ros2 service call "$srv" "$type" "$args"
}

_call_tolerant() { # 同 _call，但失败/超时不中断脚本（收尾用，保证后续步骤一定执行）
  local srv="$1" type="$2" args="${3:-}" secs="${4:-120}" rc
  _log "服务调用: $srv（容忍失败，上限 ${secs}s）"
  timeout "$secs" ros2 service call "$srv" "$type" "$args"
  rc=$?
  if [ "$rc" -eq 0 ]; then
    return 0
  fi
  _log "警告: $srv 未成功返回(rc=$rc)。继续执行后续步骤。"
  return 1
}

# ---------------- 子命令 ----------------
cmd_help() {
  sed -n '2,44p' "$0"
}

cmd_launch() {
  # [终端A] 前台启动甲方建图控制台（含预处理 + slam3d_node），常驻。
  _source_env
  _log "启动 mapping_console（按住 Ctrl+C 可整体退出）"
  exec ros2 launch hivecore_console_slam3d mapping_console.launch.py
}

cmd_start() {
  # [终端B] 生成会话名字 → 等待节点 → 复刻 GUI「Start」的排它流程：
  #   _start_mapping_exclusive: 若 session 非 stopped → stop_mapping
  #                              → enter_idle → enter_mapping → start_mapping
  #   （GUI 在 session=stopped 时也照样 enter_idle→enter_mapping→start_mapping）
  _source_env
  _new_stamp_file "${1:-$DEFAULT_MAP_NAME}" "${2:-$DEFAULT_BAG_NAME}"
  local map_pcd bag_uri
  map_pcd=$(awk -F= '/^map_pcd=/{print $2}' "$STATE_DIR/latest")
  bag_uri=$(awk -F= '/^bag_uri=/{print $2}' "$STATE_DIR/latest")
  _wait_slam3d
  _log "本次会话："
  _log "  地图（finish 保存到此）: $map_pcd"
  _log "  包（bag 子命令录到此）:  $bag_uri"

  # 与 GUI 一致：非 stopped 才 stop_mapping（GUI 用 Trigger，失败不阻断流程）
  local mode
  mode=$(ros2 topic echo --once /mapping_status --field control_signal_session_mode 2>/dev/null \
        | sed -n '1p' | tr -d '[:space:]' || true)
  mode="${mode,,}"
  if [ -n "$mode" ] && [ "$mode" != "stopped" ]; then
    _call slam3d_node/stop_mapping std_srvs/srv/Trigger {}
  else
    _log "session=${mode:-?}（stopped/未知）→ 跳过 stop_mapping（与 GUI 一致）"
  fi
  _call slam3d_node/enter_idle std_srvs/srv/Trigger {}
  _call slam3d_node/enter_mapping std_srvs/srv/Trigger {}
  _call slam3d_node/start_mapping std_srvs/srv/Trigger {}
  # 与 GUI 的 Publish regularly ON + Publish once 对齐
  # （GUI 会先关 localization 的定期发布，保证互斥）
  _call slam3d_node/set_enable_publish_localization_map std_srvs/srv/SetBool "{data: false}"
  _call slam3d_node/set_enable_publish_cloud_map std_srvs/srv/SetBool "{data: true}"
  _call slam3d_node/publish_cloud_map_once std_srvs/srv/Trigger {}
  _log "建图已开始（效果=GUI 依次点击 Start、Publish regularly ON、Publish once）。"
  _log "可启用 [终端B] 录包： bash scripts/slam_session.sh bag"
  _log "完成后结束： bash scripts/slam_session.sh finish"
}

cmd_bag() {
  # [终端C] 按状态文件名启动录包（前台常驻；finish 会自动 SIGINT 它优雅关闭）。
  _source_env
  _load_stamp_file || return 1
  local bag_uri bag_name
  bag_uri=$(awk -F= '/^bag_uri=/{print $2}' "$STATE_DIR/latest")
  bag_name=$(awk -F= '/^bag_name=/{print $2}' "$STATE_DIR/latest")
  # 允许显式覆盖包名（生成新 stamp 保存在状态里，避免 finish 找错）
  if [ -n "${1:-}" ]; then
    _new_stamp_file "${1:-$DEFAULT_MAP_NAME}" "$1"
    bag_uri=$(awk -F= '/^bag_uri=/{print $2}' "$STATE_DIR/latest")
  fi
  mkdir -p "$(dirname "$bag_uri")"
  _log "开始录制到 $bag_uri（Ctrl+C 或运行 finish 停止）"
  ros2 bag record -o "$bag_uri" /livox/lidar /livox/imu &
  local rec_pid=$!
  echo "$rec_pid" > "$STATE_DIR/bag.pid"
  _log "录包 PID=$rec_pid"
  # 前台保持：等待 bag 结束；期间用户按 Ctrl+C 同样会优雅关闭。
  set +e
  wait "$rec_pid"
  local rc=$?
  set -e
  rm -f "$STATE_DIR/bag.pid"
  _log "录包已停止（exit=$rc），文件：$bag_uri"
}

cmd_finish() {
  # [终端B/新终端] 保存地图（本会话统一名字，可显式覆盖）→ 停建图 → 停录包。
  _source_env
  _load_stamp_file || return 1
  local map_pcd bag_uri save_name
  map_pcd=$(awk -F= '/^map_pcd=/{print $2}' "$STATE_DIR/latest")
  bag_uri=$(awk -F= '/^bag_uri=/{print $2}' "$STATE_DIR/latest")
  if [ -n "${1:-}" ]; then # 只有真的给了名字才覆盖（避免空参被当显式名）
    local dir name
    dir=$(dirname "$map_pcd")
    map_pcd="$dir/${1}_$(awk -F= '/^stamp=/{print $2}' "$STATE_DIR/latest").pcd"
    _log "覆盖地图名 → $map_pcd"
  fi
  mkdir -p "$(dirname "$map_pcd")"

  # 保存地图是最重的操作：设置上限并在失败时继续收尾，避免 bag 无法封口。
  _log "保存地图中（大图可能需要数分钟，请耐心等待）..."
  local save_ok=0 rc
  timeout 1800 ros2 service call slam3d_node/save_cloud_map_once \
    hivecore_slam3d/srv/SaveCloudMap "{file_path: '$map_pcd'}"
  rc=$?
  if [ "$rc" -eq 0 ]; then
    save_ok=1
  else
    _log "警告: save_cloud_map_once 未返回成功(rc=$rc)。用文件存在性确认..."
  fi

  # 服务端可能已接受请求但响应较晚，轮询确认文件是否落盘。
  if [ "$save_ok" -eq 0 ]; then
    for _ in $(seq 1 60); do
      if [ -s "$map_pcd" ]; then
        save_ok=1
        break
      fi
      sleep 5
    done
  fi
  if [ "$save_ok" -eq 1 ]; then
    _log "地图已保存：$map_pcd"
  else
    _log "警告: 未能确认地图落盘（$map_pcd）。请稍后手动检查或重试保存。"
  fi

  # 停建图和关闭发布必须继续执行，不能因保存服务失败而中断。
  _call_tolerant slam3d_node/stop_mapping std_srvs/srv/Trigger {} 120 || true
  _call_tolerant slam3d_node/set_enable_publish_cloud_map std_srvs/srv/SetBool "{data: false}" 120 || true
  _log "建图已停止。"

  # 结束录包（若 bag 子命令还有进程存活）：优雅 SIGINT。
  if [ -f "$STATE_DIR/bag.pid" ]; then
    local bp
    bp=$(cat "$STATE_DIR/bag.pid")
    if kill -0 "$bp" 2>/dev/null; then
      _log "检测到录包进程存活（PID $bp），发送 SIGINT 正常封口（约需数秒到数分钟）..."
      kill -INT "$bp" 2>/dev/null || true
      # 更长的等待：3.6GB 规模的 bag 封口可能很慢；等进程自然退出。
      for _ in $(seq 1 120); do
        kill -0 "$bp" 2>/dev/null || break
        sleep 1
      done
      rm -f "$STATE_DIR/bag.pid"
    else
      _log "bag.pid 存在但进程已不在（可能已封口/Dataloader 结束），跳过停包。"
      rm -f "$STATE_DIR/bag.pid"
    fi
  else
    _log "未找到 bag.pid（03 录包可能未启动或已停止）。"
  fi

  _log "会话完成："
  if [ -s "$map_pcd" ]; then
    _log "  地图: $(ls -lh "$map_pcd" | awk '{print $5", "$9}')"
  else
    _log "  地图: （未找到 $map_pcd —— 请用 ros2 service call save_cloud_map_once 重试）"
  fi
  _log "  建图: 已停止。"
  if compgen -G "$bag_uri*" > /dev/null; then
    _log "  包:   $bag_uri  （如需查看： ros2 bag info $bag_uri）"
  else
    _log "  包:   （未找到 $bag_uri*）"
  fi
  _log "  现在可以到 [终端A/终端D] 按 Ctrl+C 关闭建图与 RViz。"
}

cmd_viz() {
  # [终端D] 甲方世界视角 RViz。
  _source_env
  # 桌面双击自带 DISPLAY；shell 直接调用可能没有，给个 :0 兜底。
  export DISPLAY="${DISPLAY:-:0}"
  _log "启动世界视角 RViz（DISPLAY=$DISPLAY，打开后手动 Ctrl+C 关闭）"
  local vis_launch="$SLAM_DIR/install/hivecore_slam3d/share/hivecore_slam3d/launch/includes/vis_world_viewpoint.launch.py"
  if [ ! -f "$vis_launch" ]; then
    _err "找不到甲方 RViz launch 文件: $vis_launch"
  fi
  # nested includes/ 路径用绝对文件启动；ros2 launch <pkg> <file>
  # 只解析 share/<pkg>/launch 下的单层文件名。
  exec ros2 launch "$vis_launch"
}

cmd_stop() {
  # 应急：仅停止建图（不保存）。等价 GUI「Stop」。
  _source_env
  _call slam3d_node/stop_mapping std_srvs/srv/Trigger {}
  _call slam3d_node/set_enable_publish_cloud_map std_srvs/srv/SetBool "{data: false}"
  _log "建图已停止（未保存地图）。"
}

# ---------------- 分发 ----------------
case "${1:-help}" in
  launch)  cmd_launch ;;
  start)   cmd_start "${2:-}" "${3:-}" ;;
  bag)     cmd_bag "${2:-}" ;;
  finish)  cmd_finish "${2:-}" ;;
  viz)     cmd_viz ;;
  stop)    cmd_stop ;;
  help|-h|--help)
    cmd_help
    ;;
  *)
    _err "未知子命令: $1（可用: launch | start [NAME] | bag [NAME] | finish [NAME] | stop | viz | help）" ;;
esac