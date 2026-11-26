#!/usr/bin/env bash
# 启动 slam3d lite 可视化操作台（Web GUI）
#
#   ./run_slam_lite_gui.sh            # 默认 0.0.0.0:8123
#   ./run_slam_lite_gui.sh 8090       # 指定端口
#
# 必须在“持久终端”里跑：GUI 自己退出时子线程/子进程也随之结束。
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${DIR}/../.." && pwd -P)"
export N01_NAV_REPO="${N01_NAV_REPO:-${REPO_ROOT}}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OMP_NUM_THREADS=1
export OMP_WAIT_POLICY=PASSIVE
export NUMEXPR_NUM_THREADS=1
export OPENCV_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1
# 可通过 SLAM_ROSK 指向另一份兼容的 SLAM3D Lite 综合包。
source "$REPO_ROOT/scripts/lib/slam_runtime.sh"
n01_select_slam_runtime
ROSK="$SLAM_ROSK"
export SLAM_ROSK="${ROSK}"
SETUP="${ROSK}/setup_runtime.sh"

# Start from a clean environment. Inherited AMENT_PREFIX_PATH/PYTHONPATH/
# LD_LIBRARY_PATH from an older release (e.g. ~/latest or ~/FROZEN) can make
# setup_runtime source a stale install tree even when SLAM_ROSK points at /opt.
if [[ "${N01_GUI_CLEAN_ENV:-0}" != "1" ]]; then
  if [[ ! -f "${SETUP}" ]]; then
    echo "[ERR] 找不到 ${SETUP}" >&2
    exit 1
  fi
  clean_env=(
    "HOME=${HOME}" "USER=${USER:-$(id -un)}" "LOGNAME=${LOGNAME:-$(id -un)}"
    "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    "TERM=${TERM:-xterm}" "SLAM_ROSK=${ROSK}" "N01_GUI_CLEAN_ENV=1"
  )
  for name in DISPLAY WAYLAND_DISPLAY XAUTHORITY XDG_RUNTIME_DIR \
              DBUS_SESSION_BUS_ADDRESS QT_QPA_PLATFORM ROS_DOMAIN_ID \
              RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY \
              ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS \
              CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE \
              FASTDDS_DEFAULT_PROFILES_FILE ROS_SECURITY_ENABLE \
              ROS_SECURITY_STRATEGY ROS_SECURITY_KEYSTORE ROS_LOG_DIR \
              SLAM_GUI_HOST SLAM_GUI_PORT N01_NAV_REPO N01_RUNTIME_DIR \
              N01_RUNTIME_RMW N01_CYCLONE_PREFIX N01_TRANSPORT_ROOT N01_RUNTIME_ROOT \
              N01_HIVE2_SETUP N01_HIVE2_PREFIX N01_KEEP_WHEEL_WEB \
              N01_MAPPING_BAG_DIR N01_MAPPING_BAG_TOPICS; do
    if [[ -n "${!name:-}" ]]; then
      clean_env+=("${name}=${!name}")
    fi
  done
  exec env -i "${clean_env[@]}" bash "$0" "$@"
fi

if [[ ! -f "${SETUP}" ]]; then
  echo "[ERR] 找不到 ${SETUP}" >&2
  exit 1
fi

# 关键：source Lite runtime，确保加载正确的 Lite 消息类型与动态库
# shellcheck disable=SC1090
n01_source_slam_runtime

# RSP/URDF references package://hive2_description/*.STL. The active chassis is
# /opt/hivecore/hive2-motion/current, not the legacy 1.0.0 overlay. Source it
# before the GUI/ROS worker so RViz's resource retriever can resolve meshes.
HIVE2_SETUP="${N01_HIVE2_SETUP:-/opt/hivecore/hive2-motion/current/install/setup.bash}"
if [[ -f "$HIVE2_SETUP" ]]; then
  # colcon scripts read COLCON_TRACE/COLCON_CURRENT_PREFIX without `:-`.
  # They are not nounset-safe, so temporarily disable `set -u`.
  set +u
  # shellcheck disable=SC1090
  source "$HIVE2_SETUP"
  set -u
fi

# Fail closed if the runtime still resolves any old/FROZEN release prefix.
while IFS= read -r prefix; do
  [[ -z "${prefix}" ]] && continue
  case "${prefix}" in
    "${ROSK}"/install/*|/opt/ros/humble|/opt/ros/humble/*) ;;
    *slam3d_release*|*FROZEN*)
      echo "[ERR] 检测到混入的旧 SLAM 环境前缀：${prefix}" >&2
      echo "[ERR] 请检查 ${SETUP} 的 DEST/INSTALL 路径；拒绝启动混合环境 GUI。" >&2
      exit 22
      ;;
  esac
done < <(printf '%s' "${AMENT_PREFIX_PATH:-}" | tr ':' '\n')

LOC_PREFIX="$(ros2 pkg prefix hivecore_slam3d_lite_localization 2>/dev/null || true)"
if [[ -z "${LOC_PREFIX}" || "${LOC_PREFIX}" != "${ROSK}/install/"* ]]; then
  echo "[ERR] Lite localization package prefix 不属于当前 release：${LOC_PREFIX:-未找到}" >&2
  exit 23
fi
echo "[INFO] 使用 Lite release: ${ROSK} (realpath=$(readlink -f "${ROSK}"))"
echo "[INFO] localization prefix: ${LOC_PREFIX}"

# Two-stage graph census: refresh ROS CLI discovery, then stop only a stale
# N-01 navigation stack through its own ownership-aware lifecycle entrypoint.
# Active Lite SLAM and robot drivers are protected and never killed here.
# Opening the console must not stop/start navigation or refresh ROS daemons.
# Module lifecycle operations are explicit GUI actions.

# 避免 matplotlib 在只读 HOME 下报 cache 警告（nicegui 会带起 matplotlib）
export MPLCONFIGDIR="${HOME}/.cache/matplotlib"
mkdir -p "${MPLCONFIGDIR}" 2>/dev/null || export MPLCONFIGDIR="${TMPDIR:-/tmp}/mpl_gui"

# 单实例锁：防止重复启动 GUI，避免同一 ROS domain 出现多个 GUI worker。
if ! command -v flock >/dev/null 2>&1; then
  echo "[ERR] 缺少 flock（util-linux），无法保证 GUI 单实例安全。" >&2
  exit 21
fi
LOCK_DIR="${XDG_RUNTIME_DIR:-${TMPDIR:-/tmp}}"
LOCK_FILE="${LOCK_DIR}/n01_slam_lite_gui_${UID}.lock"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "[ERR] 已有 SLAM Lite GUI 实例持有锁：${LOCK_FILE}" >&2
  echo "      请先从原终端正常退出旧 GUI，再重新启动。" >&2
  exit 20
fi

# 运行日志不写入源码包，避免打包时混入历史运行产物。
if [[ -z "${ROS_LOG_DIR:-}" ]]; then
  export ROS_LOG_DIR="${N01_GUI_LOG_DIR:-${HOME}/.ros/log}"
fi
mkdir -p "${ROS_LOG_DIR}" 2>/dev/null || export ROS_LOG_DIR="${TMPDIR:-/tmp}/ros_log_gui"
mkdir -p "${ROS_LOG_DIR}" 2>/dev/null || true
source "${REPO_ROOT}/scripts/lib/ros_transport.sh"
n01_configure_project_transport
echo "[INFO] ROS_LOG_DIR=${ROS_LOG_DIR}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW=${RMW_IMPLEMENTATION:-默认}"

export SLAM_GUI_HOST="${SLAM_GUI_HOST:-0.0.0.0}"
if [[ $# -ge 1 && -n "${1:-}" ]]; then
  export SLAM_GUI_PORT="$1"
else
  export SLAM_GUI_PORT="${SLAM_GUI_PORT:-8123}"
fi

n01_auto_release_performance() {
  # 1. 精准释放吃满整颗 CPU 核的独立底盘 Web 服务（不影响底盘驱动/舵轮/雷达/腿控）
  #    如需保留底盘 Web 调试页，启动前设 N01_KEEP_WHEEL_WEB=1 即可跳过。
  if [[ "${N01_KEEP_WHEEL_WEB:-0}" != "1" ]]; then
    local web_pids=()
    while IFS= read -r pid; do
      [[ -n "${pid}" ]] && web_pids+=("${pid}")
    done < <(ps -eo pid=,args= | awk '/wheel_controller_web_node/ && /__node:=wheel_controller_web/ && !/awk/ {print $1}')
    if [[ "${#web_pids[@]}" -gt 0 ]]; then
      echo "[INFO] [性能释放] 检测到高耗核底盘 Web 服务 wheel_controller_web (PID=${web_pids[*]})，正在精准停止..."
      kill -TERM "${web_pids[@]}" 2>/dev/null || true
      sleep 1
      local still=()
      for pid in "${web_pids[@]}"; do
        kill -0 "${pid}" 2>/dev/null && still+=("${pid}")
      done
      if [[ "${#still[@]}" -gt 0 ]]; then
        kill -KILL "${still[@]}" 2>/dev/null || true
        sleep 0.5
      fi
      echo "[INFO] [性能释放] 已释放 wheel_controller_web 占用的 CPU 核心（底盘驱动保持运行）。"
    fi
  fi
  # 2. 清理本用户 12 小时前的过期 launch 临时参数文件，防止 /tmp 堆积
  find /tmp -maxdepth 1 -user "${USER:-$(id -un)}" -name 'launch_params_*' -mmin +720 -delete 2>/dev/null || true
  echo "[INFO] [性能释放] 线程池已限单核 (BLAS/OMP/OpenCV=1)，核心导航链 NI=0，可视化节点 NI=10；$(uptime | sed 's/^ *//')"
}

n01_auto_release_performance

echo "[INFO] SLAM lite GUI 启动: http://${SLAM_GUI_HOST}:${SLAM_GUI_PORT}"
echo "[INFO] 仅启动界面与监控；建图/定位/导航由按钮手动启动，底盘和雷达由桌面底盘入口独立管理。"
echo "[INFO] 浏览器打开上面地址（远程机器可做端口转发：ssh -L ${SLAM_GUI_PORT}:127.0.0.1:${SLAM_GUI_PORT} ...）"
echo "[INFO] Ctrl+C 退出"
exec python3 -u "${DIR}/slam_lite_gui.py"
