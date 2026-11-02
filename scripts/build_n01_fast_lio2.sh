#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Colcon-generated local_setup.bash scripts may read COLCON_CURRENT_PREFIX
# before initializing it. This build script uses nounset, so source overlays
# with -u temporarily disabled, then restore strict mode immediately.
source_overlay_compat() {
  local setup_file="$1"
  local source_rc=0
  set +u
  if source "$setup_file"; then
    source_rc=0
  else
    source_rc=$?
  fi
  set -u
  return "$source_rc"
}

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[N01-FAST-LIO2] ROS 2 Humble not found at /opt/ros/humble." >&2
  exit 2
fi
# Source ROS underlay first, then an optional existing Livox driver overlay.
if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -n "${LIVOX_SETUP:-}" ]]; then
  if [[ ! -f "$LIVOX_SETUP" ]]; then
    echo "[N01-FAST-LIO2] LIVOX_SETUP does not exist: $LIVOX_SETUP" >&2
    exit 2
  fi
  source_overlay_compat "$LIVOX_SETUP"
fi

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "[N01-FAST-LIO2] Expected ROS_DISTRO=humble, got ${ROS_DISTRO:-unset}" >&2
  exit 2
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo "[N01-FAST-LIO2] colcon is missing; install ROS 2 build tools first." >&2
  exit 2
fi
# Some Hive2 deployments expose Livox through the motion-stack install
# prefix. Prefer its local_setup.bash: setup.bash may chain a build-time
# underlay that no longer exists (for example an old FROZEN workspace).
if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
  HIVE2_LIVOX_LOCAL_SETUP="${HIVE2_LIVOX_LOCAL_SETUP:-/opt/hivecore/hive2-motion/1.0.0/install/local_setup.bash}"
  if [[ -f "$HIVE2_LIVOX_LOCAL_SETUP" ]]; then
    echo "[N01-FAST-LIO2] Livox not visible yet; sourcing existing Hive2 overlay:"
    echo "  $HIVE2_LIVOX_LOCAL_SETUP"
    source_overlay_compat "$HIVE2_LIVOX_LOCAL_SETUP"
  fi
fi

if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
  echo "[N01-FAST-LIO2] livox_ros_driver2 is installed but not discoverable in this build process." >&2
  echo "  ROS_DISTRO=${ROS_DISTRO:-unset}" >&2
  echo "  ros2=$(command -v ros2 || echo missing)" >&2
  echo "  AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-unset}" >&2
  echo "If needed, run with the existing Hive2 local overlay:" >&2
  echo "  LIVOX_SETUP=/opt/hivecore/hive2-motion/1.0.0/install/local_setup.bash \\" >&2
  echo "    bash scripts/build_n01_fast_lio2.sh" >&2
  echo "No second Livox driver will be cloned or installed by this script." >&2
  exit 2
fi

cd "$ROOT"
bash scripts/import_n01_fast_lio2.sh

echo "[N01-FAST-LIO2] Building only FAST-LIO2 and its N-01 wrapper/dependencies."
echo "[N01-FAST-LIO2] No apt/rosdep package installation is performed automatically."
colcon build --packages-up-to n01_fast_lio \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+

source "$ROOT/install/setup.bash"
ros2 pkg prefix fast_lio
ros2 pkg prefix n01_fast_lio
echo "[N01-FAST-LIO2] Build complete. No SLAM node or robot hardware was started."
