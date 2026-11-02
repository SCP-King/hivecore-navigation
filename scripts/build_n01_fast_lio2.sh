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
# Validate the installed ROS package through the ament resource index.
# ros2 pkg prefix is not used as the build gate: it can fail in a mixed
# overlay environment even when the indexed package and CMake config exist.
ament_package_prefix() {
  local package_name="$1"
  local prefix
  local -a prefixes=()
  local IFS=:
  read -r -a prefixes <<< "${AMENT_PREFIX_PATH:-}"
  for prefix in "${prefixes[@]}"; do
    if [[ -n "$prefix" && -f "$prefix/share/ament_index/resource_index/packages/$package_name" ]]; then
      printf '%s\n' "$prefix"
      return 0
    fi
  done
  return 1
}

LIVOX_PREFIX=""
if ! LIVOX_PREFIX="$(ament_package_prefix livox_ros_driver2)"; then
  HIVE2_LIVOX_LOCAL_SETUP="${HIVE2_LIVOX_LOCAL_SETUP:-/opt/hivecore/hive2-motion/1.0.0/install/local_setup.bash}"
  if [[ -f "$HIVE2_LIVOX_LOCAL_SETUP" ]]; then
    echo "[N01-FAST-LIO2] Livox ament index not in current prefixes; sourcing:"
    echo "  $HIVE2_LIVOX_LOCAL_SETUP"
    source_overlay_compat "$HIVE2_LIVOX_LOCAL_SETUP"
  fi
  LIVOX_PREFIX="$(ament_package_prefix livox_ros_driver2 || true)"
fi

if [[ -z "$LIVOX_PREFIX" ]]; then
  echo "[N01-FAST-LIO2] Livox ament resource index not found in AMENT_PREFIX_PATH." >&2
  echo "  AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-unset}" >&2
  echo "Expected index: <prefix>/share/ament_index/resource_index/packages/livox_ros_driver2" >&2
  echo "No second Livox driver will be cloned or installed by this script." >&2
  exit 2
fi

LIVOX_CMAKE_CONFIG="$LIVOX_PREFIX/share/livox_ros_driver2/cmake/livox_ros_driver2Config.cmake"
if [[ ! -f "$LIVOX_CMAKE_CONFIG" ]]; then
  echo "[N01-FAST-LIO2] Livox ament index exists but CMake config is missing:" >&2
  echo "  $LIVOX_CMAKE_CONFIG" >&2
  exit 2
fi

# Make the already-installed Hive2 package discoverable to CMake as well.
case ":$${CMAKE_PREFIX_PATH:-}:" in
  *":$LIVOX_PREFIX:"*) ;;
  *) export CMAKE_PREFIX_PATH="$LIVOX_PREFIX:${CMAKE_PREFIX_PATH:-}" ;;
esac
echo "[N01-FAST-LIO2] Reusing installed Livox package: $LIVOX_PREFIX"

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
