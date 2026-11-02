#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

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
  source "$LIVOX_SETUP"
fi

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "[N01-FAST-LIO2] Expected ROS_DISTRO=humble, got ${ROS_DISTRO:-unset}" >&2
  exit 2
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo "[N01-FAST-LIO2] colcon is missing; install ROS 2 build tools first." >&2
  exit 2
fi
if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
  echo "[N01-FAST-LIO2] livox_ros_driver2 is not in the current ROS environment." >&2
  echo "Source the already-installed Livox ROS Driver2 overlay, or set:" >&2
  echo "  LIVOX_SETUP=/path/to/livox_workspace/install/setup.bash" >&2
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
