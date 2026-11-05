#!/usr/bin/env bash
# Read-only N-01 host/workspace preflight. Writes only its own report file.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
RUNTIME_ROOT="${N01_RUNTIME_ROOT:-$HOME/lyy/n01_runtime}"
REPORT_DIR="$RUNTIME_ROOT/runs/environment_preflight"
STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$REPORT_DIR"
REPORT="$REPORT_DIR/n01_environment_$STAMP.txt"
exec > >(tee -a "$REPORT") 2>&1

# shellcheck disable=SC1091
source "$ROOT/scripts/lib/ros_env.sh"

section() {
  printf '\n===== %s =====\n' "$1"
}

show_path() {
  local label="$1"
  local path="$2"
  if [[ -e "$path" ]]; then
    printf '%s: EXISTS: %s\n' "$label" "$path"
  else
    printf '%s: MISSING: %s\n' "$label" "$path"
  fi
}

section "Report identity"
printf 'timestamp: %s\n' "$(date --iso-8601=seconds)"
printf 'hostname: %s\n' "$(hostname)"
printf 'user: %s\n' "$(id -un)"
printf 'uid: %s\n' "$(id -u)"
printf 'workspace: %s\n' "$ROOT"
printf 'report: %s\n' "$REPORT"
printf 'bash: %s\n' "$BASH_VERSION"
printf 'kernel: %s\n' "$(uname -srmo)"
printf 'architecture: %s\n' "$(uname -m)"

section "Operating system"
if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  printf 'PRETTY_NAME: %s\n' "${PRETTY_NAME:-unknown}"
  printf 'VERSION_ID: %s\n' "${VERSION_ID:-unknown}"
  printf 'ID: %s\n' "${ID:-unknown}"
else
  echo "/etc/os-release: MISSING"
fi

section "CPU, memory and storage"
if command -v lscpu >/dev/null 2>&1; then
  lscpu | grep -E '^(Architecture|CPU\(s\)|Model name|Thread\(s\) per core|Core\(s\) per socket|Socket\(s\)):' || true
fi
free -h 2>/dev/null || true
df -h "$HOME" 2>/dev/null || true
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>&1 || true
else
  echo "nvidia-smi: NOT_INSTALLED_OR_NOT_IN_PATH"
fi

section "Toolchain"
for bin in bash git python3 gcc g++ cmake make colcon ros2 rviz2; do
  if command -v "$bin" >/dev/null 2>&1; then
    printf '%s: %s\n' "$bin" "$(command -v "$bin")"
  else
    printf '%s: NOT_FOUND\n' "$bin"
  fi
done
python3 --version 2>&1 || true
gcc --version 2>/dev/null | head -n 1 || true
g++ --version 2>/dev/null | head -n 1 || true
cmake --version 2>/dev/null | head -n 1 || true
colcon --version 2>&1 | head -n 2 || true

section "ROS environment before sourcing"
for name in ROS_DISTRO ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION AMENT_PREFIX_PATH CMAKE_PREFIX_PATH; do
  printf '%s=%s\n' "$name" "$(printenv "$name" 2>/dev/null || echo UNSET)"
done

section "ROS Humble and workspace overlays"
if [[ -r /opt/ros/humble/setup.bash ]]; then
  if n01_source_setup /opt/ros/humble/setup.bash; then
    echo "ROS_HUMBLE_SETUP: SOURCED"
  else
    echo "ROS_HUMBLE_SETUP: FAILED"
  fi
else
  echo "ROS_HUMBLE_SETUP: MISSING"
fi
if [[ -r "$ROOT/install/setup.bash" ]]; then
  if n01_source_setup "$ROOT/install/setup.bash"; then
    echo "WORKSPACE_SETUP: SOURCED"
  else
    echo "WORKSPACE_SETUP: FAILED"
  fi
else
  echo "WORKSPACE_SETUP: MISSING"
fi
printf 'ROS_DISTRO(after): %s\n' "${ROS_DISTRO:-UNSET}"
printf 'ROS_DOMAIN_ID(after): %s\n' "${ROS_DOMAIN_ID:-UNSET}"
printf 'RMW_IMPLEMENTATION(after): %s\n' "${RMW_IMPLEMENTATION:-DEFAULT}"
printf 'ROS_LOCALHOST_ONLY(after): %s\n' "${ROS_LOCALHOST_ONLY:-UNSET}"
printf 'AMENT_PREFIX_PATH(after): %s\n' "${AMENT_PREFIX_PATH:-UNSET}"

if command -v ros2 >/dev/null 2>&1; then
  for package in hivecore_slam3d livox_ros_driver2 scan_planner plan_env n01_live_navigation n01_static_map fast_lio; do
    if prefix="$(ros2 pkg prefix "$package" 2>/dev/null)"; then
      printf 'ROS_PACKAGE %s: %s\n' "$package" "$prefix"
    else
      printf 'ROS_PACKAGE %s: NOT_FOUND\n' "$package"
    fi
  done
fi

section "N-01 source and data paths"
show_path "workspace package scan_planner" "$ROOT/src/scan_planner/plan_manage/package.xml"
show_path "workspace package plan_env" "$ROOT/src/scan_planner/plan_env/package.xml"
show_path "workspace package live navigation" "$ROOT/src/n01_live_navigation/package.xml"
show_path "Hivecore SLAM3D underlay" "/opt/ros/humble/setup.bash"
show_path "Hive2 Livox overlay" "${LIVOX_SETUP:-/opt/hivecore/hive2-motion/1.0.0/install/local_setup.bash}"
show_path "static PCD" "${N01_STATIC_PCD_PATH:-$HOME/lyy/slam_data/maps/cloud_map_20261027_225549.pcd}"
show_path "historical Bag" "${N01_BAG_PATH:-$HOME/lyy/slam_data/bags/bag_20261027_225100}"
show_path "runtime directory" "$RUNTIME_ROOT"

section "Git state"
if git -C "$ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  printf 'branch: %s\n' "$(git -C "$ROOT" branch --show-current)"
  printf 'commit: %s\n' "$(git -C "$ROOT" rev-parse HEAD)"
  git -C "$ROOT" status --short --branch
else
  echo "WORKSPACE_GIT: NOT_A_GIT_WORKTREE"
fi

section "Safety"
echo "ROS graph was NOT queried."
echo "No ROS node was started."
echo "No goal, service, action, parameter write, or actuator command was sent."
echo "No existing process was stopped or restarted."
echo "Only this report file was created by this script."
echo
echo "N01_ENVIRONMENT_PREFLIGHT_REPORT=$REPORT"
