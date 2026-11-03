#!/usr/bin/env bash
# Run one fail-closed, planning-only SCAN goal against an already-running
# N-01 live_scan stack. Does not build, restart, or control the robot.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
DISTANCE="${1:-0.8}"
TIMEOUT="${2:-15}"
# Match the deployed N-01 ROS environment unless explicitly overridden.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# shellcheck disable=SC1091
source "$ROOT/scripts/lib/ros_env.sh"

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash is missing."
  exit 2
fi
if ! n01_source_setup /opt/ros/humble/setup.bash; then
  echo "ERROR: failed to source ROS Humble."
  exit 2
fi
if [[ ! -r "$ROOT/install/setup.bash" ]]; then
  echo "ERROR: workspace install/setup.bash is missing; build/deploy first."
  exit 2
fi
if ! n01_source_setup "$ROOT/install/setup.bash"; then
  echo "ERROR: failed to source workspace overlay."
  exit 2
fi

mapfile -t launch_pids < <(
  pgrep -f '[r]os2 launch n01_live_navigation live_scan.launch.py' || true
)
if (( ${#launch_pids[@]} != 1 )); then
  echo "ERROR: expected exactly one running N-01 live_scan launch; found ${#launch_pids[@]}."
  printf 'matching_pids=%s\n' "${launch_pids[*]:-none}"
  echo "No goal was published."
  exit 3
fi

echo "N-01 planning-only smoke test"
echo "workspace: $ROOT"
echo "live_scan_pid: ${launch_pids[0]}"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "distance_m: $DISTANCE"
echo "timeout_s: $TIMEOUT"
echo "This sends at most one /move_base_simple/goal after all freshness, health, safety and graph checks pass."
echo "It never publishes velocity, wheel, chassis or actuator commands."
exec ros2 run n01_live_navigation n01_planning_smoke_test \
  --distance "$DISTANCE" --timeout "$TIMEOUT" --confirm-planning-only
