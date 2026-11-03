#!/usr/bin/env bash
# Rebuild only the N-01 Python navigation package and run one guarded,
# planning-only smoke test against the already-running live_scan stack.
# Does NOT restart live_scan, SLAM, Livox, or any hardware driver.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$ROOT/scripts/lib/ros_env.sh"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

OUT_DIR="$HOME/lyy/n01_runtime/runs/planning_deploy"
mkdir -p "$OUT_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$OUT_DIR/n01_smoke_hotfix_$STAMP.log"
exec > >(tee -a "$LOG_FILE") 2>&1

echo "=== N-01 planning smoke-test hotfix ==="
echo "workspace: $ROOT"
echo "log_file: $LOG_FILE"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash is missing; no goal published."
  exit 10
fi

# Remove only this workspace's previous install prefixes. Keep any Hivecore
# vendor underlays inherited by the operator shell.
n01_strip_workspace_prefixes "$ROOT"
n01_source_setup /opt/ros/humble/setup.bash

cd "$ROOT"
if ! command -v colcon >/dev/null 2>&1; then
  echo "ERROR: colcon is unavailable; no goal published."
  exit 11
fi

# Verify the hotfix is present before building. The health snapshot age must
# be initialized before it is referenced in the readiness JSON.
python3 - "$ROOT/src/n01_live_navigation/n01_live_navigation/planning_smoke_test.py" <<'PY'
import pathlib
import sys
source = pathlib.Path(sys.argv[1]).read_text()
if source.index("health_rx_age_at_publish =") > source.index("readiness_snapshot = {"):
    raise SystemExit("ERROR: readiness snapshot age initialization order is not fixed.")
print("HOTFIX_SOURCE_CONTRACT_OK")
PY

mapfile -t LAUNCH_PIDS < <(
  pgrep -f '[r]os2 launch n01_live_navigation live_scan.launch.py' || true
)
if (( ${#LAUNCH_PIDS[@]} != 1 )); then
  echo "ERROR: expected exactly one running live_scan launch; found ${#LAUNCH_PIDS[@]}."
  echo "No launch will be started and no goal will be published."
  printf 'matching_pids=%s\n' "${LAUNCH_PIDS[*]:-none}"
  exit 12
fi
LIVE_SCAN_PID="${LAUNCH_PIDS[0]}"
echo "existing_live_scan_pid=$LIVE_SCAN_PID"

echo
echo "=== Build only n01_live_navigation (no stack restart) ==="
colcon build --symlink-install --packages-select n01_live_navigation \
  --parallel-workers 2 --event-handlers console_direct+

n01_source_setup "$ROOT/install/setup.bash"

mapfile -t LAUNCH_PIDS_AFTER_BUILD < <(
  pgrep -f '[r]os2 launch n01_live_navigation live_scan.launch.py' || true
)
if (( ${#LAUNCH_PIDS_AFTER_BUILD[@]} != 1 )) ||
   [[ "${LAUNCH_PIDS_AFTER_BUILD[0]:-}" != "$LIVE_SCAN_PID" ]]; then
  echo "ERROR: live_scan launch identity changed during build; refusing to publish."
  printf 'before_pid=%s after_pids=%s\n' "$LIVE_SCAN_PID" "${LAUNCH_PIDS_AFTER_BUILD[*]:-none}"
  exit 13
fi

echo
echo "=== Run one guarded planning-only smoke test ==="
echo "distance_m=0.5"
echo "bspline_timeout_s=15"
echo "readiness_timeout_s=120"
echo "This sends at most one /move_base_simple/goal after freshness, health, safety and graph checks."
set +e
ros2 run n01_live_navigation n01_planning_smoke_test \
  --distance 0.5 --timeout 15 --readiness-timeout 120 --confirm-planning-only
SMOKE_RC=$?
set -e

echo
echo "=== Hotfix smoke-test summary ==="
echo "live_scan_pid=$LIVE_SCAN_PID"
echo "smoke_test_exit_code=$SMOKE_RC"
echo "log_file=$LOG_FILE"
echo "live_scan_was_not_restarted=true"
echo "SLAM_was_not_stopped=true"
echo "actuator_command_published=false"
echo "operator_hold_changed=false"
exit "$SMOKE_RC"
