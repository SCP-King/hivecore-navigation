#!/usr/bin/env bash
# Rebuild N-01 planning code, restart only the N-01 live_scan launch, then
# generate one guarded planning-only candidate. SLAM and hardware drivers remain up.
# ROS Humble setup scripts reference optional environment variables. Keep errexit
# and pipefail, but do not enable nounset while sourcing ROS/colcon overlays.
set -Eeo pipefail

if [[ "${1:-}" != "--confirm-planning-only" ]]; then
  echo "Usage: bash scripts/deploy_n01_planning_fixes.sh --confirm-planning-only"
  echo "This rebuilds/restarts only live_scan and sends one planner-only test goal."
  exit 2
fi

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$WS_ROOT/scripts/lib/ros_env.sh"
source "$WS_ROOT/scripts/lib/colcon_build_guard.sh"
PCD_PATH="$HOME/lyy/slam_data/maps/cloud_map_20261027_225549.pcd"
PCD_ASSERTED="true"
GROUND_Z="-0.55"
BODY_HEIGHT="0.90"
RVIZ="true"
RVIZ_CONFIG=""
DISTANCE="0.8"
SMOKE_TIMEOUT="20"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

mkdir -p "$HOME/lyy/n01_runtime/runs/planning_deploy"
STAMP="$(date +%Y%m%d_%H%M%S)"
DEPLOY_LOG="$HOME/lyy/n01_runtime/runs/planning_deploy/n01_planning_deploy_$STAMP.log"
LAUNCH_LOG="$HOME/lyy/n01_runtime/runs/planning_deploy/n01_live_scan_$STAMP.log"
exec > >(tee -a "$DEPLOY_LOG") 2>&1

echo "=== N-01 deployment preflight ==="
echo "workspace: $WS_ROOT"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "deployment_log: $DEPLOY_LOG"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash is missing."
  exit 10
fi
# Remove stale prefixes from this workspace only; preserve vendor underlays
# already loaded by the operator (including Hivecore/Livox message libraries).
n01_strip_workspace_prefixes "$WS_ROOT"
# Intentionally source with nounset disabled (the script does not enable it):
# ROS Humble setup uses optional variables such as AMENT_TRACE_SETUP_FILES.
n01_source_setup /opt/ros/humble/setup.bash
# Do NOT source this workspace's own install/setup.bash before building it.
# Doing so makes colcon treat the old install/ packages as underlays of the
# same source packages, causing self-override warnings and stale prefix order.
# All packages required by --packages-up-to are discovered and built from src.
echo "build environment: ROS Humble underlay only (workspace overlay deferred until after build)"
cd "$WS_ROOT"

if ! command -v colcon >/dev/null 2>&1; then
  echo "ERROR: colcon is unavailable after sourcing ROS Humble."
  exit 11
fi
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 is unavailable after sourcing ROS Humble."
  exit 11
fi

echo "=== N-01 planning fix deployment ==="
echo "workspace: $WS_ROOT"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "build/test: scan_planner + n01_live_navigation"
echo "actuator commands: NEVER"
echo

mapfile -t LIVE_LAUNCH_PIDS < <(pgrep -f '[r]os2 launch n01_live_navigation live_scan.launch.py' || true)
if (( ${#LIVE_LAUNCH_PIDS[@]} > 1 )); then
  echo "ERROR: multiple live_scan launch parents found; refusing ambiguous restart:"
  printf '  %s\n' "${LIVE_LAUNCH_PIDS[@]}"
  exit 3
fi

if (( ${#LIVE_LAUNCH_PIDS[@]} == 1 )); then
  OLD_PID="${LIVE_LAUNCH_PIDS[0]}"
  OLD_ARGS="$(ps -p "$OLD_PID" -o args= || true)"
  echo "existing live_scan launch PID=$OLD_PID"
  echo "$OLD_ARGS"
  for arg in $OLD_ARGS; do
    case "$arg" in
      pcd_path:=*) PCD_PATH="${arg#pcd_path:=}" ;;
      pcd_map_frame_asserted:=*) PCD_ASSERTED="${arg#pcd_map_frame_asserted:=}" ;;
      ground_z_map_m:=*) GROUND_Z="${arg#ground_z_map_m:=}" ;;
      body_height_m:=*) BODY_HEIGHT="${arg#body_height_m:=}" ;;
      rviz:=*) RVIZ="${arg#rviz:=}" ;;
      rviz_config:=*) RVIZ_CONFIG="${arg#rviz_config:=}" ;;
    esac
  done
else
  if pgrep -f '[s]can_planner_node|[n]01_live_scan_bridge|[n]01_live_static_pcd' >/dev/null; then
    echo "ERROR: N-01 live nodes exist without one identifiable live_scan launch parent."
    echo "Refusing to start a duplicate navigation stack."
    exit 3
  fi
fi

if [[ ! -f "$PCD_PATH" ]]; then
  echo "ERROR: configured static PCD does not exist: $PCD_PATH"
  exit 4
fi
if [[ "$PCD_ASSERTED" != "true" ]]; then
  echo "ERROR: existing launch did not assert the current PCD map-frame contract."
  echo "No map-frame assertion will be silently changed."
  exit 4
fi

echo
echo "=== 0/4 Repair only a verified stale generated Python-link collision ==="
n01_repair_generated_python_link_collision "$WS_ROOT" "$HOME/lyy/n01_runtime"

echo
echo "=== 1/4 Build changed ROS packages (without self-sourcing install overlay) ==="
colcon build --symlink-install --packages-up-to scan_planner n01_live_navigation \
  --parallel-workers 2 --event-handlers console_direct+

# The source overlay is loaded only after colcon has completed the build.
n01_source_setup "$WS_ROOT/install/setup.bash"

echo
echo "=== 2/4 Run static inflation equivalence test ==="
colcon test --packages-select plan_env --ctest-args -R test_static_inflation --output-on-failure
colcon test-result --test-result-base "$WS_ROOT/build/plan_env" --verbose

echo
echo "=== 3/4 Restart only the N-01 live_scan launch ==="
if (( ${#LIVE_LAUNCH_PIDS[@]} == 1 )); then
  kill -INT "$OLD_PID"
  for _ in $(seq 1 150); do
    if ! kill -0 "$OLD_PID" 2>/dev/null; then break; fi
    sleep 0.2
  done
  if kill -0 "$OLD_PID" 2>/dev/null; then
    echo "ERROR: live_scan launch did not exit after SIGINT; refusing forced termination."
    exit 5
  fi
  for pattern in '[s]can_planner_node' '[n]01_live_scan_bridge' '[n]01_live_static_pcd'; do
    for _ in $(seq 1 50); do
      pgrep -f "$pattern" >/dev/null || break
      sleep 0.2
    done
    if pgrep -f "$pattern" >/dev/null; then
      echo "ERROR: old N-01 process remains after launch shutdown: $pattern"
      exit 5
    fi
  done
fi

LAUNCH_ARGS=(
  pcd_path:="$PCD_PATH"
  pcd_map_frame_asserted:="$PCD_ASSERTED"
  ground_z_map_m:="$GROUND_Z"
  body_height_m:="$BODY_HEIGHT"
  rviz:="$RVIZ"
)
if [[ -n "$RVIZ_CONFIG" ]]; then
  LAUNCH_ARGS+=(rviz_config:="$RVIZ_CONFIG")
fi

nohup ros2 launch n01_live_navigation live_scan.launch.py "${LAUNCH_ARGS[@]}" \
  > "$LAUNCH_LOG" 2>&1 < /dev/null &
NEW_LAUNCH_PID=$!
echo "$NEW_LAUNCH_PID" > "$HOME/lyy/n01_runtime/runs/planning_deploy/n01_live_scan.pid"
echo "new live_scan PID=$NEW_LAUNCH_PID"
echo "live_scan log=$LAUNCH_LOG"

sleep 2
if ! kill -0 "$NEW_LAUNCH_PID" 2>/dev/null; then
  echo "ERROR: live_scan launch exited during startup."
  tail -n 120 "$LAUNCH_LOG"
  exit 6
fi

echo
echo "=== 4/4 One planning-only B-spline smoke test ==="
set +e
ros2 run n01_live_navigation n01_planning_smoke_test \
  --distance "$DISTANCE" --timeout "$SMOKE_TIMEOUT" --confirm-planning-only
SMOKE_RC=$?
set -e

echo
echo "=== Deployment summary ==="
echo "live_scan_pid=$NEW_LAUNCH_PID"
echo "live_scan_log=$LAUNCH_LOG"
echo "deployment_log=$DEPLOY_LOG"
echo "smoke_test_exit_code=$SMOKE_RC"
echo "SLAM process was not stopped."
echo "Operator hold was not changed."
echo "No velocity, wheel, chassis, or actuator command was published."
exit "$SMOKE_RC"
