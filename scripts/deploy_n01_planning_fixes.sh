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
DISTANCE="0.5"
SMOKE_TIMEOUT="15"
READINESS_TIMEOUT="120"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

mkdir -p "$HOME/lyy/n01_runtime/runs/planning_deploy"
STAMP="$(date +%Y%m%d_%H%M%S)"
DEPLOY_LOG="$HOME/lyy/n01_runtime/runs/planning_deploy/n01_planning_deploy_$STAMP.log"
LAUNCH_LOG="$HOME/lyy/n01_runtime/runs/planning_deploy/n01_live_scan_$STAMP.log"
exec > >(tee -a "$DEPLOY_LOG") 2>&1

# A zombie still answers kill -0, but it has already exited. Use the
# kernel process state so a completed launch is not mistaken for a hung one.
n01_pid_is_running() {
  local pid="${1:-}" state
  [[ -n "$pid" ]] || return 1
  state="$(ps -o stat= -p "$pid" 2>/dev/null | awk 'NR == 1 { print $1 }')"
  [[ -n "$state" && "$state" != Z* && "$state" != X* ]]
}

n01_active_pids_matching() {
  local pattern="$1" pid
  local -a candidates=()
  local -a active=()
  mapfile -t candidates < <(pgrep -f "$pattern" || true)
  for pid in "${candidates[@]}"; do
    if n01_pid_is_running "$pid"; then
      active+=("$pid")
    fi
  done
  printf '%s\\n' "${active[@]}"
}

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
  OLD_PGID="$(ps -o pgid= -p "$OLD_PID" 2>/dev/null | tr -d ' ' || true)"
  OLD_SID="$(ps -o sid= -p "$OLD_PID" 2>/dev/null | tr -d ' ' || true)"
  if [[ "$OLD_PGID" == "$OLD_PID" && "$OLD_SID" == "$OLD_PID" ]]; then
    # New deployments are isolated with setsid, so this process group contains
    # only this launch and its children (never the vendor SLAM/Livox stack).
    echo "sending SIGINT to isolated N-01 launch process group $OLD_PGID"
    kill -INT -- "-$OLD_PGID"
  else
    # Older nohup launches may share the operator's session/process group.
    # Signal only the launch parent; never signal an unverified process group.
    echo "legacy launch is not session-isolated; sending SIGINT only to PID $OLD_PID"
    kill -INT "$OLD_PID"
  fi

  for _ in $(seq 1 150); do
    if ! n01_pid_is_running "$OLD_PID"; then break; fi
    sleep 0.2
  done
  if n01_pid_is_running "$OLD_PID"; then
    echo "ERROR: live_scan launch is still running after SIGINT; refusing forced termination."
    ps -o pid,ppid,pgid,sid,stat,etime,wchan:28,args -p "$OLD_PID" || true
    echo "--- direct children ---"
    ps -o pid,ppid,pgid,sid,stat,etime,args --ppid "$OLD_PID" || true
    if command -v pstree >/dev/null 2>&1; then
      echo "--- launch process tree ---"
      pstree -ap "$OLD_PID" || true
    fi
    LATEST_LAUNCH_LOG="$(find "$HOME/lyy/n01_runtime/runs/planning_deploy" \
      -maxdepth 1 -type f -name 'n01_live_scan_*.log' -printf '%T@ %p\\n' 2>/dev/null \
      | sort -nr | head -n 1 | cut -d' ' -f2-)"
    if [[ -n "$LATEST_LAUNCH_LOG" ]]; then
      echo "--- latest live_scan log: $LATEST_LAUNCH_LOG ---"
      tail -n 100 "$LATEST_LAUNCH_LOG" || true
    fi
    exit 5
  fi
  echo "old launch parent has exited (or is a reaped/zombie process); checking owned N-01 nodes."

  for pattern in '[s]can_planner_node' '[n]01_live_scan_bridge' '[n]01_live_static_pcd'; do
    for _ in $(seq 1 50); do
      mapfile -t ACTIVE_NODE_PIDS < <(n01_active_pids_matching "$pattern")
      (( ${#ACTIVE_NODE_PIDS[@]} == 0 )) && break
      sleep 0.2
    done
    mapfile -t ACTIVE_NODE_PIDS < <(n01_active_pids_matching "$pattern")
    if (( ${#ACTIVE_NODE_PIDS[@]} )); then
      echo "ERROR: old N-01 process remains after launch shutdown: $pattern"
      printf '  active PID: %s\\n' "${ACTIVE_NODE_PIDS[@]}"
      ps -o pid,ppid,pgid,sid,stat,etime,args -p "${ACTIVE_NODE_PIDS[@]}" || true
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

if ! command -v setsid >/dev/null 2>&1; then
  echo "ERROR: setsid is required to isolate the N-01 launch process group safely."
  exit 7
fi
# Isolate only N-01 launch and its children in a dedicated session. Future
# graceful restarts can then signal this verified process group without
# touching vendor SLAM, Livox, or the operator's terminal process group.
nohup setsid ros2 launch n01_live_navigation live_scan.launch.py "${LAUNCH_ARGS[@]}" \
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
  --distance "$DISTANCE" --timeout "$SMOKE_TIMEOUT" \
  --readiness-timeout "$READINESS_TIMEOUT" --confirm-planning-only
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
