#!/usr/bin/env bash
# Rebuild, restart exactly one N-01 live SCAN stack, and run goal-aware,
# read-only planning acceptance. Lite SLAM and its adapter are preserved.
set -Eeo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
RUNTIME_ROOT="${N01_RUNTIME_ROOT:-$HOME/lyy/n01_runtime}"
LOG_DIR="$RUNTIME_ROOT/logs"
RUN_DIR="$RUNTIME_ROOT/runs/planning_acceptance"
STAMP="$(date +%Y%m%d_%H%M%S)"
PCD="/opt/slam3d_release/resource/mapping_lite.pcd"
SLAM_CONFIG="/opt/slam3d_release/install/hivecore_slam3d_lite_localization/share/hivecore_slam3d_lite_localization/config/localization_lite.yaml"
LOG="$LOG_DIR/n01_planning_acceptance_run_$STAMP.log"
BUILD_LOG="$LOG_DIR/n01_planning_build_$STAMP.log"
LAUNCH_LOG="$LOG_DIR/n01_planning_launch_$STAMP.log"
REPORT="$RUN_DIR/n01_planning_acceptance_$STAMP.json"
PID_FILE="$RUNTIME_ROOT/runs/navigation/live_scan_launch.pid"
LIFECYCLE="$ROOT/scripts/n01_navigation.sh"
LIFECYCLE_ACTIVE=0

cleanup_navigation() {
  local rc=$?
  if (( LIFECYCLE_ACTIVE == 1 )); then
    echo
    echo "===== AUTOMATIC NAVIGATION SHUTDOWN ====="
    if ! bash "$LIFECYCLE" stop; then
      echo "ERROR: automatic navigation shutdown reported a failure."
      rc=13
    fi
  fi
  exit "$rc"
}
trap cleanup_navigation EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

mkdir -p "$LOG_DIR" "$RUN_DIR"
exec > >(tee "$LOG") 2>&1

source /opt/ros/humble/setup.bash
cd "$ROOT"

if [[ ! -f "$ROOT/install/setup.bash" ]]; then
  echo "ERROR: workspace is not built; install/setup.bash is missing."
  exit 2
fi
set +u
source "$ROOT/install/setup.bash"
set -u

echo "===== 1. REPOSITORY CONTRACT ====="
git branch --show-current
git log -1 --oneline
if [[ "$(git branch --show-current)" != "dev/n01-current" ]]; then
  echo "ERROR: this workflow is restricted to dev/n01-current."
  exit 2
fi

echo
echo "===== 2. VERIFY ACTIVE LITE MAP ====="
if [[ ! -r "$PCD" ]] || [[ ! -r "$SLAM_CONFIG" ]] || ! grep -Fq "map_path: $PCD" "$SLAM_CONFIG"; then
  echo "ERROR: active Lite localization config does not confirm $PCD."
  echo "No navigation stack was stopped or started."
  exit 2
fi
echo "PASS: $PCD matches the installed Lite localization config."

echo
echo "===== 3. CLOSE ANY PREVIOUS N01 NAVIGATION SESSION ====="
if ! bash "$LIFECYCLE" stop; then
  echo "ERROR: previous N-01 navigation session could not be closed."
  exit 3
fi

echo
echo "===== 4. VERIFY NO ORPHANED N01 NAVIGATION NODES ====="
NODES="$(ros2 node list 2>&1)"
MANAGED=(
  /n01_static_map
  /n01_live_scan_bridge
  /n01_live_runtime_supervisor
  /n01_live_safety_supervisor
  /n01_live_dynamic_obstacle_tracker
  /n01_hive2_trajectory_monitor
  /n01_live_scan_goal_gate
  /scan_planner_node
  /n01_live_navigation_rviz
)
LEFTOVERS=()
for node in "\${MANAGED[@]}"; do
  if grep -Fxq "$node" <<<"$NODES"; then LEFTOVERS+=("$node"); fi
done
if (("\${#LEFTOVERS[@]}" > 0)); then
  echo "ERROR: managed N-01 graph entries remain after lifecycle stop:"
  printf '  %s\n' "\${LEFTOVERS[@]}"
  exit 3
fi

ADAPTER_COUNT="$(grep -Fxc "/n01_localization_adapter" <<<"$NODES" || true)"
if [[ "${ADAPTER_COUNT:-0}" -gt 1 ]]; then
  echo "ERROR: duplicate localization adapters detected; refusing startup."
  exit 3
elif [[ "${ADAPTER_COUNT:-0}" -eq 1 ]]; then
  START_ADAPTER=false
  echo "Reusing the existing Lite localization adapter."
else
  START_ADAPTER=true
  echo "No adapter found; source the installed Lite runtime before starting its adapter."
  if [[ ! -r /opt/slam3d_release/setup_runtime.sh ]]; then
    echo "ERROR: /opt/slam3d_release/setup_runtime.sh is unavailable."
    exit 3
  fi
  set +u
  source /opt/slam3d_release/setup_runtime.sh
  source "$ROOT/install/setup.bash"
  set -u
  ros2 interface show hivecore_slam3d_lite_localization/msg/LocStatus >/dev/null
  echo "PASS: Lite LocStatus interface is available."
fi

echo
echo "===== 5. BUILD ====="
if ! colcon build --symlink-install \
    --packages-up-to n01_live_navigation scan_planner n01_static_map \
    --event-handlers console_direct+ 2>&1 | tee "$BUILD_LOG"; then
  echo "BUILD FAILED: $BUILD_LOG"
  exit 4
fi
set +u
source "$ROOT/install/setup.bash"
set -u

echo
echo "===== 6. AUTOMATED TESTS ====="
if ! colcon test --packages-select n01_live_navigation 2>&1 | tee -a "$BUILD_LOG"; then
  echo "TEST COMMAND FAILED: $BUILD_LOG"
  exit 4
fi
if ! colcon test-result --verbose 2>&1 | tee -a "$BUILD_LOG"; then
  echo "TEST RESULT FAILED: $BUILD_LOG"
  exit 4
fi

echo
echo "===== 7. START ONE PROTECTED LIVE STACK ====="
LIFECYCLE_ACTIVE=1
if ! bash "$LIFECYCLE" start; then
  echo "ERROR: unified N-01 navigation start failed."
  exit 5
fi
LAUNCH_PID="$(cat "$PID_FILE")"
echo "launch_pid=$LAUNCH_PID"

echo "===== 8. GOAL-AWARE READ-ONLY ACCEPTANCE ====="
echo "RViz is running. During the next 30 seconds, click '2D Goal Pose' once"
echo "and select a short, visibly clear target in the current map."
echo "This acceptance node does not publish goals or actuator commands."
if ! timeout --signal=INT --kill-after=2s 40s ros2 run n01_live_navigation n01_navigation_acceptance >"$REPORT" 2>&1; then
  echo "Acceptance process failed or timed out."
  cat "$REPORT" || true
  exit 6
fi
cat "$REPORT"

echo
echo "===== 9. LAUNCH WARNINGS / ERRORS ====="
grep -Ei "error|fatal|exception|failed|duplicate|refusing|warning" "$LAUNCH_LOG" | tail -80 || true

echo
echo "===== 10. FINAL GRAPH ====="
ros2 node list | sort

echo
echo "===== 11. ACCEPTANCE VERDICT ====="
python3 - "$REPORT" <<'PY'
import json, sys
path = sys.argv[1]
text = open(path, encoding="utf-8").read()
start = text.find("{")
if start < 0:
    print("INCOMPLETE: acceptance JSON was not produced")
    raise SystemExit(7)
data = json.loads(text[start:])
graph = data.get("graph", {})
pipeline = data.get("goal_pipeline", {})
nav = data.get("navigation", {})
print("single_instance_pass =", graph.get("single_instance_pass"))
print("navigation_ready =", nav.get("navigation_ready_topic"))
print("goal_pipeline_state =", pipeline.get("state"))
print("goal_input_count =", pipeline.get("rviz_goal_input_count"))
print("forwarded_goal_count =", pipeline.get("forwarded_goal_count"))
print("bspline_count =", pipeline.get("bspline_count"))
print("candidate_review_ready =", data.get("candidate_review", {}).get("candidate_review_ready"))
if graph.get("single_instance_pass") is not True:
    print("FAIL: duplicate or missing managed navigation nodes")
    raise SystemExit(8)
if nav.get("navigation_ready_topic") is not True:
    print("FAIL: navigation readiness is not true")
    raise SystemExit(9)
if pipeline.get("rviz_goal_input_count", 0) < 1:
    print("INCOMPLETE: no RViz goal was selected during the acceptance window")
    raise SystemExit(10)
if pipeline.get("state") != "NATIVE_SCAN_BSPLINE_RECEIVED":
    print("FAIL: goal-to-planner chain did not produce a B-spline")
    raise SystemExit(11)
if data.get("candidate_review", {}).get("candidate_review_ready") is not True:
    print("PLANNING PASS, CANDIDATE REVIEW PENDING: see full feasibility and safety JSON")
    raise SystemExit(12)
print("PASS: single stack, ready navigation, goal accepted, native SCAN B-spline and candidate review ready.")
PY
VERDICT_RC=$?

echo
echo "===== COMPLETE ====="
echo "Build/test log: $BUILD_LOG"
echo "Launch log:     $LAUNCH_LOG"
echo "Acceptance:     $REPORT"
echo "Run log:        $LOG"
echo "Navigation stack will be stopped automatically when this command exits."
echo "ACTUATOR_OUTPUT=DISABLED"
echo "EXECUTION_AUTHORIZED=false"
echo "SAFETY_AUTHORIZATION=NEVER"
exit "$VERDICT_RC"
