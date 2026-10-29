#!/usr/bin/env bash
# G1 full offline acceptance: real SCAN multi-scenario test. ROS_DOMAIN_ID=88, localhost only.
set -Eeuo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
RUNTIME="${N01_RUNTIME_ROOT:-/home/demo/lyy/n01_runtime}"
STAMP="$(date +%Y%m%d_%H%M%S)_$$"
OUT="$RUNTIME/runs/g1_acceptance/$STAMP"
ARCHIVE="$RUNTIME/runs/g1_acceptance/g1_acceptance_$STAMP.tar.gz"

if [[ "$(id -u)" == 0 ]]; then echo "Do not run with sudo/root" >&2; exit 2; fi
mkdir -p "$OUT/logs" "$OUT/ros_home/log" "$OUT/tmp" || exit 2
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export ROS_HOME="$OUT/ros_home" ROS_LOG_DIR="$OUT/ros_home/log" TMPDIR="$OUT/tmp"
export PYTHONUNBUFFERED=1
# Owned subprocesses inherit this unique token, including detached ROS children.
export N01_G1_RUN_TOKEN="g1_scan_$STAMP"
GUARD="$ROOT/scripts/g1_process_guard.py"
GRAPH="$ROOT/scripts/g1_graph_probe.py"
GRAPH_READY=0
PIDS=()
RESULT="INCOMPLETE"
REASON="unexpected_exit"

say() { printf '[G1] %s\n' "$*" | tee -a "$OUT/runner.log"; }
snapshot() {
  local name="$1"
  mkdir -p "$OUT/logs/$name"
  if (( GRAPH_READY == 0 )); then return 2; fi
  # Pure rclpy probe: no persistent ros2cli daemon.
  timeout 12s python3 "$GRAPH" \
    > "$OUT/logs/$name/graph.json" 2> "$OUT/logs/$name/graph_error.log"
}
stop_children() {
  local pid phase attempt state clean=1
  python3 "$GUARD" list > "$OUT/logs/processes_before_cleanup.json" 2>&1 || true
  for phase in INT TERM KILL; do
    # Only kill processes carrying THIS test's marker; never pkill by name.
    python3 "$GUARD" signal --signal "$phase" \
      > "$OUT/logs/cleanup_signal_${phase}.json" 2>&1 || true
    for attempt in 1 2 3; do
      sleep 1
      if python3 "$GUARD" verify \
        > "$OUT/logs/cleanup_check_${phase}_${attempt}.json" 2>&1; then
        clean=0
        break 2
      fi
    done
  done
  # Reap our direct child processes; do not hang on uninterruptible children.
  for pid in "${PIDS[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid" 2>/dev/null || true
    else
      state="$(ps -o stat= -p "$pid" 2>/dev/null || true)"
      if [[ "$state" == Z* ]]; then wait "$pid" 2>/dev/null || true; fi
    fi
  done
  if ! python3 "$GUARD" verify > "$OUT/logs/processes_after_cleanup.json" 2>&1; then
    return 1
  fi
  return "$clean"
}
finish() {
  local rc="$?" archived=0 cleanup_ok=1 graph_ok=0 attempt
  trap - EXIT ERR INT TERM HUP
  set +e
  if stop_children; then cleanup_ok=0; fi
  if (( GRAPH_READY == 1 )); then
    # DDS may need a few seconds to forget stopped participants.
    for attempt in 1 2 3; do
      if snapshot "post_cleanup_$attempt" && python3 - "$OUT/logs/post_cleanup_$attempt/graph.json" <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as file:
    g = json.load(file)
nodes = g.get("nodes", [])
print("remaining_ros_nodes:", nodes)
sys.exit(0 if not nodes else 1)
PY
      then
        graph_ok=1
        break
      fi
      sleep 2
    done
  else
    graph_ok=1
    echo 'ROS graph not initialized; no test nodes started.' > "$OUT/logs/post_cleanup_skipped.txt"
  fi
  # Also check for children of the graph probe just launched.
  if ! python3 "$GUARD" verify > "$OUT/logs/processes_final_verification.json" 2>&1; then
    cleanup_ok=1
  fi
  if (( cleanup_ok != 0 || graph_ok != 1 )); then
    RESULT="CLEANUP_FAILED"
    REASON="owned_processes_or_ros_nodes_remaining"
    rc=12
  fi
  if [[ -f "$ROS_HOME/g1_smoke_report.json" ]]; then
    cp "$ROS_HOME/g1_smoke_report.json" "$OUT/g1_smoke_report.json"
  fi
  printf 'result=%s\nreason=%s\nexit_code=%s\nowned_process_cleanup=%s\npost_cleanup_ros_graph=%s\n' \
    "$RESULT" "$REASON" "$rc" \
    "$([[ $cleanup_ok == 0 ]] && echo CLEAN || echo RESIDUAL)" \
    "$([[ $graph_ok == 1 ]] && echo CLEAN || echo RESIDUAL)" > "$OUT/summary.txt"
  tar -czf "$ARCHIVE" -C "$(dirname "$OUT")" "$(basename "$OUT")" || archived=1
  echo "=============================================="
  echo "G1 result: $RESULT | $REASON"
  echo "Owned processes: $([[ $cleanup_ok == 0 ]] && echo CLEAN || echo RESIDUAL)"
  echo "ROS graph after test: $([[ $graph_ok == 1 ]] && echo CLEAN || echo RESIDUAL)"
  echo "Logs directory: $OUT"
  if (( archived == 0 )); then
    echo "UPLOAD THIS LOG ARCHIVE: $ARCHIVE"
  else
    echo "Archive creation failed; share directory: $OUT"
    (( rc != 0 )) || rc=2
  fi
  echo "=============================================="
  exit "$rc"
}
on_error() {
  local rc="$1" line="$2" command="$3"
  {
    printf 'ERR rc=%s line=%s\n' "$rc" "$line"
    printf 'command=%s\n' "$command"
  } >> "$OUT/logs/runner_errors.log"
  if [[ "$RESULT" == "INCOMPLETE" ]]; then
    RESULT="FAIL"
    REASON="unexpected_shell_error_at_line_$line"
  fi
}
trap finish EXIT
trap 'on_error "$?" "$LINENO" "$BASH_COMMAND"' ERR
trap 'RESULT="INTERRUPTED"; REASON="keyboard_interrupt"; exit 130' INT
trap 'RESULT="INTERRUPTED"; REASON="terminated"; exit 143' TERM
trap 'RESULT="INTERRUPTED"; REASON="hangup"; exit 129' HUP

say "G1 FULL OFFLINE ACCEPTANCE; output: $OUT"
{
  date --iso-8601=seconds
  git -C "$ROOT" log -1 --oneline
  git -C "$ROOT" status --short
  uname -a
  python3 --version
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
} > "$OUT/environment.txt" 2>&1

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  REASON="ROS2_HUMBLE_MISSING"; exit 2
fi
say "0/5 Loading ROS 2 Humble (capturing setup output)"
# ROS/colcon setup may reference unset variables. Disable nounset ONLY while sourcing.
set +u
# shellcheck disable=SC1091
if source /opt/ros/humble/setup.bash > "$OUT/logs/ros_setup.log" 2>&1; then
  :
else
  setup_rc=$?
  set -u
  RESULT="FAIL"
  REASON="ros_humble_setup_failed_rc_$setup_rc"
  cat "$OUT/logs/ros_setup.log"
  exit 2
fi
set -u
if ! command -v ros2 > "$OUT/logs/ros2_path.txt" 2>&1; then
  RESULT="FAIL"
  REASON="ros2_command_missing_after_setup"
  exit 2
fi
if ! command -v colcon > "$OUT/logs/colcon_path.txt" 2>&1; then
  RESULT="FAIL"
  REASON="colcon_command_missing_after_setup"
  exit 2
fi
cd "$ROOT"
if [[ ! -f src/scan_planner/plan_manage/package.xml ]]; then
  REASON="SCAN_SOURCES_MISSING"; exit 2
fi

say "1/4 Running process guard + G1 unit tests"
if ! python3 -m unittest discover -s scripts -p 'test_g1_process_guard.py' -v > "$OUT/logs/process_guard_unit_tests.log" 2>&1; then
  RESULT="FAIL"; REASON="process_guard_unit_test_failed"
  tail -n 40 "$OUT/logs/process_guard_unit_tests.log"
  exit 3
fi
if ! python3 -m unittest discover -s src/n01_scan_dynacbf_bridge/test -p 'test_*.py' -v \
     > "$OUT/logs/unit_tests.log" 2>&1; then
  RESULT="FAIL"; REASON="unit_tests_failed"
  tail -n 40 "$OUT/logs/unit_tests.log"
  exit 3
fi
say "2/4 Building SCAN and bridge"
if ! colcon build --packages-up-to scan_planner n01_scan_dynacbf_bridge \
     --event-handlers console_direct+ > "$OUT/logs/colcon_build.log" 2>&1; then
  RESULT="FAIL"; REASON="colcon_build_failed"
  tail -n 60 "$OUT/logs/colcon_build.log"
  exit 4
fi
if [[ ! -f install/setup.bash ]]; then RESULT="FAIL"; REASON="overlay_missing"; exit 4; fi
say "Loading built ROS 2 overlay"
set +u
# shellcheck disable=SC1091
if source install/setup.bash > "$OUT/logs/install_overlay_setup.log" 2>&1; then
  :
else
  setup_rc=$?
  set -u
  RESULT="FAIL"
  REASON="install_overlay_setup_failed_rc_$setup_rc"
  cat "$OUT/logs/install_overlay_setup.log"
  exit 4
fi
set -u
GRAPH_READY=1

say "3/4 Checking isolated ROS graph (without persistent daemon)"
if ! snapshot preflight; then
  RESULT="FAIL"; REASON="ros_graph_preflight_failed"; exit 5
fi
if ! python3 - "$OUT/logs/preflight/graph.json" > "$OUT/logs/preflight_check.txt" 2>&1 <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as file:
    graph = json.load(file)
nodes = graph.get("nodes", [])
cmd_publishers = graph.get("publishers", {}).get("/cmd_vel", [])
print("visible_nodes:", nodes)
print("real_cmd_vel_publishers:", cmd_publishers)
sys.exit(0 if not nodes and not cmd_publishers else 1)
PY
then
  RESULT="ABORT"; REASON="existing_ros_nodes_or_cmd_vel_publishers"
  cat "$OUT/logs/preflight_check.txt"
  exit 5
fi

say "4/4 Running real-SCAN isolated scenarios"
mkdir -p "$OUT/cases"
FAILURES=()
for scenario in clear blocked changed no_odom; do
  say "SCENARIO: $scenario"
  if ! snapshot "pre_${scenario}"; then
    RESULT="FAIL"; REASON="graph_probe_failed_before_$scenario"; exit 6
  fi
  if ! python3 - "$OUT/logs/pre_${scenario}/graph.json" > "$OUT/logs/pre_${scenario}_check.txt" 2>&1 <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as f:
    g = json.load(f)
nodes = g.get("nodes", [])
cmd = g.get("publishers", {}).get("/cmd_vel", [])
print("pre_case_nodes=", nodes, "cmd_vel_publishers=", cmd)
sys.exit(0 if not nodes and not cmd else 1)
PY
  then
    RESULT="ABORT"; REASON="pre_existing_nodes_before_$scenario"
    cat "$OUT/logs/pre_${scenario}_check.txt"
    exit 6
  fi

  setsid ros2 launch scan_planner n01_isolated.launch.py \
    > "$OUT/logs/${scenario}_scan.log" 2>&1 &
  A=$!
  PIDS+=("$A")
  sleep 3
  if ! kill -0 "$A" 2>/dev/null; then
    RESULT="FAIL"; REASON="planner_start_failed_$scenario"; exit 7
  fi
  setsid ros2 run n01_scan_dynacbf_bridge scan_g1_acceptance \
    --scenario "$scenario" --report "$OUT/cases/${scenario}.json" \
    > "$OUT/logs/${scenario}_acceptance.log" 2>&1 &
  C=$!
  PIDS+=("$C")
  sleep 2
  if ! kill -0 "$C" 2>/dev/null; then
    RESULT="FAIL"; REASON="acceptance_node_start_failed_$scenario"; exit 7
  fi
  deadline=$((SECONDS + 85))
  while [[ ! -s "$OUT/cases/${scenario}.json" ]]; do
    if ! kill -0 "$A" 2>/dev/null; then
      RESULT="FAIL"; REASON="planner_exited_$scenario"; exit 8
    fi
    if ! kill -0 "$C" 2>/dev/null; then
      RESULT="FAIL"; REASON="acceptance_node_exited_without_report_$scenario"; exit 8
    fi
    stat="$(ps -o stat= -p "$C" 2>/dev/null || true)"
    if [[ "$stat" == *Z* ]]; then
      RESULT="FAIL"; REASON="acceptance_node_zombie_without_report_$scenario"; exit 8
    fi
    if (( SECONDS >= deadline )); then
      RESULT="FAIL"; REASON="acceptance_case_timeout_$scenario"; exit 8
    fi
    sleep 1
  done
  if python3 - "$OUT/cases/${scenario}.json" > "$OUT/logs/${scenario}_evaluation.txt" 2>&1 <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as f:
    d = json.load(f)
print("scenario=", d["scenario"], "result=", d["result"], "issues=", d["issues"])
sys.exit(0 if d.get("result") == "PASS" else 1)
PY
  then
    say "SCENARIO PASS: $scenario"
  else
    FAILURES+=("$scenario")
    say "SCENARIO FAIL: $scenario (see case report)"
  fi

  if ! stop_children; then
    RESULT="CLEANUP_FAILED"; REASON="process_residual_after_$scenario"; exit 12
  fi
  PIDS=()
  quiet=0
  for attempt in 1 2 3 4; do
    sleep 2
    if snapshot "after_${scenario}_$attempt" && python3 - \
      "$OUT/logs/after_${scenario}_$attempt/graph.json" > \
      "$OUT/logs/after_${scenario}_$attempt/check.txt" 2>&1 <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as f:
    g = json.load(f)
nodes = g.get("nodes", [])
print("post_case_nodes:", nodes)
sys.exit(0 if not nodes else 1)
PY
    then
      quiet=1
      break
    fi
  done
  if (( quiet != 1 )); then
    RESULT="CLEANUP_FAILED"; REASON="ros_graph_residual_after_$scenario"; exit 12
  fi
done

python3 - "$OUT/cases" > "$OUT/g1_acceptance_summary.json" <<'PY'
import json
import pathlib
import sys
base = pathlib.Path(sys.argv[1])
scenarios = ("clear", "blocked", "changed", "no_odom")
cases = {name: json.loads((base / (name + ".json")).read_text(encoding="utf-8"))
         for name in scenarios}
passed = all(c["result"] == "PASS" for c in cases.values())
summary = {
    "test_type": "G1_FULL_OFFLINE_ACCEPTANCE",
    "result": "PASS_OFFLINE_ONLY" if passed else "FAIL",
    "scenarios": {k: {
        "result": v["result"],
        "issues": v["issues"],
        "valid_bspline_messages": v["valid_bspline_messages"],
        "max_inflated_map_points": v["max_inflated_map_points"],
    } for k, v in cases.items()},
    "full_physical_robot_acceptance": "NOT_TESTED",
    "real_slam_bag_replay": "NOT_TESTED",
    "robot_chassis_motion": "NOT_TESTED",
}
print(json.dumps(summary, ensure_ascii=False, indent=2))
PY

if (( ${#FAILURES[@]} > 0 )); then
  RESULT="G1_OFFLINE_FAILED"
  REASON="acceptance_scenarios_failed_${FAILURES[*]}"
  say "G1 OFFLINE FAIL: ${FAILURES[*]}; reports collected and cleanup checked"
  exit 10
fi
RESULT="G1_OFFLINE_PASS"
REASON="4_scenarios_passed_and_cleaned_no_physical_acceptance"
say "G1 FULL OFFLINE ACCEPTANCE PASS; physical/Bag tests remain NOT TESTED"
