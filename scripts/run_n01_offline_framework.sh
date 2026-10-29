#!/usr/bin/env bash
# N-01 isolated offline integration pipeline. No robot or actuator outputs.
set -Eeuo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
RUNTIME="${N01_RUNTIME_ROOT:-/home/demo/lyy/n01_runtime}"
STAMP="$(date +%Y%m%d_%H%M%S)_$$"
OUT="$RUNTIME/runs/n01_offline_framework/$STAMP"
ARCHIVE="$RUNTIME/runs/n01_offline_framework/n01_offline_framework_$STAMP.tar.gz"

if [[ "$(id -u)" == 0 ]]; then echo "Do not run with sudo/root" >&2; exit 2; fi
mkdir -p "$OUT/logs" "$OUT/ros_home/log" "$OUT/tmp" || exit 2
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export ROS_HOME="$OUT/ros_home" ROS_LOG_DIR="$OUT/ros_home/log" TMPDIR="$OUT/tmp"
export PYTHONUNBUFFERED=1
# Owned subprocesses inherit this unique token, including detached ROS children.
export N01_G1_RUN_TOKEN="g1_scan_n01_offline_$STAMP"
GUARD="$ROOT/scripts/g1_process_guard.py"
GRAPH="$ROOT/scripts/g1_graph_probe.py"
GRAPH_READY=0
PIDS=()
RESULT="INCOMPLETE"
REASON="unexpected_exit"

say() { printf '[N01-OFFLINE] %s\n' "$*" | tee -a "$OUT/runner.log"; }
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
  if [[ -f "$OUT/integration_fixture_report.json" ]]; then
    cp "$OUT/integration_fixture_report.json" "$OUT/logs/integration_report_final.json"
  fi
  printf 'result=%s\nreason=%s\nexit_code=%s\nowned_process_cleanup=%s\npost_cleanup_ros_graph=%s\n' \
    "$RESULT" "$REASON" "$rc" \
    "$([[ $cleanup_ok == 0 ]] && echo CLEAN || echo RESIDUAL)" \
    "$([[ $graph_ok == 1 ]] && echo CLEAN || echo RESIDUAL)" > "$OUT/summary.txt"
  tar -czf "$ARCHIVE" -C "$(dirname "$OUT")" "$(basename "$OUT")" || archived=1
  echo "=============================================="
  echo "N01 offline framework result: $RESULT | $REASON"
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

say "N01 offline framework start; output: $OUT"
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
say "0/5 Loading ROS 2 Humble"
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
if [[ ! -f src/scan_planner/plan_manage/package.xml || ! -f src/n01_offline_integration/package.xml ]]; then
  REASON="SCAN_SOURCES_MISSING"; exit 2
fi

say "Syntax-checking all offline integration Python sources"
if ! python3 -m compileall -q src/n01_offline_integration/n01_offline_integration \
    src/n01_offline_integration/test scripts/n01_bag_inventory.py scripts/n01_bag_evidence.py \
    > "$OUT/logs/python_syntax.log" 2>&1; then
  RESULT="FAIL"; REASON="python_syntax_check_failed"
  cat "$OUT/logs/python_syntax.log"
  exit 3
fi
say "1/5 Running integration, bridge and guard unit tests"
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
if ! python3 -m unittest discover -s src/n01_offline_integration/test -p 'test_*.py' -v \
     > "$OUT/logs/integration_unit_tests.log" 2>&1; then
  RESULT="FAIL"; REASON="offline_integration_unit_tests_failed"
  tail -n 55 "$OUT/logs/integration_unit_tests.log"
  exit 3
fi
say "2/5 Building SCAN, bridge and offline integration"
if ! colcon build --packages-up-to scan_planner n01_scan_dynacbf_bridge n01_offline_integration \
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

say "3/5 Checking isolated ROS graph (without persistent daemon)"
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

# Bag metadata inspection only; NO automatic uncontrolled bag playback.
say "4/5 Inspecting recorded Bag metadata when available"
BAG="${N01_BAG_PATH:-}"
if [[ -z "$BAG" ]]; then
  for root in /home/demo/lyy /home/demo/latest; do
    [[ -d "$root" ]] || continue
    mapfile -t hits < <(timeout 10s find "$root" -maxdepth 6 \
      -type d -name 'bag_20261027_225100' -print 2>/dev/null || true)
    if (( ${#hits[@]} == 1 )) && [[ -f "${hits[0]}/metadata.yaml" ]]; then
      BAG="${hits[0]}"
      break
    fi
  done
fi
if [[ -n "$BAG" ]]; then
  python3 "$ROOT/scripts/n01_bag_inventory.py" --bag "$BAG" \
    --output "$OUT/bag_inventory.json" > "$OUT/logs/bag_inventory.log" 2>&1 || true
  # Bounded sequential read of TF edges/cloud headers without DDS publication.
  timeout 55s python3 "$ROOT/scripts/n01_bag_evidence.py" --bag "$BAG" \
    --output "$OUT/bag_evidence.json" --max-messages 120000 \
    > "$OUT/logs/bag_evidence.log" 2>&1 || true
  if [[ ! -f "$OUT/bag_evidence.json" ]]; then
    printf '{"result":"TIMEOUT_OR_UNAVAILABLE","reason":"Evidence scan did not complete"}\n' \
      > "$OUT/bag_evidence.json"
  fi
else
  printf '{"result":"NOT_SUPPLIED_OR_NOT_FOUND","reason":"Set N01_BAG_PATH; synthetic integration still runs."}\n' \
    > "$OUT/bag_inventory.json"
  printf '{"result":"NOT_SUPPLIED_OR_NOT_FOUND"}\n' > "$OUT/bag_evidence.json"
fi

say "5/5 Running fixture -> timed TF adapter -> real SCAN -> path reference"
export N01_OFFLINE_FIXTURE=1
export N01_OFFLINE_REPORT_PATH="$OUT/integration_fixture_report.json"
setsid ros2 launch n01_offline_integration offline_pipeline.launch.py \
    fixture:=true use_sim_time:=false duration_s:=75.0 \
    > "$OUT/logs/integration_launch.log" 2>&1 &
A=$!
PIDS+=("$A")
sleep 8
if ! kill -0 "$A" 2>/dev/null; then
  RESULT="FAIL"; REASON="integration_launch_exited_early"; exit 6
fi
# ros2 launch may remain alive after an individual child crashes.
# Do not wait for the 75s observer timeout after a known fixture failure.
if grep -Eq '^\[ERROR\] \[(n01_offline_fixture|n01_offline_bridge|scan_planner_node|scan_bspline_bridge|n01_offline_observer)-[0-9]+\]: process has died' \
    "$OUT/logs/integration_launch.log"; then
  RESULT="FAIL"
  REASON="critical_offline_pipeline_process_crashed"
  grep -A 7 -B 3 -E 'Traceback|process has died' "$OUT/logs/integration_launch.log" | tail -n 60 || true
  exit 8
fi
if ! snapshot running; then
  RESULT="FAIL"; REASON="running_graph_probe_failed"; exit 7
fi
if ! python3 - "$OUT/logs/running/graph.json" > "$OUT/logs/running_no_cmd_vel.txt" <<'PY'
import json,sys
g=json.load(open(sys.argv[1],encoding="utf-8"))
pubs=g.get("publishers",{}).get("/cmd_vel",[])
print("real_cmd_vel_publishers:",pubs)
sys.exit(0 if not pubs else 1)
PY
then
  RESULT="ABORT"; REASON="unexpected_cmd_vel_publisher"; exit 7
fi
deadline=$((SECONDS + 125))
while [[ ! -s "$N01_OFFLINE_REPORT_PATH" ]]; do
  if ! kill -0 "$A" 2>/dev/null; then
    RESULT="FAIL"; REASON="integration_launch_exited_before_report"; exit 8
  fi
  state="$(ps -o stat= -p "$A" 2>/dev/null || true)"
  if [[ "$state" == *Z* ]]; then
    RESULT="FAIL"; REASON="integration_launch_zombie_before_report"; exit 8
  fi
  if (( SECONDS >= deadline )); then
    RESULT="FAIL"; REASON="offline_integration_report_timeout"; exit 8
  fi
  if grep -Eq '^\[ERROR\] \[(n01_offline_fixture|n01_offline_bridge|scan_planner_node|scan_bspline_bridge|n01_offline_observer)-[0-9]+\]: process has died' \
      "$OUT/logs/integration_launch.log"; then
    RESULT="FAIL"; REASON="critical_offline_pipeline_process_crashed"; exit 8
  fi
  sleep 2
done
if python3 - "$N01_OFFLINE_REPORT_PATH" > "$OUT/logs/evaluation.txt" <<'PY'
import json,sys
d=json.load(open(sys.argv[1],encoding="utf-8"))
for k in ("result","issues","counts","last_bridge_health"):
    print(k,"=",d.get(k))
health=d.get("last_bridge_health",{})
assert health.get("control_commands_published")==0
assert d.get("cmd_vel_publishers_seen") is False
assert d.get("result")=="OFFLINE_INTEGRATION_OBSERVED"
assert health.get("output_clouds",0)>0
PY
then
  RESULT="OFFLINE_INTEGRATION_SMOKE_PASS"
  REASON="real_SCAN_3D_map_bspline_and_TF_bridge_seen_no_motor_commands"
  say "Fixture pipeline observed; actual Bag status/TF + physical safety NOT TESTED"
else
  RESULT="OFFLINE_INTEGRATION_INCOMPLETE"
  REASON="pipeline_evidence_missing_or_unsafe"
  tail -n 45 "$OUT/logs/evaluation.txt"
  exit 10
fi
