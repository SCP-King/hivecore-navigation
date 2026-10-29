#!/usr/bin/env bash
# N-01 real recorded Bag read-only open-loop functional replay, no actuators.
set -Eeuo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
RUNTIME="${N01_RUNTIME_ROOT:-/home/demo/lyy/n01_runtime}"
STAMP="$(date +%Y%m%d_%H%M%S)_$$"
OUT="$RUNTIME/runs/n01_bag_functional/$STAMP"
ARCHIVE="$RUNTIME/runs/n01_bag_functional/n01_bag_functional_$STAMP.tar.gz"

if [[ "$(id -u)" == 0 ]]; then echo "Do not run with sudo/root" >&2; exit 2; fi
mkdir -p "$OUT/logs" "$OUT/ros_home/log" "$OUT/tmp" || exit 2
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export ROS_HOME="$OUT/ros_home" ROS_LOG_DIR="$OUT/ros_home/log" TMPDIR="$OUT/tmp"
export PYTHONUNBUFFERED=1
# Owned subprocesses inherit this unique token, including detached ROS children.
export N01_G1_RUN_TOKEN="g1_scan_n01_bag_$STAMP"
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
  if [[ -f "$OUT/recorded_bag_report.json" ]]; then
    cp "$OUT/recorded_bag_report.json" "$OUT/logs/bag_report_final.json"
  fi
  printf 'result=%s\nreason=%s\nexit_code=%s\nowned_process_cleanup=%s\npost_cleanup_ros_graph=%s\n' \
    "$RESULT" "$REASON" "$rc" \
    "$([[ $cleanup_ok == 0 ]] && echo CLEAN || echo RESIDUAL)" \
    "$([[ $graph_ok == 1 ]] && echo CLEAN || echo RESIDUAL)" > "$OUT/summary.txt"
  tar -czf "$ARCHIVE" -C "$(dirname "$OUT")" "$(basename "$OUT")" || archived=1
  echo "=============================================="
  echo "N01 recorded Bag result: $RESULT | $REASON"
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

say "N01 recorded Bag functional replay start; output: $OUT"
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
# Catch Python/ROS message import and file-path name collisions before launch.
# Import only: no rclpy.init(), no DDS node, no actuators.
if ! python3 - > "$OUT/logs/observer_import_check.log" 2>&1 <<'PY'
from pathlib import Path
from n01_offline_integration import observer_node
assert observer_node.FilePath is Path, "observer output path must use pathlib.Path"
assert observer_node.RosPath.__module__.startswith("nav_msgs.msg"), "ROS Path message alias invalid"
assert observer_node.FilePath("/tmp/observer-import-only.json").suffix == ".json"
print("observer Path aliases and ROS message imports: PASS")
PY
then
  RESULT="FAIL"; REASON="observer_import_check_failed"
  tail -n 40 "$OUT/logs/observer_import_check.log"
  exit 4
fi
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

# The recorded bag is explicitly selected, never inferred from other recordings.
say "4/5 Confirming real recorded Bag source (no raw ros2 bag play)"
BAG="${N01_BAG_PATH:-/home/demo/lyy/slam_data/bags/bag_20261027_225100}"
if [[ ! -f "$BAG/metadata.yaml" ]]; then
  RESULT="FAIL"; REASON="recorded_bag_metadata_missing"
  echo "Set N01_BAG_PATH to an actual rosbag2 directory; received: $BAG"
  exit 5
fi
if ! python3 "$ROOT/scripts/n01_bag_inventory.py" --bag "$BAG" \
    --output "$OUT/bag_inventory.json" > "$OUT/logs/bag_inventory.log" 2>&1; then
  RESULT="FAIL"; REASON="bag_inventory_failed"; exit 5
fi
# Do NOT require /localization_status: this is a historical open-loop mode.
if ! python3 - "$OUT/bag_inventory.json" > "$OUT/logs/bag_preflight.txt" <<'PY'
import json,sys
d=json.load(open(sys.argv[1],encoding="utf-8"))
ts=d.get("topics",{})
required={
    "/livox/lidar/geo_filtered":"sensor_msgs/msg/PointCloud2",
    "/tf":"tf2_msgs/msg/TFMessage",
}
for topic,kind in required.items():
    assert ts.get(topic,{}).get("type")==kind, "Missing/incompatible "+topic
    assert ts[topic].get("count",0)>0, "Empty "+topic
print("Bag functional input topics: OK")
print("LocalizationStatus availability:", ts.get("/localization_status"))
print("No raw TF or control topics will be republished")
PY
then
  RESULT="FAIL"; REASON="bag_missing_functional_input_topics"; exit 5
fi
say "Historical replay: old TF mount values ignored; 0.90m used for offline conversion"
export N01_HISTORICAL_BAG_ONLY=1
export N01_OFFLINE_REPORT_PATH="$OUT/recorded_bag_report.json"
if ! python3 - > "$OUT/logs/bag_node_import.log" 2>&1 <<'PY'
from n01_offline_integration.historical_bag_node import RecordedBagReplay
from n01_offline_integration.historical_tf import HistoricalMapTf
from n01_offline_integration.dynamic_tracker_node import OfflineDynamicTracker
from n01_offline_integration.path_risk_node import OfflinePathRisk
from n01_offline_integration.dynamic_port_node import DynamicPort
print("ROS Bag, dynamic tracking and risk modules import: PASS")
PY
then
  RESULT="FAIL"; REASON="recorded_bag_module_import_failed"
  cat "$OUT/logs/bag_node_import.log"
  exit 5
fi

say "5/5 Launching real Bag -> reconstructed map cloud -> real SCAN"
setsid ros2 launch n01_offline_integration bag_pipeline.launch.py \
    bag_path:="$BAG" duration_s:=95.0 \
    > "$OUT/logs/integration_launch.log" 2>&1 &
A=$!
PIDS+=("$A")
sleep 8
if ! kill -0 "$A" 2>/dev/null; then
  RESULT="FAIL"; REASON="recorded_bag_launch_exited_early"; exit 6
fi

critical_child_died() {
  grep -Eq '^\[ERROR\] \[(n01_historical_bag|n01_offline_observer|scan_planner_node|scan_bspline_bridge|n01_dynamic_tracker|n01_dynamic_port|n01_path_risk)-[0-9]+\]: process has died' \
    "$OUT/logs/integration_launch.log"
}
if critical_child_died; then
  RESULT="FAIL"; REASON="critical_bag_replay_child_crashed"
  grep -A 7 -B 3 -E 'Traceback|process has died' "$OUT/logs/integration_launch.log" | tail -n 70 || true
  exit 6
fi
if ! snapshot running; then
  RESULT="FAIL"; REASON="running_graph_probe_failed"; exit 7
fi
if ! python3 - "$OUT/logs/running/graph.json" > "$OUT/logs/running_no_cmd_vel.txt" <<'PY'
import json,sys
g=json.load(open(sys.argv[1],encoding="utf-8"))
pubs=g.get("publishers",{}).get("/cmd_vel",[])
print("Real control publishers:",pubs)
sys.exit(0 if not pubs else 1)
PY
then
  RESULT="ABORT"; REASON="unexpected_cmd_vel_publisher"; exit 7
fi

deadline=$((SECONDS+140))
while [[ ! -s "$N01_OFFLINE_REPORT_PATH" ]]; do
  if ! kill -0 "$A" 2>/dev/null; then
    RESULT="FAIL"; REASON="bag_launch_exited_before_report"; exit 8
  fi
  state="$(ps -o stat= -p "$A" 2>/dev/null || true)"
  if [[ "$state" == *Z* ]]; then
    RESULT="FAIL"; REASON="bag_launch_zombie_before_report"; exit 8
  fi
  if critical_child_died; then
    RESULT="FAIL"; REASON="critical_bag_replay_child_crashed"; exit 8
  fi
  if (( SECONDS>=deadline )); then
    RESULT="FAIL"; REASON="bag_integration_report_timeout"; exit 8
  fi
  sleep 2
done
if python3 - "$N01_OFFLINE_REPORT_PATH" > "$OUT/logs/evaluation.txt" <<'PY'
import json,sys
d=json.load(open(sys.argv[1],encoding="utf-8"))
print("result:",d.get("result"))
print("issues:",d.get("issues"))
print("counts:",d.get("counts"))
print("bag_health:",d.get("historical_bag_health"))
assert d.get("cmd_vel_publishers_seen") is False
assert d.get("historical_bag_mode") is True
assert d.get("result")=="BAG_FUNCTIONAL_PATH_OBSERVED"
b=d.get("historical_bag_health",{})
assert b.get("raw_tf_republished") is False
assert b.get("cmd_vel_created") is False
assert b.get("output_clouds",0)>0
# G2.3 must receive exact recorded-stamp body+track pairs AND a retained
# SCAN Path geometry projection at least once, not just idle heartbeats.
counts=d.get("counts",{})
assert counts.get("path_risk_aligned_frames",0)>0, "No exact-time risk inputs"
assert counts.get("path_risk_retained_frames",0)>0, "No matched retained Path evidence"
assert d.get("last_path_risk",{}).get("safety_authorization")=="NEVER"
assert d.get("dynamic_port",{}).get("actuator_output")=="DISABLED"
PY
then
  RESULT="BAG_FUNCTIONAL_PATH_SMOKE_PASS"
  REASON="real_recorded_bag_to_SCAN_map_and_bspline_reference_seen"
  say "Recorded Bag passed OPEN-LOOP functional replay; localization tracking, G1 collision and real control NOT TESTED"
else
  RESULT="BAG_FUNCTIONAL_INCOMPLETE"
  REASON="check_bag_health_and_SCAN_trajectory_in_logs"
  cat "$OUT/logs/evaluation.txt"
  exit 10
fi
