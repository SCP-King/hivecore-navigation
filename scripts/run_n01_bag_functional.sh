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
  # Validate read-only per-frame numerical provenance after child shutdown.
  if [[ "$RESULT" == "BAG_FUNCTIONAL_PATH_SMOKE_PASS" ]]; then
    if ! PYTHONPATH="$ROOT/src/n01_offline_integration${PYTHONPATH:+:$PYTHONPATH}" \
      python3 -m n01_offline_integration.g3_frame_evidence \
      "$OUT/g3_frame_evidence.jsonl" "$OUT/g3_evidence_summary.json" \
      "$OUT/recorded_bag_report.json" \
      > "$OUT/logs/g3_evidence_check.log" 2>&1; then
      RESULT="INCOMPLETE"; REASON="g3_frame_evidence_validation_failed"; rc=11
    fi
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

# Refuse running a mixed checkout. A failed git pull can leave remote G3
# files untracked and modified alongside an old Git HEAD. That combination
# previously reached the synthetic solver demo and crashed with TypeError.
# This guard does not modify or delete any user files.
if [[ -n "$(git -C "$ROOT" status --porcelain=v1 --untracked-files=all -- scripts src/n01_offline_integration)" ]]; then
  RESULT="FAIL"
  REASON="dirty_or_mixed_offline_sources_refuse_run"
  git -C "$ROOT" status --short -- scripts src/n01_offline_integration |
    tee "$OUT/logs/dirty_sources.txt"
  say "Source checkout contains local tracked/untracked changes. STOP: do not reset or overwrite user files."
  say "Use a clean sibling clone for this isolated G3 test."
  exit 2
fi
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
# Positive evidence that the G3 optimizer actually changes a trajectory
# under a KNOWN synthetic moving obstacle. Real Bag remains a separate test.
say "1.5/5 Running G3 positive synthetic moving-obstacle optimization"
if ! PYTHONPATH="$ROOT/src/n01_offline_integration${PYTHONPATH:+:$PYTHONPATH}" \
     python3 scripts/g3_offline_solver_demo.py \
     > "$OUT/logs/g3_synthetic_solver_evidence.json" 2> "$OUT/logs/g3_synthetic_solver_stderr.log"; then
  RESULT="FAIL"; REASON="g3_synthetic_constrained_solver_failed"
  cat "$OUT/logs/g3_synthetic_solver_stderr.log"
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
from n01_offline_integration.g3_solver_node import OfflineG3Solver
print("ROS Bag, dynamic tracking and risk modules import: PASS")
PY
then
  RESULT="FAIL"; REASON="recorded_bag_module_import_failed"
  cat "$OUT/logs/bag_node_import.log"
  exit 5
fi

# G4 opt-in; unchanged G3 default. Provenance is checked before launching ROS.
if [[ -n "${N01_G4_REFERENCE_FILE:-}" ]]; then
  say "G4: validating research-only reference provenance, no motion authority"
  if ! PYTHONPATH="$ROOT/src/n01_offline_integration${PYTHONPATH:+:$PYTHONPATH}" \
      python3 - "$N01_G4_REFERENCE_FILE" > "$OUT/logs/g4_preflight.txt" 2>&1 <<'PY'
import json,sys
from pathlib import Path
from n01_offline_integration.g4_chain_evidence import ChainEvidence
source=Path(sys.argv[1]).expanduser().resolve(strict=True)
ChainEvidence(json.loads(source.read_text(encoding="utf-8")))
print("G4 reference: RESEARCH_ONLY",source)
PY
  then
    RESULT="FAIL"; REASON="g4_reference_preflight_failed"; exit 5
  fi
  export N01_G4_RESEARCH_ENABLE=1
  export N01_G4_CHAIN_REPORT_PATH="$OUT/g4_chain_evidence.json"
  say "5/5 Launching isolated Bag + SCAN mode3 + G4 reference + G4 chain observer"
  setsid ros2 launch n01_offline_integration g4_offline_scan_reference.launch.py \
      bag_path:="$BAG" reference_file:="$N01_G4_REFERENCE_FILE" \
      report_path:="$N01_G4_CHAIN_REPORT_PATH" \
      duration_s:=165.0 max_clouds:=2000 scan_rate_hz:=8.0 \
      > "$OUT/logs/integration_launch.log" 2>&1 &
else
  say "5/5 Launching real Bag -> reconstructed map cloud -> real SCAN"
  setsid ros2 launch n01_offline_integration bag_pipeline.launch.py \
      bag_path:="$BAG" duration_s:=165.0 max_clouds:=2000 scan_rate_hz:=8.0 \
      > "$OUT/logs/integration_launch.log" 2>&1 &
fi
A=$!
PIDS+=("$A")
sleep 8
if ! kill -0 "$A" 2>/dev/null; then
  RESULT="FAIL"; REASON="recorded_bag_launch_exited_early"; exit 6
fi

critical_child_died() {
  grep -Eq '^\[ERROR\] \[(n01_historical_bag|n01_offline_observer|scan_planner_node|scan_bspline_bridge|n01_dynamic_tracker|n01_dynamic_port|n01_path_risk|n01_g3_optimizer|n01_g4_reference|n01_g4_chain_observer)-[0-9]+\]: process has died' \
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

deadline=$((SECONDS+240))
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
if python3 - "$N01_OFFLINE_REPORT_PATH" "$OUT/bag_inventory.json" > "$OUT/logs/evaluation.txt" <<'PY'
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
# G3 is no longer an early-180-cloud smoke run. All recorded filtered cloud
# messages must be READ and either transformed or explicitly rejected.
inventory=json.load(open(sys.argv[2],encoding="utf-8"))
expected=int(inventory["topics"]["/livox/lidar/geo_filtered"]["count"])
assert expected>0
assert b.get("finished") is True, "Bag reader did not reach EOF"
assert b.get("input_clouds")==expected, (
    "Only %s / %s Bag clouds were attempted" %
    (b.get("input_clouds"),expected))
assert b.get("output_clouds",0)+b.get("rejected_clouds",0)==expected
print("G3 full Bag input accounted:",expected,
      "map cloud accepted:",b.get("output_clouds"),
      "TF or data rejected:",b.get("rejected_clouds"))
# G2.3 counts only genuinely usable path projections. A body outside the
# 1.5m corridor can NEVER make the validation pass. Post-hoc projections are
# allowed as separately labeled OFFLINE comparisons, not contemporaneous plans.
counts=d.get("counts",{})
risk=d.get("last_path_risk",{})
assert counts.get("path_risk_aligned_frames",0)>0, "No exact-time risk inputs"
live=int(risk.get("live_projected_reference_frames",0))
posthoc=int(risk.get("posthoc_projected_reference_frames",0))
print("G2.3 projected frames LIVE:",live,"POST_HOC:",posthoc)
print("G2.3 actual trajectory-obstacle pairs LIVE:",
      risk.get("aligned_frames_with_proximity_pairs",0),
      "POST_HOC:",risk.get("posthoc_proximity_pairs",0))
assert live+posthoc>0, "Path received but not usable for ANY projected recorded pose"
assert (counts.get("path_risk_posthoc_projected_frames",0)+
        counts.get("path_risk_retained_frames",0))>0
assert risk.get("path_header_is_historical_stamp") is False
assert d.get("last_path_risk",{}).get("safety_authorization")=="NEVER"
assert d.get("dynamic_port",{}).get("actuator_output")=="DISABLED"
# G3 REQUIRES actual recorded Bag frames entering the numerical optimizer.
# Synthetic positive tests are independently reported and cannot substitute
# for the real input-flow evidence here.
g3=d.get("g3_offline_solver",{})
assert counts.get("g3_exact_stamp_pairs",0)>0, "G3 no paired Bag inputs"
assert counts.get("g3_solver_attempts",0)>0, "G3 Bag data never reached optimizer"
assert counts.get("g3_candidate_paths",0)>0, "G3 emitted no candidate path"
assert g3.get("safety_authorization")=="NEVER"
assert g3.get("actuator_output")=="DISABLED"
print("G3 attempted solve on real Bag frames:",counts["g3_solver_attempts"])
print("G3 dynamic-constraint candidate frames:",
      counts.get("g3_dynamic_cbf_candidate_frames",0),
      "nominal-only frames:",counts.get("g3_nominal_only_frames",0))
print("G3 path observations: OFFLINE ONLY; no control authority")
PY
then
  RESULT="BAG_FUNCTIONAL_PATH_SMOKE_PASS"
  REASON="real_recorded_bag_to_SCAN_map_and_bspline_reference_seen"
  if grep -q 'G2.3 projected frames LIVE: 0 POST_HOC: [1-9]' "$OUT/logs/evaluation.txt"; then
    REASON="real_bag_SCAN_pass_G23_POST_HOC_projection_only_NOT_live_risk"
    say "G2.3 has only POST-HOC geometry comparison; real-time risk NOT VERIFIED"
  fi
  say "Recorded Bag passed OPEN-LOOP functional replay; localization tracking, G1 collision and real control NOT TESTED"
else
  RESULT="BAG_FUNCTIONAL_INCOMPLETE"
  REASON="check_bag_health_and_SCAN_trajectory_in_logs"
  cat "$OUT/logs/evaluation.txt"
  exit 10
fi

# G4 passes only if a matching reference arrived and a NEW SCAN spline plus
# downstream G2/G3 topic messages followed. It is not motion certification.
if [[ -n "${N01_G4_REFERENCE_FILE:-}" ]]; then
  say "G4: collecting independent ordered SCAN/G2/G3 evidence"
  for attempt in $(seq 1 20); do
    [[ -s "$N01_G4_CHAIN_REPORT_PATH" ]] && break
    sleep 1
  done
  if [[ ! -s "$N01_G4_CHAIN_REPORT_PATH" ]]; then
    RESULT="G4_CHAIN_INCOMPLETE"; REASON="g4_chain_report_missing_or_observer_crashed"
    exit 10
  fi
  if ! python3 - "$N01_G4_CHAIN_REPORT_PATH" > "$OUT/logs/g4_evaluation.txt" 2>&1 <<'PY'
import json,sys
d=json.load(open(sys.argv[1],encoding="utf-8"))
print("G4 ordered evidence:",d.get("result"))
print("Observed stages:",d.get("counts"))
print("G4 unresolved issues:",d.get("issues"))
assert d.get("schema")=="n01.offline.g4_chain_evidence.v1"
assert d.get("actuator_output")=="DISABLED"
assert d.get("safety_authorization")=="NEVER"
assert d.get("result")=="G4_TO_G3_TOPIC_FLOW_OBSERVED_NOT_SAFETY_CERTIFIED"
assert not d.get("issues")
assert d.get("terrain_and_collision_verified") is False
assert d.get("g3_output_proven_to_follow_the_new_g4_route") is False
PY
  then
    RESULT="G4_CHAIN_INCOMPLETE"; REASON="g4_path_scan_g2_g3_chain_not_observed"
    cat "$OUT/logs/g4_evaluation.txt"
    exit 10
  fi
  REASON="g4_initial_path_native_scan_g2_g3_ordered_topics_seen_NOT_safety"
  say "G4 function topic flow observed; NOT trajectory-following, clearance or real navigation"
fi
