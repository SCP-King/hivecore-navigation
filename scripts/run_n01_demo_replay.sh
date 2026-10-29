#!/usr/bin/env bash
# REAL Bag -> G1/G2/G3 -> read-only RViz demo -> RViz-only MP4/GIF.
# Requires a local X11-compatible GUI. Never starts chassis, SLAM control or /cmd_vel.
set -Eeuo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
RUNTIME="${N01_RUNTIME_ROOT:-/home/demo/lyy/n01_runtime}"
STAMP="$(date +%Y%m%d_%H%M%S)_$$"
OUT="$RUNTIME/demo_exports/$STAMP"
BAG="${N01_BAG_PATH:-/home/demo/lyy/slam_data/bags/bag_20261027_225100}"
VIDEO="$OUT/n01_g1_g2_g3_rviz.mp4"
GIF="$OUT/n01_g1_g2_g3_preview.gif"
PNG="$OUT/n01_g1_g2_g3_preview.png"
REPORT="$OUT/recorded_bag_report.json"
GUARD="$ROOT/scripts/g1_process_guard.py"
GRAPH="$ROOT/scripts/g1_graph_probe.py"
RESULT="INCOMPLETE"
REASON="not_completed"
GRAPH_READY=0
LAUNCH_PID=""
RVIZ_PID=""
REC_PID=""
PCD_PID=""
PCD_ENABLED=NO
PIDS=()
if (( EUID==0 )); then echo "Do not run as root/sudo" >&2; exit 2; fi
mkdir -p "$OUT/logs" "$OUT/ros_home/log" "$OUT/tmp"
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 N01_HISTORICAL_BAG_ONLY=1
export N01_OFFLINE_REPORT_PATH="$REPORT"
export ROS_HOME="$OUT/ros_home" ROS_LOG_DIR="$OUT/ros_home/log" TMPDIR="$OUT/tmp"
export N01_G1_RUN_TOKEN="g1_scan_n01_demo_$STAMP"
export PYTHONUNBUFFERED=1
say() { printf '[N01-DEMO] %s\n' "$*" | tee -a "$OUT/demo_runner.log"; }

stop_owned() {
  local phase i
  python3 "$GUARD" list >"$OUT/logs/processes_before_cleanup.json" 2>&1 || true
  # Stop ffmpeg and ROS gracefully so the MP4 can be finalized.
  for phase in INT TERM KILL; do
    python3 "$GUARD" signal --signal "$phase" \
      > "$OUT/logs/signal_$phase.json" 2>&1 || true
    for i in 1 2 3 4 5; do
      if python3 "$GUARD" verify >"$OUT/logs/verify_${phase}_${i}.json" 2>&1; then
        return 0
      fi
      sleep 1
    done
  done
  return 1
}

finish() {
  local rc="$?" cleaned=1 graph_clean=1 vid_ok=1 gif_ok=1 i
  trap - EXIT ERR INT TERM HUP
  set +e
  if stop_owned; then cleaned=0; fi
  for pid in "${PIDS[@]}"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
  if (( GRAPH_READY==1 )); then
    for i in 1 2 3; do
      if timeout 12s python3 "$GRAPH" > "$OUT/logs/post_graph.json" 2>"$OUT/logs/post_graph_error.log" &&
         python3 - "$OUT/logs/post_graph.json" <<'PY'
import json,sys
g=json.load(open(sys.argv[1],encoding="utf-8"))
sys.exit(0 if not g.get("nodes") and not g.get("publishers",{}).get("/cmd_vel") else 1)
PY
      then graph_clean=0; break; fi
      sleep 2
    done
  else
    graph_clean=0
  fi
  if [[ -s "$VIDEO" ]] && ffprobe -v error -select_streams v:0 \
     -show_entries stream=codec_name -of csv=p=0 "$VIDEO" > "$OUT/logs/video_probe.txt" 2>&1; then
    vid_ok=0
    if python3 "$ROOT/scripts/export_n01_demo_gif.py" \
        --video "$VIDEO" --gif "$GIF" --still "$PNG" \
        --start 18 --seconds 12 > "$OUT/logs/gif_export.log" 2>&1; then gif_ok=0; fi
  fi
  # Independent G3.1 frame journal must be present for a new demo PASS.
  if [[ "$RESULT" == "PASS" ]]; then
    if ! PYTHONPATH="$ROOT/src/n01_offline_integration${PYTHONPATH:+:$PYTHONPATH}" \
      python3 -m n01_offline_integration.g3_frame_evidence \
      "$OUT/g3_frame_evidence.jsonl" "$OUT/g3_evidence_summary.json" "$REPORT" \
      > "$OUT/logs/g3_evidence_check.log" 2>&1; then
      RESULT="INCOMPLETE"; REASON="g3_frame_evidence_validation_failed"; rc=11
    fi
  fi
  if [[ "$PCD_ENABLED" == "YES" && "$RESULT" == "PASS" && ! -s "$OUT/pcd_static_preview.json" ]]; then
    RESULT="INCOMPLETE"; REASON="static_pcd_preview_not_published"; rc=11
  fi
  if (( cleaned!=0 || graph_clean!=0 )); then
    RESULT="CLEANUP_FAILED"; REASON="owned_process_or_ros_graph_residual"; rc=12
  elif [[ "$RESULT" == "PASS" ]] && (( vid_ok!=0 || gif_ok!=0 )); then
    RESULT="INCOMPLETE"; REASON="recording_or_preview_export_failed"; rc=10
  fi
  printf 'result=%s\nreason=%s\nexit_code=%s\nprocess_cleanup=%s\nros_graph_cleanup=%s\nvideo_created=%s\ngif_created=%s\nstatic_pcd_requested=%s\nstatic_pcd_report=%s\n' \
    "$RESULT" "$REASON" "$rc" \
    "$([[ $cleaned == 0 ]] && echo CLEAN || echo RESIDUAL)" \
    "$([[ $graph_clean == 0 ]] && echo CLEAN || echo RESIDUAL)" \
    "$([[ $vid_ok == 0 ]] && echo YES || echo NO)" \
    "$([[ $gif_ok == 0 ]] && echo YES || echo NO)" \
    "$PCD_ENABLED" "$([[ -s "$OUT/pcd_static_preview.json" ]] && echo YES || echo NO)" \
    > "$OUT/summary.txt"
  echo "======== N01 G1-G3 DEMO ========"
  cat "$OUT/summary.txt"
  echo "VIDEO: $VIDEO"
  echo "GIF:   $GIF"
  echo "PNG:   $PNG"
  echo "LOGS:  $OUT"
  echo "All visual tracks and candidates are unverified OFFLINE estimates."
  exit "$rc"
}
trap finish EXIT
trap 'RESULT=FAIL; REASON=script_error; exit 1' ERR
trap 'RESULT=INTERRUPTED; REASON=user_interrupt; exit 130' INT
trap 'RESULT=INTERRUPTED; REASON=terminated; exit 143' TERM HUP

say "Preparing isolated G1-G2-G3 demo in $OUT"
if [[ ! -f "$BAG/metadata.yaml" ]]; then
  RESULT="FAIL"; REASON="bag_metadata_missing"; exit 2
fi
if [[ -z "${DISPLAY:-}" ]] || ! command -v xdpyinfo >/dev/null ||
   ! xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
  RESULT="FAIL"; REASON="x11_display_unavailable"; say "Requires a graphical X11 session"; exit 2
fi
for bin in ffmpeg ffprobe xdotool rviz2 colcon; do
  if [[ "$bin" == "colcon" || "$bin" == "rviz2" ]]; then continue; fi
  if ! command -v "$bin" >/dev/null; then
    RESULT="FAIL"; REASON="missing_dependency_$bin"
    say "Missing dependency: $bin. Install ffmpeg, xdotool, x11-utils; no packages auto-installed"; exit 2
  fi
done
if ! ffmpeg -hide_banner -h demuxer=x11grab 2>&1 | grep -q window_id; then
  RESULT="FAIL"; REASON="no_window_only_recording_support"; exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  RESULT="FAIL"; REASON="ros_humble_missing"; exit 2
fi
# ROS setup files may reference unset environment variables.
set +u
source /opt/ros/humble/setup.bash > "$OUT/logs/ros_setup.log" 2>&1
set -u
for bin in ros2 colcon rviz2; do
  if ! command -v "$bin" >/dev/null; then
    RESULT="FAIL"; REASON="missing_ros_tool_$bin"; exit 2
  fi
done
# Worktree check keeps demonstration reproducible, but never deletes files.
if [[ -n "$(git -C "$ROOT" status --porcelain=v1 --untracked-files=all -- scripts src/n01_offline_integration rviz)" ]]; then
  RESULT="FAIL"; REASON="mixed_local_source_tree"
  git -C "$ROOT" status --short -- scripts src/n01_offline_integration rviz
  exit 2
fi
# PCD background is opt-in only and is never forwarded into SCAN.
if [[ -n "${N01_STATIC_PCD_PATH:-}" ]]; then
  if [[ "${N01_PCD_MAP_FRAME_ASSERTED:-}" != "1" ]]; then
    RESULT="ABORT"; REASON="static_pcd_map_frame_not_asserted"
    say "PCD coordinates must first be verified against recorded Bag map frame"
    exit 2
  fi
  if [[ ! -f "$N01_STATIC_PCD_PATH" ]]; then
    RESULT="FAIL"; REASON="static_pcd_file_missing"
    exit 2
  fi
  if ! PYTHONPATH="$ROOT/src/n01_offline_integration${PYTHONPATH:+:$PYTHONPATH}" \
    python3 - "$N01_STATIC_PCD_PATH" > "$OUT/logs/pcd_header.txt" 2>&1 <<'PY'
from n01_offline_integration.pcd_static_preview import inspect_pcd
import sys
meta=inspect_pcd(sys.argv[1])
print("PCD_PREVIEW_HEADER_OK",meta["format"],meta["count"])
PY
  then
    RESULT="FAIL"; REASON="static_pcd_header_invalid_or_unsupported"
    exit 2
  fi
  PCD_ENABLED=YES
  export N01_PCD_PREVIEW_REPORT_PATH="$OUT/pcd_static_preview.json"
  say "Optional PCD BACKGROUND enabled; view-only, frame overlap not independently validated"
else
  say "No N01_STATIC_PCD_PATH: full PCD map NOT loaded; showing historical scan frames only"
fi
say "Preflight: no other nodes or control publishers in isolated ROS domain"
if ! timeout 12s python3 "$GRAPH" > "$OUT/logs/preflight_graph.json" 2>&1; then
  RESULT="FAIL"; REASON="preflight_ros_graph_unavailable"; exit 2
fi
GRAPH_READY=1
if ! python3 - "$OUT/logs/preflight_graph.json" <<'PY'
import json,sys
g=json.load(open(sys.argv[1],encoding="utf-8"))
assert not g.get("nodes"),"Existing ROS nodes in domain 88"
assert not g.get("publishers",{}).get("/cmd_vel"),"Unexpected command publisher"
PY
then RESULT="ABORT"; REASON="nonempty_ros_domain"; exit 3; fi

# Confirm this is a real recorded Bag and save its exact input count.
if ! python3 "$ROOT/scripts/n01_bag_inventory.py" --bag "$BAG"      --output "$OUT/bag_inventory.json" > "$OUT/logs/bag_inventory.log" 2>&1; then
  RESULT="FAIL"; REASON="bag_inventory_failed"; exit 4
fi
say "Building existing G1-G3 packages + new read-only marker display"
cd "$ROOT"
if ! colcon build --packages-up-to n01_offline_integration \
     --event-handlers console_direct+ > "$OUT/logs/colcon_build.log" 2>&1; then
  RESULT="FAIL"; REASON="colcon_build_failed"
  tail -n 45 "$OUT/logs/colcon_build.log"; exit 4
fi
set +u
source "$ROOT/install/setup.bash" > "$OUT/logs/workspace_setup.log" 2>&1
set -u

say "Launching historical Bag replay and read-only RViz overlays"
setsid ros2 launch n01_offline_integration bag_pipeline.launch.py \
    bag_path:="$BAG" duration_s:=165.0 max_clouds:=2000 scan_rate_hz:=8.0 \
    > "$OUT/logs/bag_launch.log" 2>&1 &
LAUNCH_PID="$!"; PIDS+=("$LAUNCH_PID")
setsid ros2 run n01_offline_integration n01_demo_markers \
    > "$OUT/logs/demo_markers.log" 2>&1 &
PIDS+=("$!")
if [[ "$PCD_ENABLED" == "YES" ]]; then
  setsid ros2 run n01_offline_integration n01_static_pcd \
    > "$OUT/logs/static_pcd.log" 2>&1 &
  PCD_PID="$!"; PIDS+=("$PCD_PID")
fi
QT_QPA_PLATFORM=xcb setsid rviz2 -d "$ROOT/rviz/n01_full_pipeline.rviz" \
    > "$OUT/logs/rviz.log" 2>&1 &
RVIZ_PID="$!"; PIDS+=("$RVIZ_PID")
sleep 5
if ! kill -0 "$LAUNCH_PID" || ! kill -0 "$RVIZ_PID"; then
  RESULT="FAIL"; REASON="bag_or_rviz_exited_early"; exit 5
fi
if [[ "$PCD_ENABLED" == "YES" ]] && ! kill -0 "$PCD_PID"; then
  RESULT="FAIL"; REASON="static_pcd_preview_crashed"
  tail -n 30 "$OUT/logs/static_pcd.log"; exit 5
fi
setsid bash "$ROOT/scripts/record_n01_demo.sh" "$VIDEO" "$RVIZ_PID" \
    > "$OUT/logs/recording.log" 2>&1 &
REC_PID="$!"; PIDS+=("$REC_PID")
say "Capturing RViz window only; current DESKTOP outside RViz is never recorded"

deadline=$((SECONDS+245))
while [[ ! -s "$REPORT" ]]; do
  if (( SECONDS>=deadline )); then
    RESULT="FAIL"; REASON="observer_report_timeout"; exit 7
  fi
  if ! kill -0 "$LAUNCH_PID"; then
    RESULT="FAIL"; REASON="bag_pipeline_exited_before_report"; exit 7
  fi
  if [[ "$PCD_ENABLED" == "YES" ]] && ! kill -0 "$PCD_PID"; then
    RESULT="FAIL"; REASON="static_pcd_preview_exited_early"
    tail -n 30 "$OUT/logs/static_pcd.log"; exit 7
  fi
  if ! kill -0 "$RVIZ_PID" || ! kill -0 "$REC_PID"; then
    RESULT="FAIL"; REASON="rviz_or_recording_exited_early"
    tail -n 30 "$OUT/logs/recording.log"; exit 7
  fi
  if grep -Eq '^\[ERROR\] \[(n01_historical_bag|scan_planner_node|n01_g3_optimizer|n01_offline_observer)-[0-9]+\]: process has died' \
     "$OUT/logs/bag_launch.log"; then
    RESULT="FAIL"; REASON="critical_bag_node_died"; exit 7
  fi
  sleep 2
done
say "Checking the actual recorded Bag G1-G2-G3 output"
if ! python3 - "$REPORT" "$OUT/demo_summary.json" "$OUT/bag_inventory.json" <<'PY'
import json,sys
from pathlib import Path
r=json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
inventory=json.loads(Path(sys.argv[3]).read_text(encoding="utf-8"))
expected=int(inventory["topics"]["/livox/lidar/geo_filtered"]["count"])
c=r.get("counts",{})
b=r.get("historical_bag_health",{})
g=r.get("g3_offline_solver",{})
assert r.get("result")=="BAG_FUNCTIONAL_PATH_OBSERVED",r.get("issues")
assert r.get("cmd_vel_publishers_seen") is False
assert b.get("finished") is True
assert expected>0 and expected<=2000
assert int(b.get("input_clouds",0))==expected, "Not a full Bag replay"
assert int(b.get("output_clouds",0))>0
assert int(b.get("output_clouds",0))+int(b.get("rejected_clouds",0))==expected
assert c.get("valid_splines",0)>0
assert c.get("g3_solver_attempts",0)>0
assert g.get("actuator_output")=="DISABLED"
assert g.get("safety_authorization")=="NEVER"
out={
  "result":"REAL_BAG_G1_G2_G3_VISUALIZATION_DATA_PASS",
  "scope":"HISTORICAL_OFFLINE_ONLY_NOT_ROBOT_SAFETY",
  "input_clouds":b.get("input_clouds"),
  "map_clouds":b.get("output_clouds"),
  "tf_or_data_rejected":b.get("rejected_clouds"),
  "scan_valid_bspline":c.get("valid_splines"),
  "g3_solver_frames":c.get("g3_solver_attempts"),
  "g3_dynamic_constraint_candidate_frames":c.get("g3_dynamic_cbf_candidate_frames"),
  "g3_nominal_only_frames":c.get("g3_nominal_only_frames"),
  "g3_node_candidate_path_count":g.get("g3_candidate_paths_published"),
  "g3_posthoc_solver_frames":c.get("g3_posthoc_solver_frames"),
  "moving_object_ground_truth_verified":False,
  "real_robot_control_enabled":False,
  "rviz_recording_is_historical_not_realtime":True,
}
Path(sys.argv[2]).write_text(json.dumps(out,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
print(json.dumps(out,ensure_ascii=False))
PY
then
  RESULT="FAIL"; REASON="demo_pipeline_no_real_g3_evidence"; exit 8
fi
# Give the video a few seconds to show the final retained path and labels.
sleep 5
RESULT="PASS"
REASON="real_bag_offline_g1_g2_g3_displayed_and_recorded"
