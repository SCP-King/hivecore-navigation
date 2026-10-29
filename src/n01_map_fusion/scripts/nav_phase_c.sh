#!/usr/bin/env bash
# Phase 5-C: 单实例运行中障碍注入闭环(隔离域,不碰真机).
# 先让同一个 sim_motion 实际移动,再注入障碍;不启动 sim_motion2.
set -o pipefail

DOMAIN=88
ROOT=/home/demo/lyy/hivecore-navigation
PCD=/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd
OUT=$ROOT/src/n01_map_fusion/output/phase_c
PARAMS=$ROOT/src/n01_map_fusion/config/fusion_planner.yaml
COLLECTOR=$ROOT/src/n01_map_fusion/n01_map_fusion/loop_collector.py
GATE=$ROOT/src/n01_map_fusion/scripts/phase_c_gate.py
REPORT=$OUT/loop_report_c.json
GATE_JSON=$OUT/gate.json
STDOUT_LOG=$OUT/phase_c.stdout.log
VERDICT=$OUT/phase_c_report.json

PHASE_S=32
COLLECT_S=29
PIDS=()

export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1
export ROS_HOME=$OUT/.ros_home ROS_LOG_DIR=$OUT/.ros_home/log
export COLCON_CURRENT_PREFIX="${COLCON_CURRENT_PREFIX:-}" AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-$(which python3)}" ROS_DISTRO="${ROS_DISTRO:-humble}" ROS_VERSION="${ROS_VERSION:-2}"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
mkdir -p "$OUT" "$ROS_HOME" "$ROS_LOG_DIR"
rm -f "$REPORT" "$GATE_JSON" "$VERDICT" "$STDOUT_LOG"

run_bg() {
    local name=$1; shift
    setsid "$@" >"$OUT/$name.log" 2>&1 &
    local p=$!
    PIDS+=("$p")
    printf '%s\n' "$p"
}

kill_group() {
    local p=$1
    kill -- "-$p" 2>/dev/null || kill "$p" 2>/dev/null || true
}

cleanup() {
    local p
    for p in "${PIDS[@]}"; do kill_group "$p"; done
    sleep 1
    for p in "${PIDS[@]}"; do
        kill -9 -- "-$p" 2>/dev/null || kill -9 "$p" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

{
    printf '[phase-c] domain=%s localhost-only\n' "$DOMAIN"
    printf '[phase-c] single sim_motion; report=%s\n' "$REPORT"
    printf '[phase-c] start static_map\n'
    run_bg static_map ros2 run n01_static_map static_map_node --ros-args -p pcd_path:="$PCD" >/dev/null
    sleep 1
    printf '[phase-c] start fusion\n'
    run_bg fusion ros2 run n01_map_fusion fusion_node --ros-args --params-file "$PARAMS" >/dev/null
    sleep 1
    printf '[phase-c] start planner with start_topic=/nav/test/pose\n'
    run_bg planner ros2 run n01_map_fusion planner_node --ros-args --params-file "$PARAMS" \
        -p start_topic:=/nav/test/pose -p start_x:=11.05 -p start_y:=8.60 >/dev/null
    sleep 1
    printf '[phase-c] start sole sim_motion\n'
    run_bg sim_motion ros2 run n01_map_fusion sim_motion --ros-args \
        -p start_x:=11.05 -p start_y:=8.60 -p start_yaw:=1.5708 \
        -p waypoint_tol:=0.03 >/dev/null
    run_bg collector_c python3 -u "$COLLECTOR" --output "$REPORT" --quit-after "$COLLECT_S" >/dev/null
    printf '[phase-c] waiting for displacement>=0.25m, goal distance>0.6m, safe obstacle site\n'
    set +e
    python3 -u "$GATE" >"$GATE_JSON" 2>"$OUT/gate.log"
    gate_rc=$?
    set -e
    if [ "$gate_rc" -ne 0 ]; then
        printf '[phase-c] FAIL gate rc=%s\n' "$gate_rc"
        python3 - "$VERDICT" "$GATE_JSON" <<'PY'
import json, sys
out, gate = sys.argv[1:]
try: data=json.load(open(gate))
except Exception as e: data={'gate_read_error': str(e)}
json.dump({'overall':'FAIL','failure_stage':'gate','gate':data}, open(out,'w'), ensure_ascii=False, indent=2)
PY
        exit 2
    fi
    printf '[phase-c] gate='; python3 -c 'import json,sys; print(json.dumps(json.load(open(sys.argv[1])),ensure_ascii=False))' "$GATE_JSON"
    inject_epoch=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["wall_epoch"])' "$GATE_JSON")
    printf '[phase-c] inject obstacle at (11.05,7.15), r=0.15, epoch=%s\n' "$inject_epoch"
    run_bg mock_obstacle ros2 run n01_map_fusion mock_obstacle --ros-args \
        -p center_x:=11.05 -p center_y:=7.15 -p radius:=0.15 \
        -p publish_after_s:=0.0 -p rate:=2.0 >/dev/null
    printf '[phase-c] wait %ss for same sim to replan and reach/hold goal\n' "$PHASE_S"
    sleep "$PHASE_S"
    printf '[phase-c] collecting and verifying\n'
    python3 "$ROOT/src/n01_map_fusion/scripts/phase_c_verify.py" \
        --report "$REPORT" --gate "$GATE_JSON" --inject-epoch "$inject_epoch" \
        --output "$VERDICT"
    rc=$?
    printf '[phase-c] verifier_rc=%s\n' "$rc"
    printf '[phase-c] reports: %s %s\n' "$REPORT" "$VERDICT"
    exit "$rc"
} 2>&1 | tee "$STDOUT_LOG"
exit "${PIPESTATUS[0]}"
