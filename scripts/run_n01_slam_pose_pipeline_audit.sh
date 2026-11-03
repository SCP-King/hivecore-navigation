#!/usr/bin/env bash
# One-command, read-only SLAM pose pipeline forensic audit.
set -Eeuo pipefail
umask 077

ROOT="$(cd "$(dirname "$0")/.." && pwd -P)"
RUNTIME_ROOT="$(printenv N01_RUNTIME_ROOT 2>/dev/null || true)"
if [[ -z "$RUNTIME_ROOT" ]]; then RUNTIME_ROOT="$HOME/lyy/n01_runtime"; fi
OUT_DIR="$RUNTIME_ROOT/runs/slam_pose_pipeline_audit"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="$OUT_DIR/n01_slam_pose_pipeline_audit_$STAMP.json"
LOG="$OUT_DIR/n01_slam_pose_pipeline_audit_$STAMP.log"
BUNDLE="$OUT_DIR/n01_slam_pose_pipeline_audit_$STAMP.tar.gz"
DURATION=30
if (( $# > 0 )) && [[ -n "$1" ]]; then DURATION="$1"; fi

mkdir -p "$OUT_DIR"
exec > >(tee -a "$LOG") 2>&1
source "$ROOT/scripts/lib/ros_env.sh"

if [[ -z "$(printenv ROS_DOMAIN_ID 2>/dev/null || true)" ]]; then export ROS_DOMAIN_ID=55; fi
if [[ -z "$(printenv RMW_IMPLEMENTATION 2>/dev/null || true)" ]]; then export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; fi
if [[ -z "$(printenv ROS_LOCALHOST_ONLY 2>/dev/null || true)" ]]; then export ROS_LOCALHOST_ONLY=0; fi

echo "=== Hivecore SLAM pose pipeline forensic audit (READ ONLY) ==="
echo "workspace=$ROOT"
echo "branch=$(git -C "$ROOT" branch --show-current 2>/dev/null || echo unknown)"
echo "commit=$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "duration_s=$DURATION"
echo "report=$REPORT"
echo "This audit subscribes/reads only. It does not publish, call services/actions, change parameters, restart processes or command actuators."

if [[ ! -r /opt/ros/humble/setup.bash || ! -r "$ROOT/install/setup.bash" ]]; then
  echo "ERROR: ROS Humble or N-01 workspace install/setup.bash is missing."
  exit 2
fi
n01_source_setup /opt/ros/humble/setup.bash
n01_source_setup "$ROOT/install/setup.bash"

ROS_PYTHON="$(printenv N01_ROS_PYTHON 2>/dev/null || true)"
if [[ -z "$ROS_PYTHON" || ! -x "$ROS_PYTHON" ]]; then ROS_PYTHON=/usr/bin/python3; fi
if [[ ! -x "$ROS_PYTHON" ]]; then ROS_PYTHON="$(command -v python3)"; fi

set +e
"$ROS_PYTHON" "$ROOT/scripts/n01_slam_pose_pipeline_audit.py" \
  --duration "$DURATION" --output "$REPORT"
AUDIT_RC=$?
set -e

if [[ -f "$REPORT" ]]; then
  tar -czf "$BUNDLE" -C "$OUT_DIR" "$(basename "$REPORT")" "$(basename "$LOG")"
fi

echo
echo "AUDIT_EXIT_CODE=$AUDIT_RC"
echo "REPORT=$REPORT"
if [[ -f "$BUNDLE" ]]; then
  echo "UPLOAD THIS BUNDLE: $BUNDLE"
else
  echo "ERROR: audit bundle was not created."
fi
echo "READ_ONLY: no publisher, client, goal, service/action call, parameter write, restart or actuator command."
exit "$AUDIT_RC"
