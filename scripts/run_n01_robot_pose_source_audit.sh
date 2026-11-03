#!/usr/bin/env bash
# Focused, read-only audit of the two /robot_pose publisher endpoints.
# Captures endpoint GIDs/QoS, aggregate pose timing, bridge counters and node wiring.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd -P)"
RUNTIME_ROOT="$(printenv N01_RUNTIME_ROOT 2>/dev/null || true)"
if [[ -z "$RUNTIME_ROOT" ]]; then RUNTIME_ROOT="$HOME/lyy/n01_runtime"; fi
OUT_DIR="$RUNTIME_ROOT/runs/robot_pose_source_audit"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="$OUT_DIR/n01_robot_pose_source_audit_$STAMP.json"
LOG="$OUT_DIR/n01_robot_pose_source_audit_$STAMP.log"
ENDPOINTS="$OUT_DIR/n01_robot_pose_endpoints_$STAMP.txt"
BUNDLE="$OUT_DIR/n01_robot_pose_source_audit_$STAMP.tar.gz"
DURATION=20
if (( $# > 0 )) && [[ -n "$1" ]]; then DURATION="$1"; fi

mkdir -p "$OUT_DIR"
exec > >(tee -a "$LOG") 2>&1
source "$ROOT/scripts/lib/ros_env.sh"

if [[ -z "$(printenv ROS_DOMAIN_ID 2>/dev/null || true)" ]]; then export ROS_DOMAIN_ID=55; fi
if [[ -z "$(printenv RMW_IMPLEMENTATION 2>/dev/null || true)" ]]; then export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; fi
if [[ -z "$(printenv ROS_LOCALHOST_ONLY 2>/dev/null || true)" ]]; then export ROS_LOCALHOST_ONLY=0; fi

echo "=== N-01 /robot_pose source audit (READ ONLY) ==="
echo "workspace=$ROOT"
echo "branch=$(git -C "$ROOT" branch --show-current 2>/dev/null || echo unknown)"
echo "commit=$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "duration_s=$DURATION"
echo "report=$REPORT"
echo "No publishers, clients, services, actions, goals, parameter writes, restarts or actuator commands."

if [[ ! -r /opt/ros/humble/setup.bash || ! -r "$ROOT/install/setup.bash" ]]; then
  echo "ERROR: ROS Humble or workspace install/setup.bash is missing."
  exit 2
fi
n01_source_setup /opt/ros/humble/setup.bash
n01_source_setup "$ROOT/install/setup.bash"

ROS_PYTHON="$(printenv N01_ROS_PYTHON 2>/dev/null || true)"
if [[ -z "$ROS_PYTHON" || ! -x "$ROS_PYTHON" ]]; then ROS_PYTHON=/usr/bin/python3; fi
if [[ ! -x "$ROS_PYTHON" ]]; then ROS_PYTHON="$(command -v python3)"; fi

set +e
"$ROS_PYTHON" "$ROOT/scripts/n01_live_stack_preflight.py" \
  --duration "$DURATION" > "$REPORT" 2>&1
PREFLIGHT_RC=$?
set -e
echo "PREFLIGHT_EXIT_CODE=$PREFLIGHT_RC"

{
  echo "=== /robot_pose endpoint graph ==="
  timeout --signal=INT --kill-after=2s 10s ros2 topic info /robot_pose --verbose 2>&1 || true
  echo
  echo "=== publisher node details ==="
} > "$ENDPOINTS"

while IFS= read -r node; do
  [[ -n "$node" ]] || continue
  {
    echo
    echo "--- ros2 node info $node ---"
    timeout --signal=INT --kill-after=2s 8s ros2 node info "$node" 2>&1 || true
  } >> "$ENDPOINTS"
done < <("$ROS_PYTHON" - "$REPORT" <<'PY'
import json
import sys
path = sys.argv[1]
try:
    with open(path, encoding="utf-8") as stream:
        data = json.load(stream)
except Exception:
    raise SystemExit(0)
names = set()
for endpoint in data.get("multiple_publisher_endpoints", {}).get("/robot_pose", {}).get("publishers", []):
    if endpoint.get("node"):
        names.add(endpoint["node"])
for endpoint in data.get("topics", {}).get("/robot_pose", {}).get("publishers", []):
    if endpoint.get("node"):
        names.add(endpoint["node"])
for name in sorted(names):
    print(name)
PY
)

{
  echo
  echo "=== relevant process snapshot ==="
  ps -eo pid,ppid,etimes,args | grep -E '[s]lam3d_node|[n]01_live|[s]can_planner|[l]ivox|[h]ivecore' || true
  echo
  echo "=== safety statement ==="
  echo "read_only=true"
  echo "goal_published=false"
  echo "actuator_output=DISABLED"
  echo "safety_authorization=NEVER"
  echo "Note: ROS 2 rclpy callback does not expose per-message DDS publisher GID."
  echo "Endpoint identity and aggregate sample timing are reported separately."
} >> "$ENDPOINTS"

echo
echo "=== structured preflight report ==="
cat "$REPORT"
echo
echo "=== endpoint/node detail ==="
cat "$ENDPOINTS"

tar -czf "$BUNDLE" -C "$OUT_DIR" \
  "$(basename "$REPORT")" "$(basename "$ENDPOINTS")" "$(basename "$LOG")"

echo
echo "PREFLIGHT_EXIT_CODE=$PREFLIGHT_RC"
echo "REPORT=$REPORT"
echo "ENDPOINTS=$ENDPOINTS"
echo "UPLOAD THIS BUNDLE: $BUNDLE"
echo "READ_ONLY: no goals, actuator output, service/action calls, parameter writes or restarts."
exit "$PREFLIGHT_RC"
