#!/usr/bin/env bash
# G4.8: first accepted REAL historical Bag scan determines the research start.
# Unknown ground Z => do a strict read-only preflight, NEVER invent 3D Z.
# No ROS nodes launched until explicit operator-verified floor is supplied.
set -Eeuo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
RUNTIME="${N01_RUNTIME_ROOT:-/home/demo/lyy/n01_runtime}"
BAG="${N01_BAG_PATH:-/home/demo/lyy/slam_data/bags/bag_20261027_225100}"
PCD="${N01_G4_PCD:-/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd}"
PCD_PREFIX="${N01_G4_PCD_PREFIX:-$RUNTIME/pcd_goal/factory_map}"
RAY_PREFIX="${N01_G4_RAY_PREFIX:-$RUNTIME/pcd_goal/bag_ray_20261030_103843}"

if [[ "$(id -u)" == 0 ]]; then
  echo "Do not run as root" >&2; exit 2
fi
for input in "$BAG/metadata.yaml" "$PCD" "$PCD_PREFIX.pgm" "$PCD_PREFIX.json" "$RAY_PREFIX.pgm" "$RAY_PREFIX.json"; do
  if [[ ! -r "$input" ]]; then echo "Missing G4 input: $input" >&2; exit 2; fi
done
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble installation is required to decode the historical Bag" >&2; exit 2
fi
# rosbag2_py Python imports need the ROS overlay, but this does NOT launch a
# ROS node or connect to a robot.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
mkdir -p "$RUNTIME/runs/n01_g4_research"
RUN="$(mktemp -d "$RUNTIME/runs/n01_g4_research/run_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
echo "G4.8: reading first accepted historical Livox scan pose (NO guess, NO actuators)"
if ! python3 "$ROOT/scripts/n01_g4_bag_start_preflight.py" \
  --bag "$BAG" --pcd "$PCD" \
  --ray-prefix "$RAY_PREFIX" --pcd-prefix "$PCD_PREFIX" \
  --output-dir "$RUN/bag_start_preflight" \
  > "$RUN/bag_start.log" 2>&1; then
  cat "$RUN/bag_start.log" >&2
  echo "G4.8_BLOCKED: $RUN/bag_start_preflight/g48_preflight.json" >&2
  exit 3
fi
cat "$RUN/bag_start.log"
echo "G4.8 evidence: $RUN/bag_start_preflight/g48_preflight.json"
echo "G4.8 conflict map: $RUN/bag_start_preflight/g43_evidence.svg"

# As requested, the floor Z is UNKNOWN. A scientific research Z may not be
# silently invented from a slice, a PCD percentile, or the Hive2 mount height.
if [[ -z "${N01_G4_GROUND_Z_MAP_M:-}" ]]; then
  echo "G4.8_XY_READY_Z_UNKNOWN: stopped before SCAN (correct; ground not yet verified)."
  echo "Inspect the preflight report; no 3D path or ROS node was created."
  exit 0
fi
if [[ "${N01_G4_FLOOR_Z_VERIFIED:-}" != "1" ]]; then
  echo "BLOCKED_UNVERIFIED_GROUND_Z: remove N01_G4_GROUND_Z_MAP_M to inspect XY only." >&2
  echo "Only supply N01_G4_FLOOR_Z_VERIFIED=1 after actual map-frame terrain evidence." >&2
  exit 4
fi

# START = first accepted timestamped Bag TF at an actual recorded scan.
# GOAL = farthest connected observed-ray-free cell reachable from that start.
# Both are computed from evidence, never from an arbitrary synthetic target.
mapfile -t COORDS < <(python3 - "$RUN/bag_start_preflight/g48_preflight.json" <<'PY'
import json,sys
d=json.load(open(sys.argv[1],encoding="utf-8"))
assert d["result"] == "BAG_ANCHORED_XY_RESEARCH_ROUTE_READY_Z_UNKNOWN"
start=d["first_accepted_bag_pose"]["map_to_base_footprint_xy_m"]
goal=d["selected_research_goal"]["goal_xy_m"]
for value in (*start,*goal):
    print(value)
PY
)
if [[ "${#COORDS[@]}" -ne 4 ]]; then
  echo "G4 anchor extraction failed" >&2; exit 4
fi
echo "G4 START (first accepted Bag scan): ${COORDS[0]} ${COORDS[1]}"
echo "G4 GOAL (observed 4-connected research component): ${COORDS[2]} ${COORDS[3]}"
echo "G4: 3D reference still research-only; not certified ground or body clearance."
if ! python3 "$ROOT/scripts/n01_prepare_g4_research.py" \
  --ray-prefix "$RAY_PREFIX" --pcd-prefix "$PCD_PREFIX" --pcd "$PCD" \
  --ground-z-assumption "$N01_G4_GROUND_Z_MAP_M" \
  --output-dir "$RUN/prepared" \
  --start "${COORDS[0]}" "${COORDS[1]}" \
  --goal "${COORDS[2]}" "${COORDS[3]}" \
  > "$RUN/prepare.log" 2>&1; then
  cat "$RUN/prepare.log" >&2
  echo "G4_PREP_INCOMPLETE; see $RUN/prepare.log" >&2
  exit 3
fi
echo "G4 prepared research artifacts: $RUN/prepared"
echo "Do not connect this offline test to an actual Hive2 motion controller."
export N01_G4_REFERENCE_FILE="$RUN/prepared/g45_scan_reference.json"
export N01_BAG_PATH="$BAG"
exec bash "$ROOT/scripts/run_n01_bag_functional.sh"
