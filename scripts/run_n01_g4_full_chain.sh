#!/usr/bin/env bash
# One command, historical research ONLY: PCD/Bag grid -> G4 geometry and
# reference -> native SCAN mode3 -> G2/G3 observer -> archived evidence.
set -Eeuo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
RUNTIME="${N01_RUNTIME_ROOT:-/home/demo/lyy/n01_runtime}"
BAG="${N01_BAG_PATH:-/home/demo/lyy/slam_data/bags/bag_20261027_225100}"
PCD="${N01_G4_PCD:-/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd}"
PCD_PREFIX="${N01_G4_PCD_PREFIX:-$RUNTIME/pcd_goal/factory_map}"
RAY_PREFIX="${N01_G4_RAY_PREFIX:-$RUNTIME/pcd_goal/bag_ray_20261030_103843}"

if [[ "$(id -u)" == 0 ]]; then
  echo "Do not run ROS integration as root" >&2; exit 2
fi
# Deliberately require a human-supplied, explicit experimental floor Z.
# This never means that the terrain has been surveyed or cleared.
if [[ -z "${N01_G4_GROUND_Z_MAP_M:-}" ]]; then
  echo "Set N01_G4_GROUND_Z_MAP_M to an explicitly inspected TEST assumption." >&2
  echo "No automatically assumed floor height is safe or authoritative." >&2
  exit 2
fi
for input in "$BAG/metadata.yaml" "$PCD" "$PCD_PREFIX.pgm" "$PCD_PREFIX.json" "$RAY_PREFIX.pgm" "$RAY_PREFIX.json"; do
  if [[ ! -r "$input" ]]; then echo "Missing G4 input: $input" >&2; exit 2; fi
done
mkdir -p "$RUNTIME/runs/n01_g4_research"
RUN="$(mktemp -d "$RUNTIME/runs/n01_g4_research/run_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
ARGS=()
if [[ -n "${N01_G4_START_X:-}${N01_G4_START_Y:-}${N01_G4_GOAL_X:-}${N01_G4_GOAL_Y:-}" ]]; then
  if [[ -z "${N01_G4_START_X:-}" || -z "${N01_G4_START_Y:-}" ||
        -z "${N01_G4_GOAL_X:-}" || -z "${N01_G4_GOAL_Y:-}" ]]; then
    echo "Provide all four N01_G4_START_X/Y GOAL_X/Y or none." >&2; exit 2
  fi
  ARGS=(--start "$N01_G4_START_X" "$N01_G4_START_Y"
        --goal "$N01_G4_GOAL_X" "$N01_G4_GOAL_Y")
else
  ARGS=(--research-route)
fi
echo "G4 PREP: strictly read-only PCD + historical Bag grid evidence, no actuators"
if ! python3 "$ROOT/scripts/n01_prepare_g4_research.py" \
  --ray-prefix "$RAY_PREFIX" \
  --pcd-prefix "$PCD_PREFIX" \
  --pcd "$PCD" \
  --ground-z-assumption "$N01_G4_GROUND_Z_MAP_M" \
  --output-dir "$RUN/prepared" "${ARGS[@]}" \
  > "$RUN/prepare.log" 2>&1; then
  cat "$RUN/prepare.log" >&2
  echo "G4_PREP_INCOMPLETE; see $RUN/prepare.log" >&2
  exit 3
fi
echo "G4 reference and geometry reports: $RUN/prepared"
echo "G4 runtime remains HISTORICAL_ONLY; scan initial path is not a safe route."
echo "G4 result will be G4_CHAIN_INCOMPLETE unless NEW SCAN and G2/G3"
echo "messages are independently seen after that exact reference."
export N01_G4_REFERENCE_FILE="$RUN/prepared/g45_scan_reference.json"
export N01_BAG_PATH="$BAG"
exec bash "$ROOT/scripts/run_n01_bag_functional.sh"
