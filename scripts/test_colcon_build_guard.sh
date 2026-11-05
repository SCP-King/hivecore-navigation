#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
# shellcheck disable=SC1091
source "$ROOT/scripts/lib/colcon_build_guard.sh"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
WS="$TMP/workspace"
RUNTIME="$TMP/runtime"
TARGET="$WS/build/scan_planner_msgs/ament_cmake_python/scan_planner_msgs/scan_planner_msgs"
mkdir -p "$TARGET" "$WS/src/scan_planner/scan_planner_msgs"
cat > "$WS/src/scan_planner/scan_planner_msgs/package.xml" <<'EOF'
<package><name>scan_planner_msgs</name></package>
EOF
printf 'generated-stale-directory-marker\n' > "$TARGET/stale.txt"

OUTPUT="$(n01_repair_generated_python_link_collision "$WS" "$RUNTIME")"
[[ ! -e "$TARGET" && ! -L "$TARGET" ]]
BACKUP="$(printf '%s\n' "$OUTPUT" | sed -n 's/^N01_COLCON_COLLISION_BACKUP=//p')"
[[ -s "$BACKUP" ]]
tar -xOzf "$BACKUP" "scan_planner_msgs/stale.txt" | grep -Fxq 'generated-stale-directory-marker'
echo "PASS: exact generated directory was archived before removal"

mkdir -p "$WS/build/scan_planner_msgs/ament_cmake_python/scan_planner_msgs"
printf 'symlink-target\n' > "$WS/src/scan_planner/scan_planner_msgs/scan_planner_msgs"
ln -s "$WS/src/scan_planner/scan_planner_msgs/scan_planner_msgs" \
  "$WS/build/scan_planner_msgs/ament_cmake_python/scan_planner_msgs/scan_planner_msgs"
n01_repair_generated_python_link_collision "$WS" "$RUNTIME" >/dev/null
[[ -L "$WS/build/scan_planner_msgs/ament_cmake_python/scan_planner_msgs/scan_planner_msgs" ]]
echo "PASS: existing correct symlink is preserved"

echo "Colcon generated-link guard tests: PASS"
