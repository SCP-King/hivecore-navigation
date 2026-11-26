#!/usr/bin/env bash
# Repair the vendor RSP lidar mount after an RSP reinstall/update.
# Measured value: base_link -> lidar_link = 0.295, 0, 0.146366.
set -euo pipefail

ROOT="${1:-/opt/hivecore/hive2-motion/current}"
VALUE="0.295 0 0.146366"
patched=0
for f in \
  "$ROOT/install/share/hive2_description/urdf/hive2_body.xacro" \
  "$ROOT/install/share/hive2_description/urdf/hive2_kinematics.xacro" \
  "$ROOT/install/share/hive2_description/urdf/hivecore_sensor_frames.xacro" \
  "$ROOT/install/share/quad_leg_description/urdf/hivecore_sensor_frames.xacro"; do
  [ -f "$f" ] || continue
  if grep -q "0.295 0 0.146366" "$f" 2>/dev/null; then
    echo "already correct: $f"
    continue
  fi
  cp -a "$f" "$f.n01_bak_lidarfix_$(date +%Y%m%d_%H%M%S)"
  sed -i "s/0\\.295 0 0\\([^.]\\|$\\)/0.295 0 0.146366/g" "$f"
  patched=$((patched+1))
  echo "patched: $f"
done

# The check above intentionally matches only the bare 0 value; verify the
# generated xacro if possible.
if command -v xacro >/dev/null 2>&1 && [ -f "$ROOT/install/share/hive2_driver/description/hive2.config.xacro" ]; then
  tmp="$(mktemp)"
  if xacro "$ROOT/install/share/hive2_driver/description/hive2.config.xacro" >"$tmp" 2>/dev/null; then
    if ! grep -q 'lidar_mount_joint' "$tmp" || ! grep -A4 'lidar_mount_joint' "$tmp" | grep -q '0.146366'; then
      echo "[ERR] generated xacro does not contain lidar z=0.146366" >&2
      rm -f "$tmp"; exit 2
    fi
    echo "verified generated lidar_mount_joint: 0.146366"
  fi
  rm -f "$tmp"
fi
echo "patched_files=$patched"
