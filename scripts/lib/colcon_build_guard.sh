#!/usr/bin/env bash
# Narrow recovery for the exact ament_cmake_python symlink collision observed
# in build/scan_planner_msgs. Never removes an install prefix or whole build tree.
#
# Usage: n01_repair_generated_python_link_collision WORKSPACE_ROOT RUNTIME_ROOT
n01_repair_generated_python_link_collision() {
  local workspace_root="${1:-}"
  local runtime_root="${2:-}"
  local package="scan_planner_msgs"
  local manifest_rel="src/scan_planner/scan_planner_msgs/package.xml"
  local workspace_real build_root build_real parent_real collision
  local backup_dir backup stamp

  if [[ -z "$workspace_root" || -z "$runtime_root" ]]; then
    echo "[N01 COLCON GUARD] ERROR: workspace and runtime roots are required." >&2
    return 2
  fi
  if [[ ! -d "$workspace_root" ]]; then
    echo "[N01 COLCON GUARD] ERROR: workspace does not exist: $workspace_root" >&2
    return 2
  fi
  workspace_real="$(cd "$workspace_root" && pwd -P)" || return 2
  if [[ ! -r "$workspace_real/$manifest_rel" ]] ||
     ! grep -Eq '<name>[[:space:]]*scan_planner_msgs[[:space:]]*</name>' "$workspace_real/$manifest_rel"; then
    echo "[N01 COLCON GUARD] ERROR: scan_planner_msgs source manifest not verified." >&2
    return 2
  fi

  build_root="$workspace_real/build/$package"
  collision="$build_root/ament_cmake_python/$package/$package"
  if [[ -L "$collision" ]]; then
    echo "[N01 COLCON GUARD] Existing target is already a symlink; no repair needed."
    return 0
  fi
  if [[ ! -e "$collision" ]]; then
    echo "[N01 COLCON GUARD] No stale generated Python-link collision detected."
    return 0
  fi
  if [[ ! -d "$build_root" || ! -d "$(dirname "$collision")" ]]; then
    echo "[N01 COLCON GUARD] ERROR: collision path exists but generated build parents are incomplete." >&2
    return 3
  fi
  if [[ ! -d "$collision" || -L "$collision" ]]; then
    echo "[N01 COLCON GUARD] ERROR: expected a real directory at the collision path; refusing to alter an unexpected object." >&2
    echo "  $collision" >&2
    return 3
  fi

  build_real="$(realpath -e "$build_root")" || return 3
  parent_real="$(realpath -e "$(dirname "$collision")")" || return 3
  case "$build_real/" in
    "$workspace_real/build/"*) ;;
    *)
      echo "[N01 COLCON GUARD] ERROR: package build path resolves outside workspace build/: $build_real" >&2
      return 3
      ;;
  esac
  if [[ "$parent_real" != "$build_real/ament_cmake_python/$package" ]]; then
    echo "[N01 COLCON GUARD] ERROR: collision parent does not match the exact generated package path." >&2
    echo "  expected: $build_real/ament_cmake_python/$package" >&2
    echo "  actual:   $parent_real" >&2
    return 3
  fi

  stamp="${N01_COLCON_GUARD_STAMP:-$(date +%Y%m%d_%H%M%S)}"
  backup_dir="$runtime_root/runs/colcon_build_recovery"
  mkdir -p "$backup_dir" || return 4
  backup="$backup_dir/${package}_python_link_collision_${stamp}_$$.tar.gz"

  echo "[N01 COLCON GUARD] Found stale generated directory blocking ament symlink creation:"
  echo "  $collision"
  echo "[N01 COLCON GUARD] Archiving this exact generated directory before removal:"
  echo "  $backup"
  if ! tar -czf "$backup" -C "$parent_real" "$package"; then
    echo "[N01 COLCON GUARD] ERROR: backup archive failed; original directory was not changed." >&2
    rm -f -- "$backup"
    return 4
  fi
  if ! tar -tzf "$backup" >/dev/null 2>&1; then
    echo "[N01 COLCON GUARD] ERROR: backup archive verification failed; original directory was not changed." >&2
    rm -f -- "$backup"
    return 4
  fi
  if ! rm -rf -- "$collision"; then
    echo "[N01 COLCON GUARD] ERROR: archived directory could not be removed; inspect manually." >&2
    return 5
  fi
  if [[ -e "$collision" || -L "$collision" ]]; then
    echo "[N01 COLCON GUARD] ERROR: collision path still exists after narrow cleanup." >&2
    return 5
  fi

  echo "[N01 COLCON GUARD] Recovered exact generated path; source and install trees were untouched."
  echo "N01_COLCON_COLLISION_BACKUP=$backup"
}
