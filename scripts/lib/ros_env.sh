#!/usr/bin/env bash
# Shared ROS/colcon setup loader for N-01 scripts.
#
# ROS Humble and colcon-generated setup files may read optional variables while
# being sourced. In particular, they are not compatible with Bash nounset
# (set -u) unless every optional variable has been initialized. This helper
# temporarily disables only nounset, preserves the caller's original setting,
# captures the source status, and leaves errexit/pipefail unchanged.
#
# Usage:
#   source "$ROOT/scripts/lib/ros_env.sh"
#   n01_source_setup /opt/ros/humble/setup.bash
#   n01_source_setup "$ROOT/install/setup.bash"

# Remove only this workspace's generated install prefixes from an inherited
# shell. Preserve ROS/vendor underlays already sourced by the operator.
# This prevents colcon from treating the workspace being rebuilt as its own
# underlay while retaining vendor message libraries and environment hooks.
n01_strip_workspace_prefixes() {
  local workspace_root="${1:-}"
  local workspace_real install_real variable value entry entry_real joined
  local -a entries=()
  local -a kept=()

  if [[ -z "$workspace_root" || ! -d "$workspace_root" ]]; then
    printf '[N01 ROS ENV] ERROR: workspace root is missing: %s\n' "$workspace_root" >&2
    return 2
  fi
  workspace_real="$(cd "$workspace_root" && pwd -P)" || return 2
  install_real="$workspace_real/install"

  for variable in \
    AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH \
    LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH ROS_PACKAGE_PATH
  do
    value="${!variable:-}"
    [[ -n "$value" ]] || continue
    entries=()
    kept=()
    IFS=: read -r -a entries <<< "$value"
    for entry in "${entries[@]}"; do
      [[ -n "$entry" ]] || continue
      entry_real="$(realpath -m "$entry" 2>/dev/null || printf '%s' "$entry")"
      case "$entry_real" in
        "$install_real"|"$install_real"/*)
          continue
          ;;
      esac
      kept+=("$entry")
    done
    if (( ${#kept[@]} )); then
      joined="$(IFS=:; printf '%s' "${kept[*]}")"
      printf -v "$variable" '%s' "$joined"
      export "$variable"
    else
      unset "$variable"
    fi
  done
  printf '[N01 ROS ENV] Removed only %s/install entries from inherited ROS/build paths; other underlays were preserved.\n' "$workspace_real"
}

n01_source_setup() {
  local setup_file="${1:-}"
  local had_nounset=0
  local source_rc=0

  if [[ -z "$setup_file" ]]; then
    printf '[N01 ROS ENV] ERROR: setup file argument is empty.\n' >&2
    return 2
  fi
  if [[ ! -r "$setup_file" ]]; then
    printf '[N01 ROS ENV] ERROR: setup file is not readable: %s\n' "$setup_file" >&2
    return 2
  fi

  case "$-" in
    *u*) had_nounset=1 ;;
  esac

  set +u
  if source "$setup_file"; then
    source_rc=0
  else
    source_rc=$?
  fi

  if (( had_nounset )); then
    set -u
  fi

  if (( source_rc != 0 )); then
    printf '[N01 ROS ENV] ERROR: sourcing %s failed (exit=%d).\n' \
      "$setup_file" "$source_rc" >&2
  fi
  return "$source_rc"
}
