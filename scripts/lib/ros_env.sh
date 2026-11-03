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
