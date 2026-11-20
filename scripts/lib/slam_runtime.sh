#!/usr/bin/env bash
# One vendor release for GUI, SLAM, navigation and control helper processes.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)/ros_env.sh"

n01_select_slam_runtime() {
  export SLAM_ROSK="${SLAM_ROSK:-/home/demo/lyy/slam3d_20261118}"
  if [[ ! -r "$SLAM_ROSK/setup_runtime.sh" ]]; then
    printf '[N01 SLAM] Missing runtime: %s/setup_runtime.sh\n' "$SLAM_ROSK" >&2
    return 2
  fi
  SLAM_ROSK="$(cd "$SLAM_ROSK" && pwd -P)" || return 2
  export SLAM_ROSK
}

n01_source_slam_runtime() {
  n01_select_slam_runtime || return $?
  # Discard known previous-release paths before joining the selected release.
  # local_setup.bash must be used for the project overlay afterwards: its
  # setup.bash can replay the old vendor underlay captured at build time.
  local variable entry joined prefix
  local -a entries kept
  for variable in AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH \
                  LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH ROS_PACKAGE_PATH; do
    entries=(); kept=()
    IFS=: read -r -a entries <<< "${!variable:-}"
    for entry in "${entries[@]}"; do
      [[ -n "$entry" ]] || continue
      case "$entry" in
        */slam3d_release|*/slam3d_release/*|\
        /home/demo/lyy/slam3d_20261118|/home/demo/lyy/slam3d_20261118/*|\
        /home/demo/lyy/slam3d_lite_20261106_test|/home/demo/lyy/slam3d_lite_20261106_test/*|\
        "$SLAM_ROSK"|"$SLAM_ROSK"/*) continue ;;
      esac
      kept+=("$entry")
    done
    joined="$(IFS=:; printf '%s' "${kept[*]}")"
    if [[ -n "$joined" ]]; then
      printf -v "$variable" '%s' "$joined"; export "$variable"
    else
      unset "$variable"
    fi
  done
  n01_source_setup "$SLAM_ROSK/setup_runtime.sh" || return $?
  # Native IK loads the same SDK as the chassis, without starting its nodes.
  # Its transitive ROS interface libraries need the SDK directory at runtime.
  if [[ -d /opt/hivecore/hive2-motion/1.0.0/install/lib ]]; then
    case ":${LD_LIBRARY_PATH:-}:" in
      *:/opt/hivecore/hive2-motion/1.0.0/install/lib:*) ;;
      *) export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}/opt/hivecore/hive2-motion/1.0.0/install/lib" ;;
    esac
  fi
  # /bin/bash on this robot has cap_sys_nice. Linux secure execution strips
  # inherited LD_LIBRARY_PATH on each child-shell entry; restore the project
  # transport BEFORE ros2 imports rclpy (including ros2 pkg prefix).
  source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)/ros_transport.sh"
  n01_configure_project_transport || return $?
  prefix="$(ros2 pkg prefix hivecore_slam3d_lite_localization)" || return $?
  if [[ "$prefix" != "$SLAM_ROSK/install/"* ]]; then
    printf '[N01 SLAM] Mixed release rejected: %s (expected %s)\n' "$prefix" "$SLAM_ROSK" >&2
    return 2
  fi
  printf '[N01 SLAM] Selected runtime: %s\n' "$SLAM_ROSK" >&2
}
