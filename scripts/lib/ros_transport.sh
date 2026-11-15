#!/usr/bin/env bash
# Apply only to the project process tree; never change desktop launch files.
n01_configure_project_transport() {
  local selected="${N01_RUNTIME_RMW:-auto}"
  local prefix="${N01_CYCLONE_PREFIX:-${N01_TRANSPORT_ROOT:-$HOME/.local/share/n01_navigation/cyclonedds}/root/opt/ros/humble}"
  if [[ "$selected" == auto ]]; then
    if [[ -r "$prefix/lib/librmw_cyclonedds_cpp.so" && -r "$prefix/lib/x86_64-linux-gnu/libddsc.so.0" ]]; then
      selected=rmw_cyclonedds_cpp
    else
      selected="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
      [[ "$selected" != rmw_cyclonedds_cpp ]] || selected=rmw_fastrtps_cpp
      echo "[WARN] Private CycloneDDS runtime unavailable; retaining $selected" >&2
    fi
  fi
  if [[ "$selected" != rmw_cyclonedds_cpp ]]; then
    export RMW_IMPLEMENTATION="$selected"
    return 0
  fi
  if [[ ! -r "$prefix/lib/librmw_cyclonedds_cpp.so" || ! -r "$prefix/lib/x86_64-linux-gnu/libddsc.so.0" ]]; then
    echo "[ERR] Project CycloneDDS runtime missing: $prefix; set N01_RUNTIME_RMW=rmw_fastrtps_cpp for explicit fallback." >&2
    return 1
  fi
  export AMENT_PREFIX_PATH="$prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$prefix/lib:$prefix/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
}
