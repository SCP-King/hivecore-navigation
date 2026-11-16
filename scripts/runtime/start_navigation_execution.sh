#!/usr/bin/env bash
# Internal GUI lifecycle: source the N-01 workspace before ros2 run, otherwise the GUI
# child environment (vendor SLAM only) cannot find n01_live_navigation.
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
N01_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$N01_PROJECT_ROOT/scripts/lib/slam_runtime.sh"
n01_source_slam_runtime
source "$N01_PROJECT_ROOT/install/local_setup.bash"
if [[ ! -w "$HOME/.ros/log" ]]; then
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/n01_ros_execution_log}"
  export ROS_HOME="${ROS_HOME:-/tmp/n01_ros_execution_home}"
  mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
fi
export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-ASYNCHRONOUS}"
# Hardware RPC and publisher-GID ownership stay on the verified control transport.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Persist the selected configuration and explicit overrides for reproducibility.
python3 - "$N01_PROJECT_ROOT/config/navigation.yaml" "$@" <<'PY_RESOLVED'
import sys,json,time,hashlib
from pathlib import Path
p=Path(sys.argv[1]);root=p.parents[2]/'n01_runtime'/'resolved';root.mkdir(parents=True,exist_ok=True)
out=root/f'navigation_{time.time_ns()}.json'
out.write_text(json.dumps({'configuration':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),
 'explicit_arguments':sys.argv[2:],'model_source':'live chassis GetParameters + /n01/robot_footprint'},ensure_ascii=False,indent=2)+'\n')
print(f'[N01 navigation] resolved configuration: {out}',flush=True)
PY_RESOLVED
# One normal navigation output owner per ROS domain; lock releases on exit.
N01_CONTROLLER_DOMAIN="${ROS_DOMAIN_ID:-0}"
[[ "$N01_CONTROLLER_DOMAIN" =~ ^[0-9]+$ ]] || exit 2
exec flock -n "/tmp/n01-navigation-${UID}-${N01_CONTROLLER_DOMAIN}.lock" \
  ros2 run scan_planner n01_navigation_controller --ros-args \
  --params-file "$N01_PROJECT_ROOT/config/navigation.yaml" "$@"
