# N-01 SCAN → DynaCBF isolated reference bridge

**Status: prototype, not a complete navigation planner or safe real-robot controller.**

The package does not modify original SCAN or DynaCBF algorithms.

- `scan_bspline_bridge`: consumes `scan_planner_msgs/Bspline` from `/planning/bspline` plus frame-matched Odometry `/body_pose`, publishes planar arc-length-sampled `nav_msgs/Path` on `/nav/test/global_path` and `/nav/test/bridge_valid` heartbeat. It does **not** verify SCAN 3D collision clearance.
- `dcbf_test_guard`: consumes legacy DynaCBF `/cmd_move` and `/local_plan`, validates age, bounds and heartbeat, publishes **only test** `/nav/test/cmd_vel`; it defaults to `armed=false` and is NOT a physical E-stop or a production motion gate.
- `bridge_smoke_feed`: synthetic curve/odom for standalone ROS 2 smoke checks. Never use as actual localization or SCAN evidence.

**G1 prerequisite:** your local feature worktree must contain `scan_planner_msgs`. As of this commit, that package is not in remote `main`. Merge `origin/main` into the local feature branch rather than discarding unpublished G1 code.

## Test first (does not need ROS)

```bash
python3 -m unittest discover -s src/n01_scan_dynacbf_bridge/test -v
```

## Build with ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
cd /home/demo/lyy/hivecore-navigation
colcon build --packages-up-to n01_scan_dynacbf_bridge --event-handlers console_direct+
source install/setup.bash
```

## Isolated ROS smoke test

Export in *every* test terminal:

```bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export ROS_HOME=/home/demo/lyy/n01_runtime/ros_home/bridge
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
source /home/demo/lyy/hivecore-navigation/install/setup.bash
```

Terminal A: `ros2 launch n01_scan_dynacbf_bridge isolated_bridge.launch.py`

Terminal B: `ros2 run n01_scan_dynacbf_bridge bridge_smoke_feed`

Terminal C:

```bash
ros2 topic echo /nav/test/bridge_valid --once
ros2 topic echo /nav/test/global_path --once
ros2 topic echo /nav/test/cmd_vel --once
ros2 topic echo /nav/test/guard_status --once
ros2 topic info /cmd_vel -v
```

Expected: nonempty reference path; valid heartbeat while synthetic feed is active; `DISARMED` and test output zero; zero actual `/cmd_vel` publishers from this package.

**Never run this synthetic publisher alongside the real SCAN planner in the same ROS domain**: both publish `/planning/bspline`. Do not interpret this synthetic smoke test as real planning, collision avoidance or G1 PASS.

Both entrypoint modules and the launch reject environments other than domain 88 + localhost-only. ROS remaps can still change topic destinations, so do not use arbitrary remapping or start this package in a robot-control launch. Realtime state freshness, TF/time consistency, MPC horizon consistency, infeasible solver cases, and Hive2 locomotion model remain unverified.

License: Apache-2.0, newly authored adapter code. No SCAN or DynaCBF upstream source copied into this package.
