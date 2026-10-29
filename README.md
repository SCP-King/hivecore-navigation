# hivecore-navigation

ROS 2 Humble source-only N-01 navigation research. Apache-2.0.

## Components

- `src/scan_planner/`: SCAN-Planner **real ROS 2** 3D map, DynAStar, B-spline optimization, FSM and `scan_planner_msgs`. Imported from `wuyi2121/SCAN-Planner`, `ros2-community` commit `d0b921c9b05a6d291d144d60882b2e0e88d2c0e0`. See `src/scan_planner/UPSTREAM_SOURCE.txt` and `NOTICE`. Only `scan_planner_node` is built; original control and simulator executables are **not included**.
- `src/n01_scan_dynacbf_bridge/`: test-only SCAN B-spline to `nav_msgs/Path` reference adapter and default-disarmed output validator, **not** real dynamic-avoidance MPC.
- Other `src/n01_*` packages: legacy N-01 baseline; do not count the legacy 2D A* as SCAN.

**Status: source migrated, NOT yet verified with this main-branch code on your Ubuntu/ROS 2 machine. NOT ready for a real robot.** No DynaCBF detection/Kalman/MPC-CBF integration yet.

## G1 build

```bash
cd /home/demo/lyy/hivecore-navigation
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s src/n01_scan_dynacbf_bridge/test -v
colcon build --packages-up-to scan_planner n01_scan_dynacbf_bridge --event-handlers console_direct+
source install/setup.bash
```

## G1 isolated smoke test

Set these in **each** terminal, without loading any real robot or SLAM overlays:

```bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export ROS_HOME=/home/demo/lyy/n01_runtime/ros_home/g1_scan
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
source /home/demo/lyy/hivecore-navigation/install/setup.bash
```

- Terminal A: `ros2 launch scan_planner n01_isolated.launch.py` (real SCAN algorithms, no controller)
- Terminal B: `ros2 run n01_scan_dynacbf_bridge scan_g1_feed` (deterministic offline pointcloud/poses/goal; no TF or cmd_vel)
- Terminal C: `ros2 run n01_scan_dynacbf_bridge scan_g1_monitor` (40-second observer, saves `$ROS_HOME/g1_smoke_report.json`)
- Verify: `ros2 topic info /cmd_vel -v`; no real command publisher allowed. When all tests are stopped, report results.

**Never run `bridge_smoke_feed` alongside `scan_g1_feed` or real SCAN**, because the former publishes synthetic `/planning/bspline`. This G1 smoke test checks live message observations only; trajectory collision, actual MPC, robot-model compatibility, 500 ms requirement and long-duration tests remain NOT TESTED.

Keep PCD, Bag, manuals, offline test outputs and reports outside this source-only repository.
