# hivecore-navigation

ROS 2 Humble source code for N-01 navigation research.

- `src/n01_*`: legacy N-01 ROS 2 packages and offline test baseline.
- `src/n01_scan_dynacbf_bridge/`: isolated B-spline reference-path adapter and test-only, **disarmed** command validator.

**Current status:** The six SCAN core packages previously tested in a local feature worktree are **not yet included in this GitHub main branch**. The bridge depends on `scan_planner_msgs` and cannot build until those SCAN packages are integrated. The current repository is **not a complete autonomous navigation stack**. Do not connect it to the Hive2 robot or command the real `/cmd_vel`.

With the required ROS 2 Humble dependencies and SCAN messages available:

```bash
source /opt/ros/humble/setup.bash
colcon list
python3 -m unittest discover -s src/n01_scan_dynacbf_bridge/test -v
colcon build --packages-up-to n01_scan_dynacbf_bridge
```

For isolated launch verification, use `ROS_DOMAIN_ID=88`, `ROS_LOCALHOST_ONLY=1` and a writable `ROS_HOME`. The package provides `isolated_bridge.launch.py` and `bridge_smoke_feed`, both **synthetic TEST-ONLY**. The bridge publishes `/nav/test/global_path` and `/nav/test/cmd_vel`, never a live chassis command.

Runtime data, reports, maps, vendor manuals, large files and generated outputs are intentionally excluded from this source-only branch. `LICENSE` and `NOTICE` are retained.
