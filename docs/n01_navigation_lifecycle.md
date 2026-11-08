# N-01 live navigation lifecycle

The live SCAN navigation stack is managed as one isolated process session. Do not
start the launch directly for routine operation; use the lifecycle entrypoint.

## Commands

From the repository root:

```bash
# Build is required before the first start or after source changes.
colcon build --symlink-install --packages-up-to n01_live_navigation scan_planner n01_static_map

# Start one live navigation stack.
bash scripts/n01_navigation.sh start

# Inspect managed navigation state.
bash scripts/n01_navigation.sh status

# Stop the stack and remove all processes owned by its launch session.
bash scripts/n01_navigation.sh stop
```

The start command first stops a previous managed session and narrowly matched
legacy N-01 navigation orphans, refreshes the local ROS CLI discovery daemon,
then starts one isolated `setsid` launch process group. The PID file is stored
under `$HOME/lyy/n01_runtime/runs/navigation/live_scan_launch.pid`; launch logs
are stored under `$HOME/lyy/n01_runtime/logs/`.

The stop command signals the isolated process group with SIGINT, then escalates
to SIGTERM and SIGKILL only if necessary. It verifies the managed ROS graph has
cleared after refreshing the local ROS CLI daemon. A stale or reused PID is
validated against the expected `ros2 launch n01_live_navigation live_scan.launch.py`
command before any process-group signal is sent.

## One-shot planning acceptance

```bash
bash scripts/run_n01_planning_acceptance.sh
```

This command builds and tests, starts the stack through the lifecycle manager,
runs the read-only goal/planning acceptance, and **always invokes stop on normal
exit, failure, timeout, SIGINT, or SIGTERM**. The navigation stack does not remain
running after this one-shot acceptance command exits.

## Protected processes

The lifecycle manager never targets the independently managed Lite SLAM process,
an independently running `/n01_localization_adapter`, Livox, IMU, robot drivers,
or actuator controllers. If the lifecycle manager itself starts the adapter
because none exists, that adapter is inside the isolated launch session and is
stopped with that session.

The navigation stack remains planning-only. Actuator output stays disabled;
this lifecycle does not authorize robot motion.
