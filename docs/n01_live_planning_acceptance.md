# N-01 Live Planning Acceptance

## Scope

This workflow validates the live, non-actuating chain:

Lite SLAM localization -> N-01 localization adapter -> live scan bridge ->
static map / dynamic scene -> Goal Gate -> native SCAN B-spline ->
Hive2 candidate kinematic review.

It does not publish a goal automatically, publish actuator commands, release
operator hold, or authorize robot motion.

## Single-instance rule

Only one `live_scan.launch.py` stack may run in a ROS domain. The launch file
uses a domain-scoped OS `flock` and checks the ROS graph before creating nodes.
A second launch is rejected if the lock is held or any managed navigation node
already exists. The independent Lite localization adapter may remain running;
it is reused rather than duplicated.

If the graph contains duplicate N-01 nodes, stop the prior
`ros2 launch n01_live_navigation live_scan.launch.py` parent process before
starting another stack. Do not stop Lite SLAM, Livox, IMU, or the independent
adapter as part of navigation-stack cleanup.

## Goal Gate status semantics

`/n01/live_navigation/goal_gate_status` now publishes a 2 Hz heartbeat.
Before an RViz goal arrives, its status is `event=heartbeat`,
`reason=waiting_for_goal`, and `goal_received=false`. This is an idle state,
not a Goal Gate crash.

When a goal arrives on `/move_base_simple/goal`, the gate publishes an
`event=goal_evaluated` result. An accepted goal is forwarded to
`/n01/live_navigation/goal_gated`. Native SCAN then receives that topic as
its remapped goal input.

The acceptance report distinguishes:
- `WAITING_FOR_RVIZ_GOAL`: no goal selected during the sampling window.
- `GOAL_SEEN_BUT_GATE_STATUS_MISSING`: goal input arrived but no gate result.
- `GOAL_REJECTED_OR_NOT_ACCEPTED`: the gate rejected or did not accept the goal.
- `GATE_ACCEPTED_BUT_NO_FORWARDED_GOAL_OBSERVED`: accepted event observed,
  but no forwarded goal was sampled.
- `GOAL_FORWARDED_WAITING_FOR_NATIVE_SCAN_BSPLINE`: planner did not emit a
  candidate during the observation window.
- `NATIVE_SCAN_BSPLINE_RECEIVED`: at least one native SCAN candidate arrived.

## One-command workflow

From the repository on `dev/n01-current`:

```bash
cd ~/lyy/hivecore-navigation
git pull --ff-only origin dev/n01-current
bash scripts/run_n01_planning_acceptance.sh
```

The script:
1. Verifies the Lite map path against the installed Lite localization config.
2. Stops only prior `ros2 launch n01_live_navigation live_scan.launch.py`
   parent processes; it does not stop SLAM, Livox, IMU, or an independent adapter.
3. Refuses to proceed if orphaned managed navigation nodes remain.
4. Builds and tests the live navigation, SCAN planner, and static-map packages.
5. Starts one protected live stack.
6. Samples the goal pipeline for 30 seconds. During this window the operator
   must select one short, visibly clear target with RViz `2D Goal Pose`.
7. Saves build, launch, acceptance JSON, and combined logs below
   `/home/demo/lyy/n01_runtime/logs` (and reports under
   `/home/demo/lyy/n01_runtime/runs/planning_acceptance`).

## Acceptance interpretation

A complete planning-chain pass requires:
- exactly one instance of each managed navigation node;
- `navigation_ready_topic=true`;
- at least one RViz goal input;
- the goal gate accepted and forwarded the goal;
- at least one `/planning/bspline` message;
- candidate review ready.

If the goal was not selected during the 30-second window, the report is
`INCOMPLETE`, not a planner failure. If B-spline output exists but candidate
review is not ready, planning has produced a candidate but the separate Hive2
review remains pending.

Actuation remains disabled in every outcome:
`actuator_output=DISABLED`, `execution_authorized=false`,
`safety_authorization=NEVER`.
