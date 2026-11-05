# N-01 Planning Readiness Audit

## Purpose

This is the one-shot, read-only evidence collection for moving from a running live-localization stack to the next native SCAN planning test.

It deliberately checks the complete chain rather than only `/robot_pose`:

1. ROS environment, active workspace revision, running processes and duplicate nodes.
2. SLAM status, global pose, odometry, IMU and LiDAR input timing.
3. Live bridge outputs: `/cloud`, `/sensor_pose`, `/body_pose` and scan-time body pose, including sampled XYZ bounds and exact timestamp comparisons.
4. TF frame chain and LiDAR mounting transform. The current observed contract is `map -> base_footprint -> base_link -> lidar_link -> imu_link -> livox_frame`; `/odom` is a separate Odometry topic and does not imply an `odom` TF frame.
5. Original PCD parser compatibility, source bounds, retained static occupancy cloud, robot pose versus active rolling-window geometry and the live SCAN overlay.
6. Native SCAN subscriptions, planner parameters, goal-gate path and B-spline output.
7. Dynamic-scene diagnostics and Hive2 candidate feasibility.
8. Operator hold, safety status and actuator-topic publisher topology.

## Run

From the checked-out repository:

```bash
cd ~/lyy/hivecore-navigation
git pull --ff-only origin dev/n01-current
bash scripts/n01_collect_planning_readiness_audit.sh
```

All planning-readiness collection is maintained in this repository. Do not copy the script to `/tmp`; its Python probes are resolved relative to the checked-out workspace.

## Outputs

The audit creates:

- `~/lyy/n01_runtime/runs/planning_readiness_audit/n01_planning_readiness_audit_<timestamp>.log`
- `~/lyy/n01_runtime/runs/planning_readiness_audit/n01_live_stack_preflight_<timestamp>.json`
- `~/lyy/n01_runtime/runs/planning_readiness_audit/n01_planning_ros_probe_<timestamp>.json`
- `~/lyy/n01_runtime/runs/planning_readiness_audit/n01_planning_readiness_bundle_<timestamp>.tar.gz`

The compressed bundle contains the log, publisher-attributed live-stack preflight JSON, XYZ/timestamp probe JSON, and generated TF graph files when available. It is the preferred artifact to attach to the next project turn.

## Hivecore SLAM GUI cross-check

The uploaded Hivecore SLAM Services GUI manual is used as an operational reference, not as the authority for current node/service names. The live ROS graph and the installed vendor release remain authoritative because the manual is an older revision.

The GUI is a service client and external-tool launcher; it does not itself run the SLAM algorithm and does not publish pose or cloud topics. The audit therefore checks the backend nodes, service servers, state topics, and GUI process separately.

Operational state transitions that matter to navigation:

- **Mapping a new map:** Mapping Session Start -> drive/collect -> Save to disk. Mapping Start may stop localization and clear the previous local map/session.
- **Using a saved map through mapping relocalization:** Stop -> Load from disk -> Relocalization -> provide `/initialpose` -> wait for `reloc_ok` -> Resume. Do not press Start after loading an overlay if the intent is to continue/relocalize against that prior map.
- **Localization session:** Load the localization map -> Session Start -> enter tracking or relocalization as appropriate. The status topic, not the button highlight alone, is the state evidence.
- **Preprocessing:** verify Cloud forward is enabled and inspect the current SetGeoFilter configuration. Disabling forwarding mutes `/livox/lidar/geo_filtered`; it is not a harmless display toggle.
- **Time contract:** live Livox/SLAM should use system time. RViz, bag replay, and the rest of the active desktop stack must not accidentally mix simulated and system time.
- **Mutual exclusion:** mapping and localization must not simultaneously own `/robot_pose` and the map-to-robot TF path. Multiple publisher endpoints on `/robot_pose` are not automatically proof of two active sessions, but the active session state and message stream must be checked.

`/initialpose` is a localization/relocalization seed. It is not the navigation destination. A navigation goal is a separate input to the N-01 Goal Gate and must not be confused with the GUI's relocalization workflow.

Older manual service names may use `lidar_3d_mapping_node` or a GUI node named `slam_services_gui`; the observed release may expose `/slam3d_node/...` and `/slam3d_services_gui`. The audit checks the current graph's actual services and does not call them.

## Planning stages

### P0 — Inputs required to attempt native SCAN planning

- Fresh SLAM tracking status, active localization processing and advancing global pose timestamps.
- Valid local odometry and scan-time pose/odometry support.
- Valid `map -> base_footprint -> base_link -> lidar_link` chain and sensor extrinsics.
- Fresh map-frame `/cloud` and scan-stamped `/sensor_pose`.
- Fresh map-frame `/body_pose` consumed by native SCAN.
- Static PCD loaded, map-frame asserted, and source bounds intersecting the active SCAN window.
- Native planner node present with the expected subscriptions and a healthy goal-gate snapshot.

### P1 — Candidate trajectory review

- A B-spline has been produced after a goal is supplied.
- The B-spline has valid order, knots and finite control points.
- Hive2 candidate feasibility has current body-pose and joint feedback.
- Dynamic-scene diagnostics are current.

The dynamic-obstacle tracker is intentionally visualization/diagnostics only. Native SCAN consumes the transformed map-frame `/cloud` directly for live occupancy updates, together with the separate static PCD overlay. The tracker output must not be mistaken for a direct planner input.

A missing `/planning/bspline` before a valid goal is sent is not, by itself, a planner failure. The audit does not invent or publish a goal.

### P2 — Physical execution acceptance

This stage is separate from planning. The audit only reads the safety state and command-topic graph. It never changes operator hold, authorizes motion, or publishes actuator commands. Physical controller mapping, wheel geometry, emergency-stop behavior, braking distance and other acceptance checks remain unverified until separately tested.

## Important interpretation limits

- Standard PCD files do not encode or prove their coordinate registration. The PCD parser check proves file-format compatibility and reports bounds; the audit also compares the SCAN static PCD path with the vendor localization map path when that parameter is available. The active static-overlay status reports whether the source intersects the current SCAN window. Coordinate registration still depends on the operator-confirmed vendor SLAM map-frame contract.
- A direct `map -> base_footprint` TF edge is not required by the current bridge contract because the bridge derives `/body_pose` from timestamped global SLAM anchors plus relative odometry. The `base_footprint -> lidar_link` extrinsic and `odom -> base_link` motion chain are important.
- The dynamic obstacle tracker reports unmatched returns for visualization and diagnostics. The native SCAN occupancy map receives `/cloud` and the static overlay independently.
- The safety supervisor is a monitor, not an actuator controller. Its fail-closed state is expected during planning-only work.

## Safety contract

The audit:

- subscribes to ROS topics and inspects graph, parameters, files and process metadata;
- reads parameters only;
- does not publish `/move_base_simple/goal`, `/n01/live_navigation/goal_gated`, `/cmd_vel`, `/cmd_wheel`, or chassis commands;
- does not invoke ROS services or actions;
- does not change parameters, stop nodes, restart nodes, launch stacks, build packages or run tests.

## Planning-only SCAN smoke test

The latest live-readiness audit can report READY_FOR_GOAL_PLANNING while
/planning/bspline remains empty: the read-only audit deliberately sends no
goal. After rebuilding and restarting the live SCAN launch, run one bounded
candidate-planning test:

```bash
source /opt/ros/humble/setup.bash
source /home/demo/lyy/hivecore-navigation/install/setup.bash
ros2 run n01_live_navigation n01_planning_smoke_test --distance 0.8 --timeout 15 --confirm-planning-only
```

The one-command deployment and verification entry point is:

```bash
bash scripts/deploy_n01_planning_fixes.sh --confirm-planning-only
```

It builds the changed packages, runs the static-inflation equivalence test,
restarts only the existing live_scan launch (preserving its PCD and launch
geometry arguments), and then runs the planning-only smoke test. It leaves
the vendor SLAM process and hardware drivers running.

The test derives a short goal from the current /body_pose, sends one
PoseStamped through the existing /move_base_simple/goal input, and waits for
a new /planning/bspline. It refuses to publish unless system health and the
static overlay are ready, the safety supervisor explicitly reports
actuator_output=DISABLED and safety_authorization=NEVER, and the B-spline
graph contains only the runtime supervisor and candidate monitor as
consumers. It never changes operator hold or publishes actuator commands.

A passing result proves candidate generation only. It does not prove wheel
contact geometry, physical collision clearance, emergency-stop acceptance,
or authorization to move the robot.
