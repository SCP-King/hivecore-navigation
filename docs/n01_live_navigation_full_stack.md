# N-01 live localization, mapping, perception and SCAN integration

## Scope and six-module delivery

This branch keeps the vendor Hivecore SLAM process as the localization authority. It does not patch or replace the vendor ICP/IMU implementation.

1. **Localization adapter** — consumes `/localization_status`, filtered Livox clouds and scan-time TF; publishes exact-stamp map-frame `/cloud`, `/sensor_pose` and `/body_pose`. A bounded FIFO waits for the scan-time transform and drops expired scans.
2. **Map system** — `n01_static_map` reads the operator-supplied binary XYZ PCD as a read-only source and publishes one retained `/grid_map/static_occupancy` sample with transient-local QoS; native SCAN retains the static overlay independently from dynamic raycasting and restores it when the sliding window moves.
3. **Obstacle system** — native SCAN receives the unfiltered map-frame live cloud as its authoritative dynamic occupancy/raycast input. A separate visualization-only tracker compares live returns with coarse static PCD voxels and publishes unmatched candidates and short-lived cluster tracks. Those candidate topics do not replace or filter the SCAN input.
4. **Path planning** — the repository's native SCAN planner remains the planner. It consumes exact-stamp cloud/pose pairs, maintains the rolling occupancy map, uses the static PCD collision overlay, and accepts goals only through the live readiness gate.
5. **Hive2 robot adapter** — a non-actuating B-spline monitor screens nominal four-wheel steering necessary conditions using the documented hip-pivot proxy geometry and URDF steering limit. It publishes a candidate Path and wheel-angle visualization only. It does not publish `/cmd_vel`, `/cmd_wheel`, `/chassis_controller/rt_command` or any hardware command.
6. **Visualization and runtime acceptance** — RViz displays the static PCD, live scan, native SCAN occupancy/inflation, unmatched obstacle candidates, B-spline candidate path and nominal wheel steering vectors. A runtime supervisor reports bridge, map, body pose, TF and vendor timing status.

## Critical scan/pose synchronization

The bridge stamps `/cloud`, `/sensor_pose` and `/body_pose` with the original LiDAR scan timestamp. Native SCAN's LiDAR GridMap now uses a ROS 2 `message_filters::ExactTime<PointCloud2, Odometry>` synchronizer for `/cloud` + `/sensor_pose`. It no longer combines a cloud callback with whichever sensor pose happened to arrive most recently. The bridge and native SCAN therefore agree on the same scan timestamp contract.

The TF retry queue is bounded:
- TF retry period: 20 ms
- maximum wait: 450 ms
- queue capacity: 8 scans
- maximum cloud age: 600 ms
- latest TF substitution: prohibited
- timestamp rewriting: prohibited

The observed vendor `/mapping_processing_timings` is a `geometry_msgs/msg/Vector3Stamped`. The supervisor now subscribes to it and records raw x/y/z values and timestamp age. It intentionally does not assign units or meanings to those axes without the vendor's field semantics.

## Dynamic obstacle interpretation

The planner's authoritative dynamic obstacle input is still the complete `/cloud` stream, consumed by native SCAN raycasting. The extra tracker is for visualization and evidence: it reports live returns not found in the coarse static PCD voxel set, clusters them in XY and associates short-lived IDs. This is not a semantic classifier; map misregistration, sparse PCD sampling, glass, occlusion and moving objects against static surfaces can produce false candidates or missed candidates. Do not use its JSON or markers as a safety sensor or as the only collision layer.

## Hive2 model boundary

The four wheel steering screen uses the documented nominal hip-pivot span (0.56 m x 0.27 m), nominal wheel radius 0.095 m, and URDF ankle-yaw limit +/-1.5708 rad. The 0.56/0.27 m values are not measured tire-contact wheelbase/track. The wheel allocation implementation in `motion_controller/ChassisController`, effective runtime controller parameters, fixed-pitch joint feedback, tire contact geometry, whole-body collision envelope and braking distance are not verified here. The monitor therefore reports necessary-condition screening only and always keeps:
- `controller_model_verified=false`
- `wheel_contact_geometry_verified=false`
- `collision_envelope_verified=false`
- `pitch_hold_feedback_verified=false`
- `execution_authorized=false`
- `actuator_output=DISABLED`
- `safety_authorization=NEVER`

This is a completed non-actuating planning/visualization adapter, not a certified or activated Hive2 motion controller.

## Build and launch

Stop only the previous `live_scan.launch.py` instance with Ctrl+C. Keep the vendor SLAM, Livox, IMU, robot-state and TF publishers running.

```bash
cd /home/demo/lyy/hivecore-navigation
source /opt/ros/humble/setup.bash
git fetch origin feat/live-localization-scan-bridge
git pull --ff-only
colcon build --packages-up-to n01_live_navigation scan_planner n01_static_map n01_scan_dynacbf_bridge --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select n01_live_navigation
colcon test-result --verbose
```

Launch the same PCD/map integration with RViz enabled:

```bash
ros2 launch n01_live_navigation live_scan.launch.py \
  pcd_path:=/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd \
  pcd_map_frame_asserted:=true \
  ground_z_map_m:=-0.55 \
  body_height_m:=0.90 \
  rviz:=true
```

`pcd_map_frame_asserted:=true` remains an operator assertion that this PCD belongs to the active vendor map frame. It is not automatic registration. If that assumption is not true, do not start the static overlay.

## Runtime topics

- `/n01/live_navigation/readiness` — scan bridge JSON counters and age.
- `/n01/live_navigation/navigation_ready` — fail-closed goal readiness Bool.
- `/n01/live_navigation/system_health` — combined runtime health JSON.
- `/n01/live_navigation/dynamic_obstacle_candidates` — unmatched point cloud, visualization only.
- `/n01/live_navigation/dynamic_obstacle_tracks` — candidate cluster markers.
- `/n01/live_navigation/dynamic_obstacle_status` — tracker diagnostics.
- `/n01/live_navigation/hive2_candidate_path` — sampled SCAN B-spline Path.
- `/n01/live_navigation/hive2_wheel_kinematics` — nominal wheel steering arrows.
- `/n01/live_navigation/hive2_feasibility` — non-actuating necessary-condition JSON.
- `/n01/live_navigation/goal_gate_status` — accepted/rejected RViz goal reason.
- `/mapping_processing_timings` — raw vendor Vector3Stamped, captured by the supervisor.

## Acceptance boundary

Repository changes and pure-Python contract tests do not prove a successful ROS 2 Humble C++ build, live DDS synchronization, PCD registration, collision-free route, actual dynamic obstacle avoidance, controller feasibility or physical stopping. The local machine must run the build and real-sensor validation. Until the effective Hive2 controller model and physical collision/stopping envelope are verified, this launch remains planning-only and no actuator publisher is started.
