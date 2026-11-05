# N-01 Navigation — Localization Backend Contract

## Goal

N-01 Navigation consumes localization as a replaceable data provider. Its planning core must not depend on Hivecore SLAM C++ classes, shared libraries, services, custom message classes, or vendor-specific state-field names.

## Runtime boundary

```text
Hivecore SLAM3D (or another localization backend)
    | vendor status message + standard pose topic
    v
Backend adapter (vendor-specific module)
    | diagnostic_msgs/DiagnosticArray
    v
/n01/localization/status
    |
    v
N-01 live scan bridge -> native SCAN planner
```

The current Hivecore adapter subscribes to /localization_status with the dynamically resolved type hivecore_slam3d/msg/LocalizationStatus. This dependency is intentionally isolated in localization_adapter.py. The bridge and pure navigation core consume only the normalized contract.

## Normalized ROS contract

Topic: /n01/localization/status

Type: diagnostic_msgs/msg/DiagnosticArray

Exactly one DiagnosticStatus named localization is required. Its values contain:

| Key | Meaning | Required for READY |
|---|---|---|
| map_loaded | localization map is loaded | true |
| role | backend role | localization |
| mode | backend mode | tracking |
| pose_valid | valid global pose is being published | true |
| tf_valid | backend's declared TF output is valid | true |
| quality_valid | backend quality/registration acceptance passed | true |
| tracking_valid | tracking state is internally valid | true |
| imu_sync_ok | IMU synchronization diagnostic | reported, not used as an implicit success |
| fitness_mse | optional numeric backend quality metric | diagnostic only |

The DiagnosticArray header stamp must represent the source status observation time, not adapter publication time. Missing keys or stale timestamps fail closed.

## Stable core dependencies

The N-01 bridge consumes /n01/localization/status (normalized DiagnosticArray), /robot_pose (nav_msgs/msg/Odometry, map -> base_footprint), /odom (nav_msgs/msg/Odometry, odom -> base_link), /livox/lidar/geo_filtered (sensor_msgs/msg/PointCloud2, lidar_link), and the body-to-lidar TF extrinsic.

The status protocol is now backend-neutral. Remaining topic/frame/time semantics are current robot integration contracts and can later be moved to a robot/sensor profile without changing SCAN.

## Adding another SLAM backend

Implement a small adapter that publishes the normalized DiagnosticArray contract and standard timestamped Odometry pose. Do not add vendor-specific imports or field names to core.py or live_scan_bridge.py. Keep all backend-specific state interpretation in the adapter. Then run backend-neutral contract tests and live interface conformance tests.

## Safety semantics

- Missing status, missing required key, invalid status stamp, stale status, or any false required state is NOT READY.
- imu_sync_ok remains an independent diagnostic. It is not silently forced true.
- The adapter never publishes goals, velocity, wheel, chassis, or actuator commands.
- This change isolates the status protocol; it does not certify time alignment, map registration, obstacle geometry, or motion safety.

## Current scope and remaining work

This first decoupling increment isolates vendor-specific localization status. Pose and point-cloud topics already use standard ROS message types, but their topic/frame/time semantics remain configured integration contracts. A subsequent increment should extract topic/frame/QoS settings into a named backend/robot profile and add a second fake backend conformance test. The current Hivecore adapter still needs a real ROS 2 runtime build and live verification on the target machine.
