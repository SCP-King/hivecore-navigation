# N-01 Localization Backend Contract

## Architecture rule: navigation is backend-independent

N-01 navigation core MUST NOT depend on a SLAM vendor, package, message definition, launch file, process, GUI, or private configuration. The only module permitted to know an external localization wire protocol is `n01_localization_adapter`.

The current deployment profile is the customer's headless HiveCore SLAM3D Lite. It is an external ROS 2 process managed by the customer's CLI. N-01 neither starts nor stops it, does not link against its libraries, and does not include its source tree.

## Data flow

```text
External SLAM3D Lite (managed out-of-process)
  /loc_status (message type discovered at runtime)
  /robot_pose (nav_msgs/msg/Odometry)
             |
             v
N-01 Localization Adapter (only backend-specific boundary)
  - discovers /loc_status ROS type from ROS graph
  - maps explicitly configured field paths (no guessed fields)
  - validates status stamp and pose frame/stamp
  - republishes standard N-01 interfaces
             |
             +--> /n01/localization/status
             |      diagnostic_msgs/msg/DiagnosticArray
             |
             +--> /n01/localization/pose
                    nav_msgs/msg/Odometry
                         |
                         v
N-01 Live Scan Bridge -> native SCAN -> navigation diagnostics
```

The navigation bridge subscribes only to `/n01/localization/status` and `/n01/localization/pose`; it must never subscribe directly to `/loc_status` or the external `/robot_pose`. Backend-specific topic names are confined to the adapter profile.

## Normalized status contract

The adapter emits one DiagnosticStatus named `localization`, with these stable key/value entries:

| Key | Type/meaning |
|---|---|
| `map_loaded` | strict boolean string |
| `role` | normalized role string; expected `localization` |
| `mode` | normalized tracking-state string; expected `tracking` |
| `pose_valid` | strict boolean string |
| `tf_valid` | strict boolean string |
| `quality_valid` | strict boolean string |
| `tracking_valid` | strict boolean string |
| `imu_sync_ok` | independent diagnostic; never forced true |
| `fitness_mse` | finite numeric string or `nan` |

The DiagnosticArray header stamp is copied from the external status source stamp. Missing/zero source stamp forces NOT READY. Missing field mapping, missing fields, unknown boolean encodings, and unrecognized role/mode values fail closed. No status is inferred from topic existence alone.

## Adapter profile

Profile: `src/n01_live_navigation/config/slam3d_lite_adapter.yaml`.

- External status topic: `/loc_status`.
- External pose topic: `/robot_pose`.
- Internal status topic: `/n01/localization/status`.
- Internal pose topic: `/n01/localization/pose`.
- The external status ROS type is discovered at runtime; no HiveCore generated-message dependency is imported or declared by N-01.
- `status_field_paths_json` intentionally defaults to an empty mapping. It MUST be populated only after the installed Lite interface definition and actual status samples have been inspected. Until then the adapter publishes NOT READY.
- The current pose frame expectations (`map`, `base_footprint`) are an integration profile, not assumptions embedded in navigation algorithms; verify against the actual Lite output before deployment.

## Pose and scan time contract

The adapter forwards the original Odometry message without changing its timestamp, pose, covariance, or frame identifiers, after checking a nonzero stamp and configured parent/child frames. The bridge retains bounded timestamped pose interpolation and exact scan-stamp association. It must not substitute latest pose/TF, rewrite timestamps, or extrapolate without an explicit bounded odometry model.

## Replacing the localization backend

To add or replace a backend:

1. Add a profile describing that backend's external status and pose topics.
2. Implement or configure the adapter mapping for the real message schema.
3. Verify status semantics, source timestamps, QoS, pose frame IDs, and pose freshness.
4. Run the same backend-neutral adapter contract and live interface conformance tests.
5. Change only the adapter profile/adapter implementation and deployment wiring.

Do not modify `core.py`, `live_scan_bridge.py`, SCAN, map fusion, dynamic-obstacle tracking, or planning algorithms to accommodate a backend's private message schema.

## Verification still required on target machine

The Lite message type and its semantic fields have not yet been confirmed in this repository context. Run after sourcing the customer's Lite environment:

```bash
ros2 topic list -t | grep -E '^/(loc_status|robot_pose|livox/lidar/geo_filtered|odom)'
ros2 topic type /loc_status
ros2 interface show "$(ros2 topic type /loc_status)"
ros2 topic info /loc_status --verbose
ros2 topic echo /loc_status --once
ros2 topic type /robot_pose
ros2 topic info /robot_pose --verbose
ros2 topic echo /robot_pose --once
ros2 topic hz /robot_pose
```

Do not guess the field paths. Do not mark the adapter ready until the above evidence is reviewed and the profile is populated with verified paths/values.

## Safety

The adapter only publishes diagnostics and standard Odometry. No goal, velocity, wheel, chassis, or actuator commands are published. `navigation_ready` is not `execution_authorized`; current deployment remains planning-only.
