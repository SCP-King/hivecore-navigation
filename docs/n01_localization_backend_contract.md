# N-01 Localization Backend Contract

## Architecture rule
N-01 navigation core MUST NOT depend on a SLAM vendor, package, message definition, launch file, process, GUI, or private configuration. Only the localization adapter may know an external localization wire protocol.

The customer runs HiveCore SLAM3D Lite as an independent ROS 2 process. N-01 does not start, stop, link against, or vendor that process.

## Verified Lite wire interface (target machine, 2026-11-05)
`/loc_status` type: `hivecore_slam3d_lite_localization/msg/LocStatus`.

Fields confirmed by `ros2 interface show` and the supplied sample:
- `header` (`std_msgs/Header`), with source timestamp and frame ID
- `session` string: `idle`, `relocalization`, or `tracking`
- `map_loaded` bool
- `reloc_ok` bool
- `reloc_fitness` float64

The Lite interface description states `reloc_ok` means the solver converged and fitness passed the configured `reloc_fitness_score_threshold`; this result is latched into Tracking until another Relocalization attempt or deactivation. N-01 does not invent another fitness threshold. It requires `reloc_ok=true` and checks `reloc_fitness` is finite and non-negative only as a data-integrity check.

`/robot_pose` type: `nav_msgs/msg/Odometry`. Observed frame IDs: `map` -> `base_footprint`. Publisher: `localization_lite_node`. The supplied frequency sample varied around 8.7-9 Hz and showed a maximum inter-message interval up to 0.440 s in that run. This is one observation, not a guaranteed rate or latency bound.

## Data flow
```text
External SLAM3D Lite (independently managed)
  /loc_status : LocStatus
  /robot_pose : nav_msgs/Odometry
          |
          v
N-01 Localization Adapter (only vendor-specific boundary)
  - discovers status ROS type at runtime
  - maps verified LocStatus fields explicitly
  - checks nonzero source stamp and configured pose frames
  - preserves original Odometry payload and timestamp
          |
          +--> /n01/localization/status : DiagnosticArray
          +--> /n01/localization/pose   : nav_msgs/Odometry
                         |
                         v
N-01 Live Scan Bridge / Offline Bridge / Navigation Core
```

Navigation bridges consume only normalized N-01 topics. No navigation algorithm imports Lite-generated messages.

## Normalized status keys and readiness
The adapter emits a DiagnosticStatus named `localization` with:
- `session`: lowercase source session string
- `map_loaded`: strict boolean string
- `reloc_ok`: strict boolean string
- `reloc_fitness`: finite numeric string
- `backend_status_type`: discovered source type
- `status_mapping_valid`, `status_mapping_errors`
- `source_stamp_valid`, `readiness_semantics`

Readiness is true only when source stamp is nonzero, mapping succeeds, `map_loaded=true`, `session=tracking`, and `reloc_ok=true`. Consumers also enforce receive-freshness timeouts.

LocStatus does NOT expose independent `tf_valid`, `imu_sync_ok`, `pose_valid`, or general tracking-quality fields. N-01 must not synthesize these as true. Pose frame and stamp checks are performed on the actual Odometry stream. TF availability, sensor synchronization, and other health checks remain separate N-01 runtime checks.

The supplied sample reported `session=relocalization`, `map_loaded=true`, `reloc_ok=false`, `reloc_fitness=0.2332805536`; this is correctly NOT READY.

## Adapter profile
`src/n01_live_navigation/config/slam3d_lite_adapter.yaml` maps only verified fields: `session`, `map_loaded`, `reloc_ok`, and `reloc_fitness`. External topics are confined to this profile. Internal topics remain `/n01/localization/status` and `/n01/localization/pose`.

## Backend replacement
A new backend requires only a new adapter/profile that emits the same normalized DiagnosticArray and Odometry contracts. Do not modify SCAN, map fusion, dynamic-obstacle tracking, trajectory monitoring, or navigation core to consume vendor-specific messages.

## Safety
This integration only forwards localization diagnostics and Odometry. No goal, velocity, wheel, chassis, or actuator command is emitted. Navigation readiness is not execution authorization; current deployment remains planning-only.
