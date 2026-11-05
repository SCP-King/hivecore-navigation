# N-01 repository structure and script governance

## 1. Repository boundary

This repository is the **乙方 N-01 navigation and research stack**. It is not a copy or fork of the vendor HiveCore SLAM product. Vendor SLAM, robot description, motion SDK, and hardware launch files remain external dependencies and are not vendored into this repository.

The current optional FAST-LIO2 integration is deliberately isolated:

- `n01_fast_lio` starts the upstream mapping backend only when explicitly selected.
- FAST-LIO outputs stay under `/n01/fast_lio/*`; its private frame is `n01_fast_lio_map`.
- FAST-LIO TF broadcasting is disabled by default.
- `slam_backend.launch.py` defaults to `backend:=vendor`, which starts no replacement SLAM node and preserves the vendor launch/interface.
- `n01_fast_lio_odometry_adapter` validates and forwards odometry only. It does not publish TF, relabel the private map as the vendor map, or implement saved-map localization.
- The N-01 perception/scan/planning packages consume agreed ROS interfaces; they must not modify vendor source, URDF, launch files, or runtime state.

This is an interface-level decoupling contract, not a claim that FAST-LIO currently replaces vendor saved-map localization. The private FAST-LIO map still requires independent alignment/localization work before it can be used as a navigation world frame.

## 2. Package naming

Use the `n01_<domain>` prefix for first-party ROS packages:

| Package | Responsibility |
|---|---|
| `n01_bringup` | N-01 visualization and bringup launch files |
| `n01_fast_lio` | Optional isolated FAST-LIO2 backend and odometry adapter |
| `n01_perception` | Sensor-to-map perception adapter |
| `n01_scan` | Online SCAN map construction |
| `n01_static_map` | Offline/static PCD map provider |
| `n01_map_fusion` | Static/dynamic map fusion and global routing research |
| `n01_offline_integration` | Offline replay, evidence, and integration-only nodes |
| `n01_scan_dynacbf_bridge` | Isolated SCAN reference/validation bridge |
| `scan_planner` | Upstream SCAN-Planner package family; preserve upstream identity |

Do not rename ROS package names, executable names, topics, frames, or launch arguments as part of shell-script cleanup. Those are integration contracts and require a separate migration.

## 3. One stable command entrypoint

Use `bash scripts/n01 <command>` as the human-facing command namespace. The router only dispatches to focused implementation scripts; it does not replace their checks, alter ROS environment variables, or launch anything during `help`.

Examples:

```bash
bash scripts/n01 help
bash scripts/n01 fast-lio-build
bash scripts/n01 fast-lio-bag /path/to/rosbag2
bash scripts/n01 g1-smoke
bash scripts/n01 g1-acceptance
bash scripts/n01 bag-functional
bash scripts/n01 demo-replay
```

The original script paths remain available for existing desktop shortcuts, notes, and automation. Do not delete or rename them until repository-wide references and the user's machine-local shortcuts have been migrated.

## 4. Script inventory and naming policy

Root-level scripts are distinct workflow implementations, not duplicate aliases:

| Existing implementation | Stable command | Purpose |
|---|---|---|
| `import_n01_fast_lio2.sh` | `fast-lio-import` | Pinned upstream source import |
| `build_n01_fast_lio2.sh` | `fast-lio-build` | ROS/Colcon build |
| `run_n01_fast_lio2_bag.sh` | `fast-lio-bag` | Isolated raw Livox Bag mapping |
| `run_g1_offline.sh` | `g1-smoke` | Short isolated smoke test |
| `run_g1_acceptance.sh` | `g1-acceptance` | Full multi-scenario acceptance |
| `run_n01_bag_functional.sh` | `bag-functional` | Historical Bag functional replay |
| `run_n01_offline_framework.sh` | `offline-framework` | Isolated framework integration |
| `run_n01_g4_full_chain.sh` | `g4-research` | G4 historical-map research |
| `run_n01_demo_replay.sh` | `demo-replay` | RViz demo and media export |
| `record_n01_demo.sh` | `record-demo` | RViz-window-only recording |

Package-local scripts stay beside their owning package and are exposed through the router only where they represent a supported workflow:

- `n01_map_fusion/scripts/nav_min_loop.sh`
- `n01_map_fusion/scripts/nav_phase_c.sh`
- `n01_map_fusion/scripts/replay_fusion_a_star.sh`
- `n01_static_map/scripts/replay_nav_chain.sh`

Python probes, validators, report generators, and test fixtures are implementation/support tools; they are not promoted to top-level shell commands unless they form a stable operator workflow.

### Naming rules for future scripts

- Operator-facing root scripts: `<verb>_n01_<workflow>.sh` (for example, `run_n01_...`, `build_n01_...`, `record_n01_...`).
- Package-local scripts: remain under `src/<package>/scripts/` and use a package/workflow-specific name.
- Python tools: `n01_<workflow>_<action>.py`; ROS nodes retain the package's existing executable naming contract.
- Avoid numbered temporary variants, duplicate wrappers, ambiguous names such as `test.sh`, and scripts that silently source a vendor runtime.
- Keep generated Bags, PCDs, logs, media, and reports outside the source tree.

## 5. Safe cleanup policy

No existing workflow script is deleted in this change. Several root runners share process-ownership and ROS graph-probe logic, but they have different execution/acceptance semantics. Consolidating that safety-critical lifecycle code is a separate refactor and must be covered by regression tests before removing duplicated implementations.

Before deleting any script, search all tracked references (README/docs/launch/desktop files), inspect package-local callers, and check machine-local desktop shortcuts. A script is removable only when its behavior is demonstrably covered by a canonical replacement and its callers have been migrated.

## 6. Runtime safety boundary

Offline runners use ROS domain 88 with localhost isolation where documented. This is not a substitute for physical disconnection from the robot network. Do not run offline replay and simulated command workflows on a connected robot host unless the specific procedure explicitly authorizes it. No offline test result authorizes actuator output or autonomous movement.
