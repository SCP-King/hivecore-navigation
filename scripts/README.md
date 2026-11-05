# N-01 workflow commands and script policy

The stable operator-facing entrypoint is:

```bash
cd /home/demo/lyy/hivecore-navigation
bash scripts/n01 help
```

It dispatches to focused implementation scripts without sourcing vendor SLAM or motion overlays itself. Each implementation keeps its own preflight, ROS environment, process ownership, and safety checks.

## Common commands

```bash
# Optional FAST-LIO2 source/build/replay
bash scripts/n01 fast-lio-import
bash scripts/n01 fast-lio-build
bash scripts/n01 fast-lio-bag /path/to/rosbag2 [output.pcd]

# Offline validation (isolated ROS domain 88)
bash scripts/n01 g1-smoke
bash scripts/n01 g1-acceptance
bash scripts/n01 bag-functional
bash scripts/n01 offline-framework
bash scripts/n01 g4-research

# Demo
bash scripts/n01 demo-replay
bash scripts/n01 record-demo /path/to/output.mp4 [RVIZ_PID]
```

## G1 smoke versus acceptance

Run only on an Ubuntu 22.04 / ROS 2 Humble development computer physically isolated from the robot chassis, real SLAM, and control SDK.

- `g1-smoke` maps to `run_g1_offline.sh` and is a short smoke test.
- `g1-acceptance` maps to `run_g1_acceptance.sh` and runs the full multi-scenario acceptance suite.

Both use ROS_DOMAIN_ID=88 and localhost-only isolation. Their cleanup uses the per-run `N01_G1_RUN_TOKEN` process marker rather than system-wide `pkill` or `killall`.

A cleanup result of `CLEAN` means the runner observed no remaining identifiable processes from that run and no remaining nodes in its isolated ROS graph. It is not an absolute guarantee against SIGKILL, power loss, or deliberately detached processes.

Logs and archives are stored outside the source tree under:

`/home/demo/lyy/n01_runtime/runs/`

The runner prints the exact archive path to upload. Upload that archive for diagnosis; do not paste only the final summary.

## Script inventory

Root-level scripts are distinct workflows, not aliases. Use the command router for stable names; keep direct script paths for compatibility with existing notes and shortcuts.

| Command | Implementation | Purpose |
|---|---|---|
| `fast-lio-import` | `import_n01_fast_lio2.sh` | Import pinned upstream source |
| `fast-lio-build` | `build_n01_fast_lio2.sh` | Build optional FAST-LIO2 backend |
| `fast-lio-bag` | `run_n01_fast_lio2_bag.sh` | Isolated FAST-LIO2 Bag mapping |
| `g1-smoke` | `run_g1_offline.sh` | Short offline smoke |
| `g1-acceptance` | `run_g1_acceptance.sh` | Full offline acceptance |
| `bag-functional` | `run_n01_bag_functional.sh` | Historical Bag functional replay |
| `offline-framework` | `run_n01_offline_framework.sh` | Isolated integration pipeline |
| `g4-research` | `run_n01_g4_full_chain.sh` | G4 historical-map research |
| `demo-replay` | `run_n01_demo_replay.sh` | Read-only RViz demo/export |
| `record-demo` | `record_n01_demo.sh` | RViz-window-only video capture |

Package-local research scripts remain under their owning package. See [repository structure and script policy](../docs/n01_repository_structure_and_script_policy.md) for the package boundary, naming rules, and safe deletion criteria.

## Safety

Offline tests do not authorize real robot movement. Do not run the offline replay or simulated control workflows on a host connected to the real robot network unless the specific procedure explicitly authorizes it. No script in this directory is a substitute for physical isolation, an independent emergency stop, or formal motion acceptance.
