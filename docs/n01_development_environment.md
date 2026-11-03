# N-01 development environment baseline

This file records environment facts observed in project terminal transcripts and the conventions used by repository scripts. It is not a substitute for a fresh host report.

## Observed host and workspace

- Shell prompt/user: `demo@demo-NuoDaJia-PC`
- Workspace: `/home/demo/lyy/hivecore-navigation`
- ROS distribution: ROS 2 Humble
- Workspace is built with colcon and contains C++ SCAN packages and Python N-01 ROS 2 packages.
- Static map directory: `/home/demo/lyy/slam_data/maps/`
- Runtime/evidence directory: `/home/demo/lyy/n01_runtime/`
- Main Git remote: `https://github.com/SCP-King/hivecore-navigation`
- Active development branch in the current task: `fix/n01-pose-freshness-watchdogs`

The Ubuntu release, kernel, CPU, RAM, GPU, driver versions, installed ROS package versions, and current runtime graph have not yet been verified from a fresh host report.

## Environment conventions

1. Use Bash explicitly: `bash scripts/<entrypoint>.sh`.
2. ROS Humble is expected at `/opt/ros/humble/setup.bash`.
3. Source ROS and colcon setup files through `scripts/lib/ros_env.sh` and `n01_source_setup`. This temporarily disables Bash nounset only during `source`, preserves the caller's original nounset setting, and preserves the source exit code.
4. Do not run ROS/colcon setup files while `set -u` is active without the compatibility loader.
5. Do not ask the operator to manually source ROS before running a repository entrypoint; entrypoints should load and validate their own environment.
6. Do not silently change ROS domain, RMW implementation, or localhost isolation. Live deployment currently defaults to domain 55; isolated Bag/offline workflows deliberately use domain 88 and localhost-only mode. These are separate workflows, not interchangeable defaults.
7. Preserve the installed Hivecore SLAM3D and Hive2/Livox interfaces. Do not replace vendor packages or start a second vendor SLAM stack.
8. N-01 planning validation is planning-only. Never publish chassis, wheel, velocity, or actuator commands, and never change operator hold or execution authorization.
9. Distinguish static CI, host compilation, ROS graph/runtime tests, Bag replay, and physical robot tests. Passing one level does not imply the next level passed.
10. Every operator-facing diagnostic should be a single repository script, save a timestamped report under the runtime directory, and state clearly whether it starts/stops processes or sends ROS messages.

## One-shot host adaptation report

Run:

```bash
cd /home/demo/lyy/hivecore-navigation
git pull --ff-only origin fix/n01-pose-freshness-watchdogs
bash scripts/n01_environment_preflight.sh
```

The report is written to `~/lyy/n01_runtime/runs/environment_preflight/`. It records OS/toolchain details, ROS environment and package prefixes, expected map/Bag paths, and Git state. It does not query the ROS graph, start a node, send a goal, call a service/action, write parameters, or touch actuators.

## Deployment evidence policy

A deployment is not considered successful merely because GitHub accepted a commit or CI passed. The completion record must separately report:

- Git commit and CI result
- host preflight result
- colcon build result
- package test result
- process restart scope and process health
- ROS topic/pose freshness and frame checks
- planning smoke result
- safety state and explicit confirmation that actuator commands were not published
