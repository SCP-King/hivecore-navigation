# hivecore-navigation

ROS 2 Humble source-only N-01 navigation research. Apache-2.0.

## Components

- `src/scan_planner/`: SCAN-Planner **real ROS 2** 3D map, DynAStar, B-spline optimization, FSM and `scan_planner_msgs`. Imported from `wuyi2121/SCAN-Planner`, `ros2-community` commit `d0b921c9b05a6d291d144d60882b2e0e88d2c0e0`. See `src/scan_planner/UPSTREAM_SOURCE.txt` and `NOTICE`. Only `scan_planner_node` is built; original control and simulator executables are **not included**.
- `src/n01_scan_dynacbf_bridge/`: test-only SCAN B-spline to `nav_msgs/Path` reference adapter and default-disarmed output validator, **not** real dynamic-avoidance MPC.
- Other `src/n01_*` packages: legacy N-01 baseline; do not count the legacy 2D A* as SCAN.

**Status: source migrated, NOT yet verified with this main-branch code on your Ubuntu/ROS 2 machine. NOT ready for a real robot.** No DynaCBF detection/Kalman/MPC-CBF integration yet.

## G1 build

```bash
cd /home/demo/lyy/hivecore-navigation
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s src/n01_scan_dynacbf_bridge/test -v
colcon build --packages-up-to scan_planner n01_scan_dynacbf_bridge --event-handlers console_direct+
source install/setup.bash
```

## G1 isolated smoke test

Set these in **each** terminal, without loading any real robot or SLAM overlays:

```bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export ROS_HOME=/home/demo/lyy/n01_runtime/ros_home/g1_scan
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
source /home/demo/lyy/hivecore-navigation/install/setup.bash
```

- Terminal A: `ros2 launch scan_planner n01_isolated.launch.py` (real SCAN algorithms, no controller)
- Terminal B: `ros2 run n01_scan_dynacbf_bridge scan_g1_feed` (deterministic offline pointcloud/poses/goal; no TF or cmd_vel)
- Terminal C: `ros2 run n01_scan_dynacbf_bridge scan_g1_monitor` (diagnostic observer, max 90 seconds or 8 seconds after first valid spline; saves `$ROS_HOME/g1_smoke_report.json`)
- Verify: `ros2 topic info /cmd_vel -v`; no real command publisher allowed. When all tests are stopped, report results.

**Never run `bridge_smoke_feed` alongside `scan_g1_feed` or real SCAN**, because the former publishes synthetic `/planning/bspline`. This G1 smoke test checks live message observations only; trajectory collision, actual MPC, robot-model compatibility, 500 ms requirement and long-duration tests remain NOT TESTED.

Keep PCD, Bag, manuals, offline test outputs and reports outside this source-only repository.

## G1 diagnostic note

If the planner prints `final_plan_success=1` but the observer reports `bspline_messages=0`, do **not** mark G1 as PASS. The diagnostic monitor records publisher discovery, topic types/QoS, received `/cloud`, `/body_pose`, `/move_base_simple/goal`, and presence of real `/cmd_vel` publishers to distinguish missing input, DDS endpoint mismatch and observation-window timing. The test feed repeats the same stationary offline goal at 4, 17, 32 and 52 seconds. A stationary synthetic robot is not evidence of physical motion or obstacle-free trajectory execution.

## Hive2 / SLAM / SCAN 参数基线（2026-10-29）

- [Hive2 参数审计与未决冲突（中文）](docs/hive2_parameter_inventory_20261029.md)
- [机器可读 Hive2 参数清单（JSON）](config/reference/hive2_parameter_inventory_20261029.json)

参数清单记录 Hive2 运动栈与 SLAM 的已报告尺寸、Livox MID-360 外参、点云/TF 接口、速度需求，以及**用户已选定的 Hive2 当前高度 0.90 m**、最新 SLAM release 中**仍默认开启**的 0.30 m mount RSP（须显式关闭）、仍需核验的 TF 唯一发布者和药厂安全距离。**只读参考，不是 ROS 可执行配置**。本仓库的 G1 `planner.yaml` 仍然使用独立的合成测试几何值；不能将其当作 Hive2 真机参数。当前高度基准虽确定，但最新 SLAM 的双发布风险仍存在，运行态未验证，故不自动修改启动配置。

**P0 接口与只读预检规格：** [Hive2 SLAM / SCAN P0 Readiness Preflight](docs/hive2_slam_p0_readiness_preflight.md)，涵盖显式禁用 SLAM mount RSP、`/localization_status` tracking/配准判据和 `lidar_link→map` 点云采集时刻转换。**尚未执行、不作为真机运行许可。**

## N-01 离线整体框架（最新）

**一键验证：** `bash scripts/run_n01_offline_framework.sh`，自动启动隔离 TF/点云 fixture、严格输入桥接、真实 SCAN、B 样条参考路径与禁用的动态避障接口，并清理打包。可选 `N01_BAG_PATH=/path/to/bag` 对已录制 Bag 做只读清单；本轮**不会自动回放可能含运动命令的 Bag**。

详细边界、流程、验收状态与下一阶段步骤：[N-01 离线集成框架](docs/n01_offline_framework.md)。

**2026-10-29 第三轮离线合成框架验收：PASS**（提交 `0af1e75`，10/10 单测、375 条 map 点云、4/4 条有效 SCAN B 样条、4 条参考路径、零实机指令、清理 CLEAN）。这**不包括真实 Bag 回放、完整 G1 碰撞验收或 DynaCBF/MPC-CBF 求解器**；第三轮的原始测试日志由用户单独保存。

## Real recorded Bag functional replay (offline only)

**Next stage:** read-only whitelist of recorded Livox cloud + TF, reconstruct `lidar_link→map` using recorded map poses and selected Hive2 mounts, then run actual SCAN + reference-Path pipeline. Historical `0.30/0.90 m` conflicting TF edges are not broadcast. No actuator or raw bag playback, no false Localization Tracking claim.

Run `bash scripts/run_n01_bag_functional.sh` after pulling main. See [真实 Bag 功能回放说明](docs/n01_real_bag_functional.md) for one-command test and precise limitations. Synthetic fixture smoke baseline remains unchanged.

**2026-10-29 实际 Bag 开环功能冒烟：PASS**（提交 `91eb96c`，读取过滤点云 180 条、map 转换 99 条、有效 SCAN B 样条 2/2、Path 1 条，零 /cmd_vel，清理 CLEAN）。其余 **81 条因扫描时刻 map TF 不可用被拒绝**；历史 0.30/0.90 m 冲突仍记录为风险。本结果不代表完整碰撞验收或真机定位/动态 MPC-CBF 已实现。

## G2 离线动态障碍功能阶段（首次功能冒烟已通过）

从真实 Bag 重建后的 map 点云输入新增动态聚类、跨帧关联与匀速短时预测，输出 `/n01/offline/dynamic_tracks`，供禁用控制的动态接口读取。**Kalman/MPC-CBF 求解器和真机控制仍未实现；不能视为真实动态障碍安全验收。** 见 [功能说明](docs/n01_dynamic_tracking_stage.md)。

**G2 首轮实测（2026-10-29 20:36，提交 `3b45c13`）：** 29/29 集成单测通过；真实 Bag 点云 99 帧进入动态模块并处理；100 条动态诊断消息，87/99 帧输出未经验证的运动候选（最大 7 个/帧）；SCAN 有效 B 样条 2/2，Path 1 条，/cmd_vel 0，清理 CLEAN。**高候选发生率及最大约 5.94m 的聚类意味着潜在静态结构误报风险**，不能作为真实动态障碍检测精度或 MPC-CBF 安全通过的证据。

## G2.2 离线静态背景过滤和轨迹风险预览（待验收）

在已运行的真实 Bag→SCAN→动态候选链上增加多帧静态栅格过滤、过大聚类排除、运动方向一致性，以及**只读路径近距离风险预览** `/n01/offline/path_risk`。该模块不求解 MPC-CBF，不对真机发任何控制命令，不将“无风险候选”视为安全。沿用 `bash scripts/run_n01_bag_functional.sh` 验证，见 [G2.2 开发与验收范围](docs/n01_g22_functional.md)。
