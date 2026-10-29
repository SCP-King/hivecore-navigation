# hivecore-navigation

ROS 2 Humble source-only N-01 navigation research. Apache-2.0.

## Components

- `src/scan_planner/`: SCAN-Planner **real ROS 2** 3D map, DynAStar, B-spline optimization, FSM and `scan_planner_msgs`. Imported from `wuyi2121/SCAN-Planner`, `ros2-community` commit `d0b921c9b05a6d291d144d60882b2e0e88d2c0e0`. See `src/scan_planner/UPSTREAM_SOURCE.txt` and `NOTICE`. Only `scan_planner_node` is built; original control and simulator executables are **not included**.
- `src/n01_scan_dynacbf_bridge/`: test-only SCAN B-spline to `nav_msgs/Path` reference adapter and default-disarmed output validator, **not** real dynamic-avoidance MPC.
- Other `src/n01_*` packages: legacy N-01 baseline; do not count the legacy 2D A* as SCAN.

**Current status:** G2.2 real-Bag functional smoke has passed, G2.3 strict path geometry needs more runtime evidence, and G3 now adds an **unverified offline finite-control MPC-CBF approximation**. There is **no real robot motion authorization**. Kalman/full Hive2 MPC-CBF control is not implemented or safety-certified.

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

**G2.2 修复版实测（2026-10-29 21:05，提交 `ef851fe`）：** 用户 ROS 2 Humble 一键真实 Bag 测试 PASS（44/44 新集成包单测，99 条 map 点云，运动候选帧 16/99，SCAN 有效 B 样条 2/2，路径风险诊断 95 条，其中 1 次可评估，零 /cmd_vel，清理 CLEAN）。对照旧版 87/99 只能说明过滤输出变化，不证明真实误报率；新阶段优先改善路径/目标时序和有标注动态障碍验证。参见 [G2.2 验收记录](docs/n01_g22_functional.md)。

## G2.3 历史路径与动态障碍时间轴融合（待本机验收）

新增 `ReplayPathSynchronizer`：按历史 Bag 扫描时间戳严格配对 `/body_pose` 和动态目标报告，投影当前记录的机器人位置到**仅作诊断的保留 SCAN Path**；用明确假设的行进速度与目标匀速预测计算相对 ETA、可能重叠位置及跨帧最差风险记录。Path 当前时钟 header **不**等同历史 Bag 时戳。只读输出 `/n01/offline/path_risk`，不创建任何底盘控制话题；仍未实现真 MPC-CBF 求解器。按 `bash scripts/run_n01_bag_functional.sh` 一次性验收。见 [G2.3 开发说明](docs/n01_g23_path_timeline.md)。

**G2.3 21:54 验收复盘：** 原版 `bd60a5a` 时间戳精确配对 99 帧，但 SCAN Path 出现前 95 帧，余下 4 帧均超出 1.5m 走廊，实际可计算动态风险帧数 0。原 runner 的 PASS 含“路径存在但投影不可用也被计为复用”的验收假阳性。新修复对路径超界严格拒绝，另外加最多 160 帧的显式 POST-HOC 历史 Path 几何对照，且与同时序 LIVE 风险严格分账；未在用户 Ubuntu 复测。见 [G2.3 复盘与修复](docs/n01_g23_path_timeline.md)。

## G3 已有真实 Bag 全链数据流与有限时域约束优化（2026-10-29 离线功能 PASS）

不急于做 RViz。现把历史药厂过滤点云（已知 928 帧）尽可能全部送入 map/SCAN/动态跟踪/预测/简化 MPC-CBF 求解器；输出离线约束报告和 `/n01/offline/g3_candidate_path`，**绝不输出底盘运动命令**。一键脚本另生成有真值合成运动障碍的求解正向证据，严格区分它与没有真值的药厂 Bag。见 [G3 数据全流程说明](docs/n01_g3_full_data_pipeline.md)。

**G3 用户真 Bag 实测（2026-10-29 22:52，代码 `1a8a36f`）：** 整份 928 条点云已读取，571 条成功进入 map，569 帧按历史时间戳进入 G3 精确配对；61 帧实际执行数值 MPC-CBF 近似求解并生成候选 Path（51 名义 + 10 含未经认证运动候选约束），SCAN 4/4 有效 B 样条，82/82 集成测试 PASS、零 `/cmd_vel`、清理 CLEAN。**这不是 Hive2 真实动态避障认证**；另有 357 条历史 TF 缺失及 P0 未决。完整审计见 [G3 实测记录](docs/n01_g3_full_data_pipeline.md)。

## 真实 Bag 全链 RViz 演示与 MP4 / GIF（待图形桌面验收）

新增 `bash scripts/run_n01_demo_replay.sh`，将现有 G1/G2/G3 离线结果显示为 RViz 历史点云、SCAN 参考、动态目标、名义/优化轨迹，并录制**RViz 单个窗口**的 MP4、GIF 与 PNG。保持零底盘指令；晚到 Path 的事后计算被显著标记 POST-HOC。详见 [演示使用说明](docs/n01_g123_demo_recording.md)。


## G3.1 逐帧证据审计（已开发，待 Ubuntu ROS 2 回放验收）

真实 Bag 功能脚本与 RViz 演示脚本现在会从 G3 节点独立记录每次 **实际数值求解** 的 `g3_frame_evidence.jsonl`，包含原始历史扫描时间、POST-HOC/回放顺序参考标记、名义与候选 XY、最小 h 代理值、离散 CBF 检查/拒绝次数、研究性质的状态标识。心跳不会作为求解帧。退出时独立核验 journal 并生成 `g3_evidence_summary.json`，若文件缺失、不完整或少于 observer 观察的求解次数则本轮不标记 PASS。此证据仍不是动态障碍真值或 Hive2 碰撞安全验收。

这属于对已通过 G3 功能链的**可审计性增强**，没有增加底盘指令、真实 TF 广播和控制权限。纯 Python 新增 5 项单测已在开发环境通过；尚未在用户 Ubuntu 图形桌面重跑，旧 MP4 不能代替本轮运行时证据。运行说明参见 [G1-G3 演示与 G3.1 逐帧审计](docs/n01_g123_demo_recording.md)。

## G3.2 静态 PCD 背景（仅展示，待 Ubuntu 验收）

此前 RViz `/cloud` 是真实 Bag 的逐帧点云，并不是完整的已保存 PCD 整图；`/nav/test/global_path` 也只是 SCAN 的局部参考（旧 Bag 的测试目标约前方 1.5m），G3 数值候选时域约 1.5s，不等于厂区全局 A→B 导航。

新增可选只读 `n01_static_pcd` 与 RViz 静态底图 `/n01/offline/static_pcd_map`。必须先确认用户 PCD 和历史 Bag `map` 是同一坐标系，然后在运行演示脚本前设置 `N01_STATIC_PCD_PATH=/真实路径/地图.pcd` 和 `N01_PCD_MAP_FRAME_ASSERTED=1`。当前支持未压缩 ASCII/BINARY PCD；不支持 `binary_compressed`。不设置路径时仍显示逐帧点云，并明确告知未加载整图。灰色整图仅用于显示，不进入规划器、不发送底盘控制，不能作为地图配准证明或导航安全授权。详细步骤见 [G1-G3 演示文档](docs/n01_g123_demo_recording.md)。


## G4.0 PCD 栅格与目标点路线预览（已开发、待真实 PCD 验收）

新增 `scripts/n01_pcd_goal_preview.py` 和纯 Python 模块 `pcd_goal_grid.py`，将可读的整图 PCD 转为带未知/自由/占据区分的 2D PGM/YAML 栅格并保存 SVG。必须先确认地面高度 `--floor-z`，未提供时只显示 PCD 高程直方统计；可在提供 `--start X Y --goal X Y` 后生成**独立 A* 全图参考折线预览**。默认将未知区域视为不可通行，若显式允许穿越未知区域，结果只供研究可视化。

**本阶段全图 A* 不是 SCAN 输出。** 原生 SCAN 支持 `fsm.navi_mode=3` 接收 `initial_path`，但尚未将这份整图参考路径与静态占据地图接入其局部 B 样条优化，也未真实 PCD/起终点验证。项目此前在真实 Bag 上通过的 SCAN 是局部轨迹规划而非全厂 A→B 导航。开发/验收要求见 [G4 说明](docs/n01_g4_pcd_goal_planning.md)。严格保持离线、无底盘输出。

另外已修复 G3.1 `Path` 名称冲突（ROS Path 覆盖 pathlib.Path）造成的启动崩溃；须在用户 ROS 2 Humble 环境复测。
