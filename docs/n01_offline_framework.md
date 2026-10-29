# N-01 药厂 DOG 导航避障：整体离线框架（首版）

**建成范围：离线 ROS 2 集成骨架及一次性自测**；不是 G1 静态避障完整验收，不是 G2 DynaCBF/MPC-CBF 可用算法，也不是 Hive2 真机控制系统。2026-10-29 定义。

## 架构与数据流

```text
[离线输入层]
    +-----------------------+        +-----------------------------+
    | 录制 Bag (只读清单)   |        | 隔离合成 TF/点云/目标发生器 |
    | /tf /tf_static       |        | N01_OFFLINE_FIXTURE=1 only  |
    | /localization_status |        +--------------+--------------+
    | /livox/lidar/geo_*   |                       |
    +-----------+-----------+                      |
                | 未来 strict Bag 重放 (无自动执行) |
                v                                    v
    +----------------------------------------------------------+
    | n01_offline_sensor_bridge                                |
    | 1) freshness/status: tracking + registration TF published|
    | 2) base_footprint->base_link 0.90m/no 0.30m conflicts    |
    | 3) map <- lidar_link transform AT CLOUD TIMESTAMP        |
    | 4) transform XYZ, preserve PointCloud2 other fields     |
    | 5) fail closed on missing TF/status, /cmd_vel publisher  |
    +------------------------------+---------------------------+
                                   | /cloud (frame=map)
                                   | /body_pose + /sensor_pose
                                   v
                     +---------------------------+
                     | REAL SCAN Planner         |
                     | 3D Occupancy + B-spline  |
                     | /planning/bspline         |
                     +-------------+-------------+
                                   |
                     +-------------v-------------+
                     | Existing B-spline bridge  |
                     | /nav/test/global_path     |
                     +-------------+-------------+
                                   |
                     +-------------v-------------+
                     | DynaCBF interface PORT    |
                     | STATUS: NOT_IMPLEMENTED   |
                     | NO MPC/CBF or cmd output  |
                     +---------------------------+
```

所有新增 ROS 节点要求 `ROS_DOMAIN_ID=88` 和 `ROS_LOCALHOST_ONLY=1`。默认离线门控为 **strict**，没有真实 `LocalizationStatus.msg` 字段契约时拒绝数据。fixture 模式只能使用单独的 `/n01/offline/test/localization_ready` 布尔消息，且必须显式导出 `N01_OFFLINE_FIXTURE=1`；**不能将合成布尔状态当作甲方定位成功证明**。

## 一键自测（仅一条命令）

开发机 Ubuntu 22.04 + ROS 2 Humble，**断开真实控制 SDK/底盘，不启动实机 SLAM**。

```bash
cd /home/demo/lyy/hivecore-navigation
git pull --ff-only origin main && bash scripts/run_n01_offline_framework.sh
```

脚本自动：
1. 隔离 Domain 88，检查 ROS 图已有节点与 `/cmd_vel`，出现冲突直接中止；
2. 跑 G1 旧单测、进程守护单测、离线接口/几何单测；
3. `colcon build --packages-up-to scan_planner n01_scan_dynacbf_bridge n01_offline_integration`；
4. 如可发现先前 `bag_20261027_225100` 或指定 `N01_BAG_PATH`，只读检查 Bag `metadata.yaml` 话题类型/数量，另用限额扫描器读取录制 TF 数值、点云 header 和定位状态的已知布尔标记；**这一步不回放 Bag，不证明真实 TF 有效**；
5. 启动测试用 `map→base_footprint→base_link(z=.90)→lidar_link(x=.295)` TF、合成 `lidar_link` 点云、离线状态旗标；
6. 适配层将点云按 **header.stamp** 转换成 `map`，给真实 SCAN 与原有 B-spline path bridge；DynaCBF port 只报告未实现，不生成控制命令；
7. 记录 `/cloud`、`/body_pose`、`/sensor_pose`、膨胀地图、B-spline 的实测计数及桥接状态；
8. 退出时 **只清理本轮进程**、检查 ROS 图是否清空、归档 tar.gz。最终显示 `UPLOAD THIS LOG ARCHIVE:`，把对应压缩包上传给助手。

Bag 已知绝对目录时，同样**一条命令**可加入只读清单：

```bash
cd /home/demo/lyy/hivecore-navigation && git pull --ff-only origin main && N01_BAG_PATH=/实际路径/bag_20261027_225100 bash scripts/run_n01_offline_framework.sh
```

不指定路径时脚本会按原记录名在 `/home/demo/lyy` 和 `/home/demo/latest` 内有界搜索；找不到会明确在 JSON 中写 `NOT_SUPPLIED_OR_NOT_FOUND`，不把缺失 Bag 当成已验收。

日志通常位于 `/home/demo/lyy/n01_runtime/runs/n01_offline_framework/`，包含：

- `summary.txt`：本轮结论、退出码、进程清理和 ROS 图收尾；
- `integration_fixture_report.json`：各输入和真实 SCAN 输出计数、缺失证据；
- `bag_inventory.json`：录制数据中的实际话题和缺项，**只读**；
- `bag_evidence.json`：只读提取 Bag 记录的 `base_footprint→base_link` 高度取值、是否有 0.30/0.90 双值、点云帧名和状态标志；扫描有 55 秒时间上限与 12 万条消息上限，不能从历史记录证明当前运行态唯一 TF 发布者；
- `logs/integration_launch.log`：全部 ROS 节点日志；
- `logs/*unit_tests.log`：各模块单测；
- `logs/*graph.json`：ROS 图与 `/cmd_vel` 查询证据。

## Strict Bag 接口的现有边界

已有 Bag 足以开发和回放真实感知链，**但当前缺少甲方 `LocalizationStatus.msg` 的完整 ROS 包名与 outer/inner 字段实际路径**；不猜测其字段布局。strict bridge 已支持：

- 从 ROS 图动态发现 `/localization_status` 的消息类型，使用 `rosidl_runtime_py.get_message` 订阅；
- 用 `outer_path`、`inner_path`、`tf_flag_path` 和配置期望值检查状态；路径为空或字段不符将 **NOT_READY**；
- `slam_rsp_disabled_confirmed=false` 为默认值。必须有甲方启动覆盖及单一发布者预检证据才有资格变更；绝不因 Bag 中有 `/tf_static` 就判定单一 RSP；
- 拒绝 `lidar_link` 点云缺少 `map` 扫描时刻 TF、z=0.30m 冲突、错误 frame 和过期定位状态；
- 仅发布 `/cloud`、`/body_pose`、`/sensor_pose` 供 SCAN；`/body_pose` 的 twist 为**离线占位零值**，不具备闭环动态速度估计能力，绝不可代替真机 odom。

`src/n01_offline_integration/config/offline_bridge.yaml` 中包含保守默认门控参数。真正的 Bag 回放尚未自动激活：需先检查 metadata，拿到实际 `LocalizationStatus.msg` 契约，并确定/排除 Bag 里的双 RSP 历史数据。下一阶段再实现严格白名单回放及定位状态分支；不允许未经评估就运行无白名单的 `ros2 bag play`（其中可能包含运动指令）。

## 模块状态（不能误报）

| 模块 | 当前状态 | 可以验证什么 |
| --- | --- | --- |
| 多传感器接口与元数据 | 已搭建；录制 Bag 清单需实际读取 | bag 话题/类型/计数 |
| SLAM 定位门控 | 已搭建 strict 字段契约 + 离线 fixture | 失效/缺字段/超时不放行；实际 .msg 尚未接线 |
| TF 高度风险门控 | 已搭建 0.90m 观察与 0.30m 拒绝 | 合成 TF；历史 Bag 的单一发布者仍需核实 |
| 点云坐标转换 | 已搭建 PointCloud2 二进制 XYZ 转换 | 帧、时间戳、tf lookup/异常阻断 |
| SCAN 三维规划 | ROS 2 实现已存在并接入 launch | 真正发布占据地图/B-spline 的观测 |
| B-spline → reference Path | 已复用现有测试桥接 | 轨迹参考输出（不控制底盘） |
| DynaCBF/MPC-CBF | **未实现求解器**；只接入受禁用接口 | 接口存在和不能发送运动指令 |
| 真机 Hive2 控制 | **未接入、不允许** | 无 |
| G1 完整碰撞验收 | **仍未完成** | 历史 B/C 场景需要单独证明 |
| 真实 Bag 全链路回放 | **待消息契约和白名单回放** | 目前只可做只读 Bag 清单 |

## 接下来需要补充（不影响本轮一次性测试）

`/home/demo/latest/slam3d_release` 中的 `LocalizationStatus.msg` **完整原始内容**、已有 Bag 的明确路径、该 Bag 的 `metadata.yaml`，以及甲方 SLAM 当前显式关闭 mount RSP 的启动证据。待这些信息齐备，才能把 strict Bag 回放、实时速度估计和 P0 TF 权威链做成真实数据验收，之后再实现动态障碍检测与 DynaCBF/MPC-CBF 求解器。

## 安全说明

- 新框架不能独立证明 `/tf_static` 上某条边只有单一发布者：TFMessage 不携带发送者身份，仍需结合甲方启动参数和 ROS 图检查，**存在疑点就不放行**。
- 本次通过的结果名最多为 `OFFLINE_INTEGRATION_SMOKE_PASS`，不是 G1/G2/药厂现场验收通过。
- 测试独立 ROS 域，脚本对所有由其创建的进程使用 per-run token 清理；不得 `pkill -f ros` 或干扰其他 ROS 工作域。

## 2026-10-29 首次一键日志复盘（19:20 本地）

首次执行的 **源码编译及全部已有 Python 单测通过、进程与 ROS 图清理 CLEAN**；但 `n01_offline_fixture` 在第一次发送 TF 时因 Humble 严格浮点字段收到 Python int 抛出 `AssertionError`，导致 `/livox/lidar/geo_filtered`、状态和目标均未发布。日志同时报告 B-spline reference bridge 对 `/body_pose` 使用 reliable 订阅，而 sensor publisher best effort，造成 QoS 不兼容。

修复方向：合成 TF 数值全部强制 `float`、加入 ROS 消息类型回归单测、B-spline bridge 的 odom 订阅改为 sensor-data QoS；一键脚本发现核心子节点异常退出立即失败，不再等待观察器直到超时；观察器补充真正 Path reference 和未激活动态端口的证据。**改动未经 Ubuntu/ROS 2 再次实际运行验证，不应声称整链已通过。**

同轮 **真实 Bag 只读审计**（`/home/demo/lyy/slam_data/bags/bag_20261027_225100`）已确认 `/livox/lidar/geo_filtered`=928 条、`/tf`=2731 条、`/tf_static`=12 条、`/robot_pose`=609 条，`/localization_status` **缺失**；限额扫描 120000 条消息显示 `base_footprint→base_link` z **0.30/0.90 m 双值历史冲突**，点云帧 `lidar_link`。因此此 Bag 可作为**历史 TF/点云数据审计依据**，不可直接以严格 LocalizationStatus tracking 判据宣称真机定位就绪。该事实不受合成 fixture 修复影响。

## 2026-10-29 第二次一键日志复盘（19:38 本地）

日志归档 `n01_offline_framework_20261029_193835_951563.tar.gz`；执行 commit `330bb97`。**9 项新包单元测试通过，ROS 2 构建通过，清理进程及 ROS 图均 CLEAN**。最新合成 fixture 已成功发布 TF、`/body_pose`，SCAN 日志记录收到 z=0.900 m 的首次位姿和第一个目标，且 FSM 从 `INIT` 到 `WAIT_TARGET` 再到 `GEN_NEW_TRAJ`。

**新阻断点不是数据缺失：** 观察器 `observer_node.py` 同时 `from pathlib import Path` 和 `from nav_msgs.msg import Odometry, Path`，后一个 `Path` 覆盖文件路径类，导致构造 `Path(target)` 时抛出 `TypeError: Path.__init__() takes 1 positional argument but 2 were given`。脚本检测到 `n01_offline_observer` 异常退出并提前终止，**没有获得 B 样条或碰撞验收结论**。本次修复将两者分别命名 `FilePath` / `RosPath`，加入独立 AST 回归单测以及构建后不启动 ROS 节点的导入预检；**尚未重新在用户的 Humble 系统运行验证**。

相同 Bag 的只读审计继续显示 `/localization_status` 缺失，`/mapping_status` 存在；限额扫描见历史 `base_footprint→base_link` 的 z=0.30 与 0.90 m 两个取值，点云帧为 `lidar_link`。这说明 Bag 可继续做数据流开发，但不能直接证明当前运行态的唯一 RSP 或严格定位就绪。

## 2026-10-29 第三次离线整体框架验收（19:47，本轮 PASS）

**提交版本：** `0af1e75`。**原始证据：** 用户上传的 `n01_offline_framework_20261029_194749_956839.tar.gz`（本仓库不保存实际日志/Bag 大文件）。该记录是已完成的**隔离合成输入集成冒烟验收**，**不是**真实 Bag 时序回放、G1 全路径碰撞验收、G2 MPC-CBF 求解器测试或实机导航。

| 证据项 | 第三轮实际结果 |
| --- | --- |
| 一键运行摘要 | `OFFLINE_INTEGRATION_SMOKE_PASS`；退出码 0 |
| 离线接口单测 | 10 / 10 PASS |
| ROS 2 `colcon` 编译和观察器导入预检 | PASS |
| 合成 TF | `map→base_footprint→base_link(z=0.90)→lidar_link` 已驱动测试输入；并非在线 RSP 唯一性证明 |
| `/livox/lidar/geo_filtered` → `/cloud(map)` | 观察器收到 375 条转换后点云；桥接器最后记录输入/输出 370/370，拒绝 0、TF 失败 0（观察者异步采样计数不同） |
| `/body_pose`、`/sensor_pose` | 各收到 375 条 |
| SCAN 膨胀地图 | 32 条，最大点数 31,965 |
| SCAN B 样条 | 4 条，全部通过消息结构有效性检查；日志 4 次 `final_plan_success=1` |
| 参考路径 | `/nav/test/global_path` 收到 4 条 |
| 合成目标 | 3 条；FSM 有 `GEN_NEW_TRAJ`、`EXEC_TRAJ`、`REPLAN_TRAJ` |
| 动态避障接口 | 74 条状态；`dynamic_detection`、`kalman_prediction`、`mpc_cbf_solver` 仍 `NOT_IMPLEMENTED`；`actuator_output=DISABLED` |
| 控制输出与收尾 | `/cmd_vel` 发布者 0；本轮进程及 ROS 图清理 **CLEAN** |
| 测试报告 issues | `[]`（本轮该测试规格下无异常） |

本轮日志中 4 次 SCAN 内部成功规划的总计算耗时约 **6.15–6.30 ms**，但不能据此推断整链从感知到动作的端到端 500 ms 指标合格；静态/动态避障的碰撞安全及长时稳定性均未被该冒烟报告证实。

**真实录制 Bag 的独立只读证据保持不变：** `bag_20261027_225100` 记录了 928 条过滤点云、2731 条 `/tf`、12 条 `/tf_static`、609 条 `/robot_pose`；`/mapping_status` 存在，`/localization_status` 缺失。当前扫描器按上限仅读取前 120000 条消息，观察到 `base_footprint→base_link` **0.30 m 与 0.90 m 两种高度**。该历史 Bag **没有被本次 PASS 测试实际回放给 SCAN**，也没有证明运行中的唯一 TF 发布源；不能将合成 fixture 门控结果记为真实 SLAM Tracking 已通过。

**推荐下一步**：保留已通过的合成集成测试作为回归基线；优先建设只读取指定白名单话题、隔离且不会广播控制命令的真实 Bag 输入链路，同时在独立数据集隔离或重新映射下审核历史 TF 冲突和缺失的定位状态。G1 碰撞验收、动态障碍检测/预测、MPC-CBF 实现继续保持未通过/未实现状态。没有必要为该已通过的合成冒烟测试立即再跑一次。
