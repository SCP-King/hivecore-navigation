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
4. 如可发现先前 `bag_20261027_225100` 或指定 `N01_BAG_PATH`，只读检查 Bag `metadata.yaml` 中的话题类型/数量；**这一步不回放 Bag，不证明真实 TF 有效**；
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
