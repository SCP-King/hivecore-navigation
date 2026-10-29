# N-01 真实 Bag 功能回放（首版，2026-10-29）

**目标：先完成真实药厂记录数据对接与规划功能，性能优化后续进行。**

这是与已通过的合成 fixture **不同** 的入口：从历史 Bag 读取真实 `/livox/lidar/geo_filtered` 和 `/tf`，在离线程序内部重建点云 `map` 坐标，送给原有真实 SCAN Planner、B 样条 Path 桥接及禁用的 DynaCBF 接口。

## 本轮已开发的功能

1. **真实数据输入：** 用 `rosbag2_py.SequentialReader` **只读**读取 Bag，仅允许输入 `/tf`、`/tf_static` 和 `/livox/lidar/geo_filtered`。完全不调用 `ros2 bag play`，**绝不重新广播历史 `/tf`/`/tf_static`，不重新发布任何 Bag 记录的运动指令、底盘话题或原始传感器话题**。
2. **历史 TF 处理：** 仅提取 `map→base_footprint` 的历史变换，按点云 `header.stamp` 进行有界插值/最近邻匹配；无法匹配就拒绝当前点云。统计 Bag 中 `base_footprint→base_link` 的不同高度，并标记 0.30/0.90 m 冲突，但**不直接套用旧挂载边**。
3. **选定几何下的离线重建：** 用用户确认的 `base_footprint→base_link: z=0.90m` 及 `base_link→lidar_link: x=0.295m` 组合位姿，将 `lidar_link` 点云实际转换到 `map`，并生成 `/body_pose` 和 `/sensor_pose`，供 SCAN 在同一规划系建图。结果只代表**基于当前选择的历史重建**，不是已验证实时 TF。
4. **离线规划：** 启动真正的 SCAN planner，订阅本模式产生的 `/cloud`、`/body_pose`、`/sensor_pose`。由于 Bag 没有 RViz 导航目标，节点会在第一批有效点云后自动产生相对第一帧记录位姿前方约 1.5 m 的**离线测试目标**；首版只抽取前 180 条符合过滤条件的点云，按约 4 Hz 做功能回放，不声称已扫描整包；该目标不保证一定是药厂真实可通行点，不会发送到底盘。
5. **路径接口与动态避障占位：** SCAN B 样条接入已有 Path 参考输出接口；DynaCBF/Kalman/MPC-CBF 保持 **NOT_IMPLEMENTED**、控制输出 **DISABLED**。
6. **诊断与清理：** 新模式独立发布 `/n01/offline/bag_replay_status`，记录真实输入/输出、时间匹配失败、历史 TF 高度列表、目标条数、是否见 `/cmd_vel`；新的脚本运行单测、编译、ROS 图隔离检查、自动验收、按本轮 token 清理进程并生成 `.tar.gz`。

## 已有 Bag 实际约束

用户提供的 `bag_20261027_225100` 在上一轮只读审计中包含 928 条过滤点云、2731 条 `/tf`、12 条 `/tf_static`、609 条 `/robot_pose`，并录有 `map→base_footprint` 的历史变换。**历史挂载高度同时出现 0.30/0.90 m，而 `/localization_status` 缺失。**

因此新增模式叫 **`HISTORICAL_BAG_OPEN_LOOP_NOT_LOCALIZATION_TRACKING`**，只用于规划功能开发。与严格 `/localization_status` 门控的真实在线模式明确分离；此模式**不声称定位 tracking 已通过，不允许将回放输出接入实机控制**。

### 一条命令测试

默认使用上一轮已发现的本机 Bag：`/home/demo/lyy/slam_data/bags/bag_20261027_225100`。

```bash
cd /home/demo/lyy/hivecore-navigation && git -c http.version=HTTP/1.1 pull --ff-only origin main && bash scripts/run_n01_bag_functional.sh
```

若要指定其他 Bag，同样只需一条命令（把路径改成实际 Bag 目录）：

```bash
cd /home/demo/lyy/hivecore-navigation && git -c http.version=HTTP/1.1 pull --ff-only origin main && N01_BAG_PATH=/path/to/bag bash scripts/run_n01_bag_functional.sh
```

本轮脚本不重跑已通过的合成 fixture 75 秒验收，而是直接对**真实 Bag 功能链路**验收。运行结束会显示 `UPLOAD THIS LOG ARCHIVE:`，把 `n01_bag_functional_*.tar.gz` 上传即可。

### 如何判断

| 结果 | 含义 |
| --- | --- |
| `BAG_FUNCTIONAL_PATH_SMOKE_PASS` | 本轮至少观察到真实录制点云转换、SCAN 膨胀地图、有效 B 样条和 Path 参考输出，且无 `/cmd_vel` 发布源；仅是**离线功能链路**通过 |
| `BAG_FUNCTIONAL_INCOMPLETE` | 有些链路已工作，但缺失目标、地图、轨迹或参考路径；可查看报告中的 `historical_bag_health`、`issues`，不能当 PASS |
| `FAIL/ABORT/CLEANUP_FAILED` | 代码错误、输入缺失、ROS 域不干净或进程残留；按具体错误分析 |

如果历史 map TF 与点云时间戳对不齐，脚本将体现 `skipped_no_map_tf`；如果自动生成的目标被障碍阻挡，SCAN 可能没有有效 B 样条。这是实际功能开发的反馈，不通过修改 G1 安全参数或伪造 TF/轨迹来硬凑 PASS。

## 当前边界

- **不是实时 Localization Tracking 验收**：Bag 缺失 `/localization_status`，历史 map TF 仅供离线数据重建。
- **不是全身安全碰撞模型**：SCAN 仍使用当前研究配置，0.90 m 是离线转换基准，实际轮足姿态及安全包络未定。
- **不是 DynaCBF/MPC-CBF 可用实现**：仍仅有禁用接口，后续开发功能时逐步扩充动态障碍感知、预测、约束求解。
- **不是性能基准测试**：暂不以 ≤500ms/连续 8h 等指标作为本阶段完成判据，但会保留实际运行日志供后续优化。
- 没有修改甲方 SLAM、运动栈、ROS Domain 以外的系统配置，也没有发布真实控制消息。

**截至本次提交，真实 Bag 新模式完成的是代码框架与离线验收脚本，尚未在用户 Ubuntu 环境实际执行。**
