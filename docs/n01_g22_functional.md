# G2.2：静态背景过滤、稳定运动判定与离线路径风险预览

**状态：开发代码已接线，待用户 Ubuntu 22.04 / ROS 2 Humble 实测。** 本阶段优先实现功能，不追逐规划耗时指标；严禁连接 Hive2 底盘或发送控制命令。

## 输入输出数据流

```text
已录制药厂 Bag（只读取白名单：/tf、/tf_static、/livox/lidar/geo_filtered）
   → 历史 map TF 按点云时间戳重建（0.90m 仅为用户选定的离线几何）
   → /cloud(map)、/body_pose(map) ──────────────┬→ 原有 SCAN
                                              │    └→ B-spline → /nav/test/global_path
                                              │                         │
                                              └→ G2.2 动态跟踪器        │
                                                   ├ map 聚类             │
                                                   ├ 聚类尺寸上限 1.8m   │
                                                   ├ 背景稳定栅格过滤     │
                                                   ├ 多帧方向一致性       │
                                                   └→ /n01/offline/dynamic_tracks
                                                                 │       │
                                                                 └──┬────┘
                                                                    ↓
                                                      /n01/offline/path_risk
                                                       离线 h_proxy 距离风险
                                                                    ↓
                                                    dynamic_port → observer
                                                   状态报告、日志；**无控制**
```

## 本次实现的功能

1. **异常尺度剔除**：原日志出现了 XY 外廓跨度约 5.94 m 的运动候选，容易将墙体/货架拼成“动态物体”。新增默认 1.8 m 尺度门槛，仅用于研究型目标候选过滤，**不删改供 SCAN 建图的原始 map 点云**。
2. **滚动静态背景过滤**：在 `map` 坐标系累计 8 帧、至少 5 次重复占据的栅格作为稳定背景；候选聚类与稳定栅格重叠达到 65% 时不参与动态跟踪。会错误压制停留或极慢移动物体，不能用作唯一安全感知依据。
3. **多帧运动一致性**：改进跨帧关联后对最近最多 5 组位移速度方向做一致性判断；至少 3 组方向有效、一致性达到 0.8 且命中不少于 4 次，才输出 `MOVING_HYPOTHESIS_UNVERIFIED`。仍非人员/车辆真实分类。
4. **新路径风险接口** `/n01/offline/path_risk`（`std_msgs/String` JSON）：消费 SCAN `/nav/test/global_path` 与运动候选，按**假定** 0.45 m/s 路径行进速度，在 1.5 秒内（步长 0.25 s）计算 `h_proxy = d_xy² − R²`。R 为**假设** 0.60m 基础半径加候选目标外廓半对角线。可报告 `POTENTIAL_PROXIMITY_CONFLICT` 或 `NO_PROXY_OVERLAP_UNDER_UNVERIFIED_ASSUMPTIONS`，但**后一项绝非“路径安全”或真 CBF 通过**。
5. **严格无控制输出**：新路径预览节点只有 `/n01/offline/path_risk` 一个诊断 publisher。没有控制仿射机器人模型、制动计算、MPC 优化器、运动约束闭环、控制话题。接口固定输出 `safety_authorization=NEVER`、`cbf_solver_implemented=false`、`actuator_output=DISABLED`，`dynamic_port` 保留 `mpc_cbf_solver=NOT_IMPLEMENTED`。
6. **回归测试**：新增静态背景压制、移动目标保留、5.94m 过大障碍剔除、反转时间戳重置、位移抖动不误判、持续同向目标、参考路径与目标交叉、远离目标、无目标不等于安全、无效路径拒绝，以及发布话题只读检查。

## 运行约束与验收策略

继续复用此前通过的真实 Bag 一键入口；原 Bag 无 `/localization_status` 且历史 0.30/0.90m TF 高度冲突，这仍属于**开环历史数据开发**，绝不视为实时定位通过。真实 Bag 不提供已标注的动态障碍真值，因此本轮只要求新跟踪器正常处理点云、生成风险接口消息及 SCAN 正常输出 B 样条/Path；**不要求一定有运动候选，也不要求出现风险冲突**。

```bash
cd /home/demo/lyy/hivecore-navigation && git -c http.version=HTTP/1.1 pull --ff-only origin main && bash scripts/run_n01_bag_functional.sh
```

脚本使用 `ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1`；不会自动广播历史 Bag 的 TF 或控制命令。运行后上传 `n01_bag_functional_*.tar.gz`。

查看 `recorded_bag_report.json`：
- `counts.dynamic_track_processed`：应有真实点云处理证据；
- `counts.path_risk_messages`：应有风险诊断消息；`path_risk_evaluable` 可以为 0（当没有可信运动候选或路径短暂过期）；
- `last_path_risk.status`：必须按未验证前提解释，不能当作安全许可；
- `dynamic_port.mpc_cbf_solver=NOT_IMPLEMENTED`、`dynamic_port.actuator_output=DISABLED`，且 `/cmd_vel` 发布者 0；
- SCAN /planning/bspline、Path 输出以及自动进程/ROS 图清理仍需通过。

## 后续开发方向

当有标注的运动障碍数据或可复现的移动目标场景时，评估背景误删、跟踪偏差与预测准确性，再独立实现含 robot/control model、适当碰撞包络、不确定性及约束求解的**离线真 CBF/MPC 模块**。只有完整 P0 与现场安全验收后才考虑底盘运动，规划时延优化留后。

相关：[G2 首轮动态候选](n01_dynamic_tracking_stage.md) · [真实 Bag 回放](n01_real_bag_functional.md) · [P0 未决](hive2_slam_p0_readiness_preflight.md)。
