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

## 2026-10-29 20:59：首次 G2.2 测试失败与源码修复

用户上传一键日志包：`n01_bag_functional_20261029_205936_993133.tar.gz`，运行版本 `e290c46`。

- `summary.txt`：`result=FAIL`、`reason=offline_integration_unit_tests_failed`、`exit_code=3`；
- Python 语法、旧桥接 9/9、进程守护测试均通过；新集成包 **43 项中 41 项通过、2 项失败**，分别为 `test_moving_target_constant_velocity_horizon`、`test_consistent_target_remains_detectable`；
- 测试在 `colcon` 构建及 ROS 图初始化之前中止，因此**本轮没有实际开始真实 Bag、SCAN、路径风险预览测试**；`owned_process_cleanup=CLEAN`、`post_cleanup_ros_graph=CLEAN`。

**根因：** G2.2 Tracker `update()` 中清空 `t.velocity_history` 的语句缩进到了 `if i not in matched_tracks` 之外，使成功匹配的跟踪目标也在每帧被清空历史。持续运动因此永远达不到至少 3 次有效方向样本门槛。

**修复：** 将速度历史重置放入“目标未关联成功”条件内部；不降低运动速度、方向一致性、轨迹有效性等判据。额外新增 `test_matched_history_persists_then_clears_after_miss` 精确验证“连续匹配历史保留、丢失匹配时清空”，并继续执行原先两项失败用例。

**待验证：** 修复提交未在用户 Ubuntu 上重新执行，此处不得标记 G2.2 通过。后续沿用 `bash scripts/run_n01_bag_functional.sh` 做一次统一真实数据验收；只有结果日志明确通过，才能宣布该阶段功能测试完成。

## 2026-10-29 21:05：G2.2 修复版通过真实 Bag 功能验收

**原始用户日志：** `n01_bag_functional_20261029_210503_995837.tar.gz`；测试代码版本 `ef851fe`。本节为**用户 ROS 2 Humble 开环功能实测**，不是现场安全认证。

| 观测项 | 结果 |
| --- | --- |
| 一键摘要 | `BAG_FUNCTIONAL_PATH_SMOKE_PASS`、退出码 0、报告 `issues=[]` |
| 单元测试 | 集成包 44/44 PASS；桥接 9/9 PASS；清理守护 1/1 PASS |
| ROS 2 build | 8 packages finished |
| Bag 点云 | 180 条读取、99 条成功转换到 `map`、81 条因无法在扫描时间获取历史 map TF 而拒绝，错误 frame/畸形云均为 0 |
| 动态跟踪 | 100 条诊断、99 帧实际处理；`moving_hypothesis_frames=16`；最大候选 3 个/帧 |
| 对照上版 G2.1 | 同一历史 Bag 99 帧中有 87 帧产生运动候选；本次减为 16 帧。**仅表明新过滤逻辑改变了输出，不构成误报率/准确率定量证据** |
| 聚类过滤 | 过滤器累计 `oversized_clusters_rejected_total=140`、`static_background_rejected_total=163`；最后一帧统计原始聚类 8、输出候选聚类 3、过大剔除 3、稳定背景剔除 2；背景重置 4 次 |
| 路径风险预览 | 95 条诊断，`path_risk_evaluable=1`，`path_risk_conflict_previews_seen=0`；最后状态 `NO_FRESH_REFERENCE_PATH` |
| SCAN | 11 条膨胀地图、最大点数 198187；2/2 有效 B 样条、1 条参考 Path、2 个离线目标、两次 `final_plan_success=1` |
| 只读与收尾 | `/cmd_vel` 发布者 0、`actuator_output=DISABLED`、进程和 ROS 图 CLEAN |

**已经通过：** 改正的跨帧运动历史逻辑没有再次触发单测失败；真实药厂历史点云已进入 G2.2 静态背景过滤、方向一致性运动候选链；诊断性路径风险接口与 SCAN 并行工作，无控制输出。

**不能据此宣称：** 静态误报率达到某指标；真实移动人员/车辆已被正确识别；无候选时安全；完成 CBF/MPC 约束求解；动态碰撞验收、实机定位就绪或可下发运动控制。当前末态 `NO_FRESH_REFERENCE_PATH` 和全程只有一次可计算的路径-目标配对，说明下一步应优先开发**路径参考与障碍轨迹时间对齐、持续可计算的离线轨迹风险评估**，并对可能的背景误删补充运动真值对照，而不是立刻调性能。

**仍未解决的数据风险：** 81/180 条原始点云缺扫描时刻 map TF；旧 Bag 的 `base_footprint→base_link` 高度同时含 0.30 / 0.90 m；`/localization_status` 缺失。历史离线回放的 PASS 不是 P0 严格定位验收。

**操作：** 本轮只需将上述验收记录归档，无须重复运行已通过的 G2.2 测试。随后开发新功能时，再执行统一离线回归。

## G2.3 继续开发：历史轨迹与目标时间融合（待本机验收）

G2.2 实测风险接口绝大多数是未评估心跳，因此 G2.3 只保留 SCAN Path 的几何参考，严格按照历史 Bag 的 **body_pose 与动态报告同一 scan stamp** 进行反复评估，当前 Path 消息时间不能假装成历史采样时间。原 G2.2 PASS 不覆盖新增 G2.3，详情见 [G2.3](n01_g23_path_timeline.md)。
