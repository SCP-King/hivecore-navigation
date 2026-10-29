# G2.3 历史路径持续评估与障碍预测的扫描时间轴融合

**实现版本：待用户在 ROS 2 Humble 开环离线验证。** 本阶段继续功能优先，不修改 SCAN 核心规划参数，不接 Hive2 控制，不输出运动命令。

## 已知问题与设计边界

上一轮 G2.2 在真实药厂历史 Bag 上完成 99 帧动态检测，观察到 95 条风险接口消息，但**只有 1 次路径—动态目标候选被实际配对评估**。原风险节点用宿主机单调计时作为 Path 15s 过期判据，而 SCAN Path 由桥接节点以当前 ROS 时钟发布；历史 Bag 点云与 /body_pose 的 stamp 则来自当时录制时间。这两个时间轴不能直接混为一个时钟。路径也通常只发布一次，随后仍会有真实 Bag 点云输入。

**G2.3 的策略：保留最后一条 SCAN 路径作为“仅用于诊断的历史几何假设”，而不是宣称这条路径始终实时有效。** 每次用**完全相同的历史消息时间戳**连接机器人 /body_pose 与 /n01/offline/dynamic_tracks，再将此时刻的机器人 map 位姿投影到保留的路径上，从投影点沿路径累计假定行进距离。障碍预测以相同扫描时间为 t=0。

## 代码架构

```text
/robot_pose + /tf (历史 Bag，只读)
        ↓
historical_bag_node → /body_pose(map, 历史 scan stamp) ──┐
                    → /cloud(map, 同 scan stamp)        │
                              ↓                         │
                   dynamic_tracker                      │
                              ↓                         │
       /n01/offline/dynamic_tracks (历史 scan stamp)    │
                              └─────────┬───────────────┘
                                        ↓
                          ReplayPathSynchronizer
                           严格 stamp 相等配对
                                        ↑
     SCAN B-spline → /nav/test/global_path（只存几何，不把其
                       当前时钟 header 伪当成历史 Bag stamp）
                                        ↓
                    map 位姿投影到最新保留路径
                                        ↓
                    路径假定 ETA 与目标匀速预测
                                        ↓
           /n01/offline/path_risk（JSON，只读证据）
                                        ↓
                dynamic_port → observer → 归档日志
```

## 本次新增的功能

1. **精确时间匹配**：`replay_path_sync.py` 只在 `/body_pose.header.stamp` 与 `dynamic_tracks.stamp_ns` 完全一致时评估。支持两种消息先后顺序；两侧缓冲各最多 16 条。**不使用最新 TF/里程计替代缺失的原始扫描时刻数据。**
2. **Path 保留与当前位置投影**：Path 只有一条也可以继续供随后的历史扫描帧使用，实时收到新的 Path 时替换旧几何。每帧按历史机器人位置投影至最近线段，记录到路径偏差与累计弧长，**机器人与保留 Path 相距超过 1.5m 时只输出不可评估诊断**；不允许在终点外强行外推。
3. **同一历史扫描时刻的相对 ETA 风险预览**：沿 Path 假定 0.45 m/s 行进，在未来 0..1.5 秒（0.25 秒步长）推算参考位置；动态障碍速度为未核实的匀速假设，在同一相对时间计算中心间距和 `h_proxy = d_xy² - R²`，记录目标编号、预计碰撞时刻、该时刻机器人与障碍坐标、历史扫描 stamp、最小代理距离风险值。
4. **跨帧累计报告**：`aligned_replay_frames`、`aligned_frames_with_proximity_pairs`、`reference_reused_without_replanning`、`minimum_barrier_proxy_across_recorded_frames_m2` 等写入路径风险消息、动态接口和一次性观察器日志。没有已确认运动假设仍可证明时间对齐，但不得判为“无碰撞”。
5. **缺失、倒退、离线输入结束**：历史时间倒退时清空关联缓冲并废弃已有 Path，要求新路径再进入；Bag 扫描结束、3 秒内无新匹配帧时只发 `NO_RECENT_EXACT_TIMESTAMP_PAIRED_FRAME` 心跳，历史最坏风险保留作证据，**心跳不计为新的风险计算结果**。
6. **原始安全边界保持**：`safety_authorization=NEVER`、`cbf_solver_implemented=false`、`mpc_solver_implemented=false`、`actuator_output=DISABLED`。没有优化/求解控制量、真实制动约束、闭环安全证明，也不发布 `/cmd_vel`。

## 一次性验收

保持 Hive2 真机运动控制链断开，在 Ubuntu 22.04 + ROS 2 Humble 上运行：

```bash
cd /home/demo/lyy/hivecore-navigation && git -c http.version=HTTP/1.1 pull --ff-only origin main && bash scripts/run_n01_bag_functional.sh
```

脚本将运行已有及新增单测、构建、白名单历史 Bag 回放、SCAN 规划、动态跟踪、时间轴融合与观察器，最后清理本轮进程并打包 `n01_bag_functional_*.tar.gz`。

**新增验收门槛：** `recorded_bag_report.json.counts.path_risk_aligned_frames > 0`、`counts.path_risk_retained_frames > 0`；仍要求 SCAN 有效 B 样条/Path、动态跟踪真实点云处理、`/cmd_vel` 发布者 0 与进程/ROS 图 CLEAN。是否出现真实运动候选、风险重叠并不决定软件链路 PASS，因为缺少真值标注。

> 特别注意：这里对齐的是**历史障碍报告与历史机身位置的扫描时间戳**，并没有将由当前时钟发布的 SCAN Path 赋予真实历史生成时刻。保留的 Path 是离线假设，行进时间依据假定速度，不代表实时定位、真正 MPC-CBF 或真机安全验收通过。

## 后续模块

在本轮软件接口通过后，下一步可开发基于 B 样条参数化轨迹时间、已知障碍真值和真实机器人状态的离线约束评估；进一步增加运动学/制动模型并实现真正的 MPC-CBF 求解器。**在 P0 定位状态、TF 权威冲突、实机安全包络与控制约束闭环未验证前，依旧禁止任何实机控制输出。**

## 2026-10-29 21:54 实测复盘：发现“保留 Path”验收假阳性，现已在下一提交修复

原始用户日志：`n01_bag_functional_20261029_215431_1019311.tar.gz`；测试版本 `bd60a5a`。

**该版本的基础链路结果：** runner 标记 `BAG_FUNCTIONAL_PATH_SMOKE_PASS`、退出码 0，ROS 2 Humble 8 包编译、61/61 集成包单测、9/9 路径桥接测试、清理守护测试均通过；99 条真实录制点云转换成功，81 条因扫描时刻 map TF 不可用被拒；动态模块处理 99 帧，16 帧出现未经真值验证的运动候选；SCAN 产生 1/1 条合法 B 样条与 1 条 Path；无 `/cmd_vel` 发布者，清理 CLEAN。

**但 G2.3 风险联调不合格：** `aligned_replay_frames=99` 证明精准历史 stamp 配对；`aligned_frames_before_first_path=95`，其余 **4 帧全部在保留 Path 的 1.5m 允许走廊之外**（`body_outside_reference_frames=4`）；`aligned_frames_with_proximity_pairs=0`、`path_risk_evaluable=0`、`minimum_barrier_proxy_across_recorded_frames_m2=null`。根因是 `ReplayPathSynchronizer` 在“已找到 Path 几何，但位姿超出走廊”的诊断结果上也错误地设置了 `reference_reused_without_replanning=true`，被 observer 误计为 4 帧有效复用，造成验收脚本假阳性。**上述旧版 PASS 仅指 Bag→SCAN 与时间对齐，不代表 G2.3 完成有效路径风险计算。**

### 修复设计及新增功能

1. **明确判据：** 只有机器人位置到 Path 最近投影距离不超过 1.5m、且投影后仍有真实剩余路径几何，才记 `path_projection_usable=true`；路径超界、到达路径终点或缺数据均记为 `false`。它们**不得**使新 G2.3 验收通过。
2. **非实时的有界事后对照**：ROS 历史录制流通常在 Path 生成前已播放多帧。允许最多暂存 160 条**已经精确同 stamp 配对**的历史机器人位姿与障碍候选；当 SCAN Path 后来抵达时，使用该条路径几何对先前帧做 **POST_HOC_PATH_GEOMETRY_COMPARISON_NOT_HISTORICAL_PLANNING** 比较。这是历史几何研究，**不是**“当时有该 Path”或闭环避障证据。
3. **两种证据隔离**：`live_projected_reference_frames` 为 Path 已经进入图后的真实消息顺序投影数量；`posthoc_projected_reference_frames` 为后来 Path 对旧帧事后投影数量；`aligned_replay_frames` 仍只统计输入时同 stamp 配对的帧，不将事后对照重复计为新帧。动态避障真实距离候选配对也分 `aligned_frames_with_proximity_pairs` 与 `posthoc_proximity_pairs`。
4. **严格无指令**：新节点仍只发布 `/n01/offline/path_risk` 诊断；风险 proxy、posthoc 最危险值与扫描 stamp 属于假定速度、未知目标真值下的探索结果，`safety_authorization=NEVER`、CBF/MPC 求解器未实现、`actuator_output=DISABLED`。
5. **新回归测试**：超出走廊不计“有效 Path”；后到 Path 对前序精确配对帧可做事后处理，但不冒充 contemporaneous；历史缓冲有上限、重复消费不重复计数；时钟倒退清空历史缓冲及参考路径。

**复验命令不变：**

```bash
cd /home/demo/lyy/hivecore-navigation && git -c http.version=HTTP/1.1 pull --ff-only origin main && bash scripts/run_n01_bag_functional.sh
```

复验报告必须分别列出 LIVE 与 POST_HOC 投影；只有确实存在**走廊内**的投影才可证明历史几何关联功能。若仅 POST_HOC 有效，脚本会显式输出 `real_bag_SCAN_pass_G23_POST_HOC_projection_only_NOT_live_risk`；这**不等于**实时风险评估完成。若这两类投影都没有，测试应标记 `BAG_FUNCTIONAL_INCOMPLETE`，不能再把 4 帧超界冒充 PASS。

**代码尚未在用户的 ROS 2 Humble 环境重跑。** 上述修复是新的待验收版本，不应反过来改变旧日志实际结果。
