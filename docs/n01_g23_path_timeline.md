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
