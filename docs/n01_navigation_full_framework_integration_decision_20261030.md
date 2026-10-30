# N-01 / Hive2：从“地面研究”转向端到端导航框架联调的决策（2026-10-30）

> **新增证据修正：不应继续默认 map 地面 Z=0。** 旧地图与 /tf 动态 +0.30m 链的点云重合证据更强，局部候选地面约 -0.60～-0.50m；旧离线程序按 +0.90m 重建点云可能相对原 PCD 垂直错位约 0.60m。新 G4.12 可只读比较两个链，并按 SE(3) 算理论补偿；另提供显式 0.30m 历史回放研究参数，旧 0.90m 回放仍为默认。不能直接改实时 SLAM TF 或确认全图地面。见 [G4.12 复核](n01_g412_historical_mapping_030_vs_motion_090.md)。

## 结论

用户目标：不继续为了地面高程无限增加测试；以甲方 Hive2 底盘为权威，优先看 **定位 → G4 全局参考路线 → SCAN 局部规划 → G2 动态障碍 → G3 轨迹候选** 能否贯通，并可观察、可录屏。**同意把端到端功能演示作为下一研发目标**，但功能演示 ≠ 实机可安全行驶；硬件运动控制仍无授权。

关键事实：
- Hive2 Xacro：`base_footprint → base_link` 名义固定 (0,0,+0.9)m，`base_link → lidar_link` 名义 (+0.295,0,0)m。0.9m 是 **两个 link 的坐标差**；它不是独立测量的轮胎接地点至地面间距，不能仅此宣称地面在 `map Z=0`。
- 从历史 Bag 第一帧可用扫描，`map → base_footprint` Z ≈ +0.003491m。**可以将 map Z=0 当作平地参考的研究假设**，但该数字来源于 TF，不是轮地支撑验收。
- 同一路线起点附近 711 个 PCD 样本的低层 P10≈-0.5265m、median≈+2.5945m，存在多个高度层。仅凭激光高度分布无法证明 Z=0 是地面，或按相同 Z 沿全程能安全通行。
- SCAN 现配置 `grid_map.body_height=0.4m`；Hive2 安装坐标差为 0.9m。两者属于不同来源的几何参数，必须核对 SCAN 对 body_height 的 C++ 使用语义；**不能以盲目改成 0.9m 代替碰撞模型集成**。
- 历史 Bag 功能链：928 帧全读取，571 变换成功、357 因历史 TF 不足拒绝；G3.4 104 次求解按量化容差满足 Hive2/SCAN 平移限幅，但 103 次为 POST-HOC。16 个含移动假设的 CBF 帧内未见候选因为 CBF 被淘汰。不能描述为实时闭环避障。
- 现有 G4.8 使用历史 Bag 的真实起点，得到约 8.036m、29 路点的 **XY 研究路线**。Z 未核实，当前 `run_n01_g4_full_chain.sh` 默认会在 SCAN ROS 启动之前停止。尚未成功证明“同一条 G4 路线进入 SCAN 并产生对应的新 G3 轨迹”。

## 按实际目标分三层验收，停止反复测试基础链

### 1. 第一目标：无电机输出的端到端功能演示（优先）

启动前提：**物理隔离的开发计算机**、`ROS_DOMAIN_ID=88`、`ROS_LOCALHOST_ONLY=1`、`N01_HISTORICAL_BAG_ONLY=1`、历史 Bag；所有候选仅为 `/n01/offline/*` 诊断/显示话题，`/cmd_vel` 和底盘 RT command 发布者为零。可以在纯算法/可视化层使用**明示的假设平地**做功能实验，但必须另立研究标志及产物来源，不可把 `N01_G4_FLOOR_Z_VERIFIED=1` 用于伪造地面确认，不可把结果当作可通行路径。

所需证据（同一条起点/终点路径）：
1. 真实 Bag 扫描时间戳通过 TF 获得 map 位姿；路径起点与当时机器人位置一致；
2. G4 产出 29 点研究初始路由；ROS 仅在明确适配研究高度语义的安全隔离流程中将其作为 `/initial_path` 发送；
3. **之后** SCAN 产生新增原生 `/planning/bspline`，由 bridge 得到 `/nav/test/global_path`，G2 输出对应跟踪信息，G3 输出新候选和限幅屏查；
4. 由 `g4_chain_evidence.json` 追踪严格先后顺序；`G4_CHAIN_INCOMPLETE` 不能当成功；额外检查后续 G3 候选是否真的依赖**新 G4 路线**，否则仅称 ROS 话题流贯通；
5. 轨迹/点云/动态假设记录可导出图或离线录屏；未获地面与全身碰撞证据时明确“计算可视化，不可执行”。

现阶段前置工作：提供单独 **非执行** 的研究高度/参考语义适配设计，并检查 SCAN mode3 真实代码；只有确定与设备运动 TF 的关系后才启用 ROS 话题联通测试。不得静默绕过地面/碰撞保护。

### 2. 第二目标：Hive2 + SLAM 静止在线只读联调

在机器人**保持静止、不接受运动命令**情况下，由甲方启动已知底盘/SLAM 组件（当前项目不会发出启动硬件命令），只读观测：
- `base_footprint→base_link` 的权威应为 Hive2 运动 URDF +0.90m；SLAM 的 `launch_robot_state_publisher=true` 默认会带来 +0.30m 重复发布风险，启动时须由有权的集成运维确认 **SLAM RSP 关闭**，且检查唯一 TF 发布者。
- SLAM 负责 `map→base_footprint` 定位 TF；状态的真实 `LocalizationStatus.msg` 字段 `control_signal_outer_role=localization`、`control_signal_session_mode=tracking`、`registration_output_tf_published=true`。校验状态时间与扫描 header.stamp，一过期就阻断。
- 原始 Livox `livox_frame` 与过滤云 `lidar_link` 的变换用真实采集时间查询，不把重命名充作变换。
- 不重用历史 Bag 的合成零 twist 作为当前实测运动速度。机体轮速、yaw/ankle 关节反馈需要受控接口说明后才可参与轨迹可执行性检查。

只读完成指标：地图坐标连续、时间戳匹配、状态门控正确、没有冲突 TF、无任何运动指令。它**不是**在线自动导航验收，也不需要先开发全地图地面高程模型。

### 3. 第三目标：低速实机走通（晚于上述两层）

只有在甲方硬件负责人确认运动控制的真实控制模式、八个 pitch 关节保持反馈、四个踝部转向关节的关节限位与轮速转向分配、底盘制动距离/急停、真实整机包络、物理监护和经过现场确认的短路线通行区域后，才能制定逐步受控的低速行驶测试。此项目当前**没有**连接实体 `/cmd_vel` 或 `/chassis_controller/rt_command` 的实现/授权。不能用“雷达高约 0.9m，因此地面 Z=0”单独批准实机运动。

## 当前开发决策

1. **停止重复要求用户上传 Bag/URDF/robot_motion.yaml**；已有足够基础参数。
2. **保留当前 fail-closed 地面阈值门**；不将 `map Z=0` 猜测写成 `ground_verified=true`，也不在本项目自动使能底盘运动命令。
3. 下一项真正的代码核对任务是 SCAN `grid_map.body_height`、`grid_map.ground_height`、`fsm.navi_mode=3` 对研究路由 Z 的处理；然后独立做历史 Bag 只读的“基于假设坐标的端到端 ROS 话题联通”实验，报告与实机许可严格分离。
4. 第二离线测试仍用 G4.9/G4.10 PCD Z 证据描述未知区；其目的是提示视觉演示和以后真机的障碍，而不是无限期阻止**纯软件计算链**开发。
5. 仅当完整 G4→SCAN→G2→G3 话题证据齐全，才考虑静止在线定位只读联调；控制授权与运动测试由机器人现场负责方独立批准。

**本文件是一份工程决策与研发验收规划，不代表某次 ROS 全链已经运行成功。**


## 已完成的 C++ 语义核对与自动工具（2026-10-30 新增）

直接核对仓库 `src/scan_planner/plan_manage/src/scan_replan_fsm.cpp`：
- mode 3 (`fsm.navi_mode=3`) 通过订阅 `initial_path` 收到路径；
- `SCANReplanFSM::pathCallback` 对每个 `pose_stamped.pose.position.z` 执行 `+ body_height_`，然后调用 `planGlobalTrajByWaypoints`；
- `SCANReplanFSM::odometryCallback` 的 `odom_pos_.z` 直接来自 `/body_pose`；
- 当前离线 G4 发布器拒绝实际 `/body_pose` 与初始路径上 `reference_z + scan_body_height` 差距超过 0.5m；研究配置 SCAN `body_height=0.4m`。
- `grid_map.ground_height=0` 在 `GridMap::initMap` 参与地图网格原点的 Z 初始化；它不是路面分割得到的高程地图。

**因此**，仅看 Hive2 链 `base_footprint→base_link=+0.9` 无法推出 SCAN 当前 mode3 的 Z 可以直接复用；若只假设 `reference_z=0`，SCAN 研究目标 Z 为 0.4m，而用户提供的首个 Bag 位姿经 0.9m 安装后 `base_link` 大约处于 map Z≈0.9m，差值约 0.5m，正好在参考发布器严限附近。不同扫描姿态可能略有差异，不能提前断言哪一帧一定通过或失败。

已新增 `n01_offline_integration/g411_mode3_z_alignment.py` 并接入 `scripts/n01_g4_bag_start_preflight.py`。读取每次实测的第一帧有效 `map→base_link` 重建结果，输出 `g411_scan_mode3_height_alignment.json`，其中明确列出当研究假设 Z=0、SCAN 使用真实配置 body_height=.4、发布器限制 .5 时的**数值偏差与会否触发参考发布门控**。结果永远不会表示地面已经测得，也不会自动发出三维路径、修改 SCAN 或向底盘控制器发送消息。

已经专门检查过：把 `N01_G4_FLOOR_Z_VERIFIED=1` 设成真值只为启动测试，会混淆功能联调和地面验收，因此**没有把这种强制放开措施提交到最终仓库**。既有默认隔离运行脚本保持 fail-closed。

**下一步研究工作**：复用实际 Bag+PCD 的只读 G4 检查，首先确定 Z=0 假设引起的真正高度偏差，再设计与实机隔离、明确非地面假设的 ROS 算法联通演示；随后执行静止状态下的实时 SLAM 只读 TF/LocalizationStatus 门控。低速轮式实际运动须由甲方独立确认刹车、关节反馈和现场授权。
