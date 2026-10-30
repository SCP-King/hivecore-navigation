# G3.3：按 Hive2 `robot_motion.yaml` 筛查离线 G3 候选轨迹（2026-10-30）

**已实现并接入** `n01_offline_integration/g3_pipeline_core.py` 和逐帧审计 `g3_frame_evidence.py`；纯 Python CI 验证。**未在 Ubuntu ROS2 真实 Bag 中重新运行该版本**，不启动底盘/SLAM、无底盘指令、无真实轮式安全授权。

## 现有资料已经足以开发的部分

用户上传并经审计的两份 `robot_motion.yaml`，SHA-256 相同：
`3f19abe0a65e02b331a0d51c18ec0503278a4d9a6b942332b47c4f7d434584f0`。
核心运动配置：四轮独立驱动、四个 ankle_yaw 转向、wheel radius=0.095m、wheel_base_x=.56m、wheel_base_y=.27m、delta_y=.085m、chassis max linear speed=2.0m/s、nominal/max linear acceleration=.6/1.0m/s²。
当前项目另有 N-01 导航限速 `<=1.0m/s`，原生 SCAN 的 `max_vel=.75m/s`、`max_acc=.5m/s²`。
配置值**不是实测加减速、轮速控制律或安全认证**。

当前 G3 数值原型 `solve_discrete_mpc_cbf` 的 `max_accel=3.2m/s²` 是**旧研究默认值**，明显高于 Hive2 控制配置的 1.0m/s²。为保持历史 G3 研究/回归对照，**本次不伪装成适配完成而直接全局替换默认值**；新增后验约束筛查并产生真实差异证据。

## 新增离线约束筛查

新模块：`n01_offline_integration/g3_hive2_wheel_screen.py`，唯一输入为 G3 `candidate_xy_m` 与 `discretization_dt_s`。检测轨迹段上的平均平动速度和离散相邻段速度变化的加速度模长，按三个不同来源分别记录：

1. Hive2 底盘配置的最大线速度 2.0m/s、最大线加速度 1.0m/s²；
2. SCAN 已采用的最大速度 0.75m/s、最大加速度 0.5m/s²；
3. N-01 项目速度要求 1.0m/s。

逐帧可能输出：
- `CONFIGURED_CHASSIS_TRANSLATION_EXCEEDED`：轨迹超过已审计底盘平移配置；明确暴露与 G3 默认值不一致；
- `SCAN_TRANSLATIONAL_PLANNING_BOUNDS_EXCEEDED`：未超过底盘配置，但超过 SCAN 的平移约束；
- `TRANSLATIONAL_BOUNDS_WITHIN_CONFIG_4WS_UNVERIFIED`：这些离散平移必要条件范围内，**绝非四轮转向可执行**；
- `G3_HAS_NO_CANDIDATE` / `INVALID_DISCRETE_TIME` / `INVALID_XY_CANDIDATE`：无足够信息时阻断结论。

首次加速度按 **G3 求解器显式假设的初始零速度**计算，并明确标注 `first_acceleration_measured=false`：历史 `/body_pose` 的旧桥接消息并不携带真实底盘 twist，不可认为起步实际从静止开始。
角速度、转向角速度、轮速、轮胎侧滑、制动、地面支撑、全身碰撞等仍为 `unverified_requirements`，所有情况下 `robot_model_execution_feasible=false`、`physical_safety_authorization=NEVER`。

### 在全链中的位置

```text
历史 Bag 点云 + 历史时间戳 map /body_pose
               ↓
SCAN 原生 B-spline -> Path 研究参考
               ↓
G2 运动候选 ——→ G3 离散近似 MPC-CBF
                           ↓
                    G3 candidate_xy_m
                           ↓
       G3.3 Hive2 来源参数平移约束筛查
                           ↓
    /n01/offline/g3_solver_report (附 hive2_motion_limit_screen)
    g3_frame_evidence.jsonl (重算并逐帧记账)
    g3_evidence_summary.json (聚合超限统计)
                           ✕ 永远不接 /cmd_vel / 底盘控制器
```

`g3_pipeline_core.G3HistoricalCoordinator.evidence()` 额外统计：
`hive2_xy_screened_candidate_frames`、`hive2_chassis_limits_exceeded_frames`、`hive2_scan_limits_exceeded_frames`、`hive2_translation_within_limits_frames_unverified_4ws`。
后验屏查不改变历史候选优化结果或 `g3_candidate_path` 的**仅研究可视化性质**，不修改任何真实硬件配置或 SLAM 参数。

## 下一步：分阶段，避免重复索要基础参数

### A. 一次真实历史 Bag 离线 G3.3 验证

在物理隔离 PC、原有测试环境 `ROS_DOMAIN_ID=88`、`ROS_LOCALHOST_ONLY=1`、`N01_HISTORICAL_BAG_ONLY=1` 约束下复用原 `scripts/run_n01_bag_functional.sh`。期望输出必须明确：真实历史时间戳匹配数量，实际产生的候选数量，G3.3 超限帧数及其来源，`/cmd_vel` 发布者零、清理 CLEAN。**尚未执行这一步**。GitHub 静态 CI 仅证明对合成/受控纯 Python 输入的筛查正确。

### B. G4 地图衔接

复用已经从实际 Bag 锚定的 8.036m / 29 点 XY 路线。先检查沿线路的 G4.9 PCD 多层表面与局部地面支撑/未知区，不将 `-0.5m` 低簇自动认地面；明确 SCAN `body_height=.4m` 与 Hive2 TF `base_link=.9m` 不是同类参数。三维语义未对齐时仍不得将 G4 路线强行送入 SCAN。

### C. 真正四轮转向可执行性模型（下一轮算法研发）

使用已知的几何/线加减速配置，继续研发不输出命令的轮式逆运动学与转向变化率研究评估。若目标是**精确复制甲方控制器的轮速/转向角分配、前进/倒车与 `yaw_soft_limit_rad=π` 对 URDF `ankle_yaw` ±π/2 的处理**，仍需 `motion_controller/ChassisController` 的实现或等价可验证说明。**没有这些信息时只给必要条件和 INCONCLUSIVE，不能写“实机可执行”。**

### D. 以后实机集成门槛（当前不执行）

先由甲方/运动栈核实同一边 `base_footprint→base_link` 的唯一 Hive2 0.90m 发布者，禁用 SLAM 0.30m 重复 RSP；定位必须满足真实 `LocalizationStatus` tracking、扫描时刻 TF；固定 pitch 关节反馈、轮式控制路径、地面净空、制动距离和安全距离也需验收。不得由离线代码打开机体控制权限。

## 已加入 CI 的静态回归

```bash
python3 -m unittest discover -s src/n01_offline_integration/test \
  -p 'test_g33_hive2_wheel_screen.py' -v
python3 -m unittest discover -s src/n01_offline_integration/test \
  -p 'test_g3_mpc_cbf.py' -v
python3 -m unittest discover -s src/n01_offline_integration/test \
  -p 'test_g3_frame_evidence.py' -v
```

限幅数值由测试与 `config/reference/hive2_robot_motion_audit_20261030.json` 核对，防止仓库参数在以后改动时漂移。测试仍不包含 ROS、实机控制器和完整药厂环境。

## 2026-10-30 后续 G3.4：由后验筛查升级为求解时约束

此前 G3.3 只计算诊断，`solve_discrete_mpc_cbf` 独立默认仍为旧 3.2m/s² 以保留历史回归对照。现在 **`G3HistoricalCoordinator` 的默认真实历史 Bag 研究输入**已切换成 `acceleration_increment` 格、`max_speed=.75m/s`、`max_accel=.5m/s²`，并在最后防线清空所有超限候选。旧 3.2m/s² 不再是新的**实际离线 Bag 协调器**生成配置，但独立旧数值测试依旧存在；不能混称二者。参见 [G3.4 完整算法和单轮验收](n01_g34_hive2_bounded_candidate_generation.md)。
