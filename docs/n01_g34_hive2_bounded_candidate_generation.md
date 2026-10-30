# G3.4：Hive2/SCAN 限幅下的离线候选轨迹生成（2026-10-30）

**当前状态**：源码已接入历史 Bag → SCAN → G2 → G3 的默认离线研究路径；GitHub 纯 Python 回归测试已通过。用户 Ubuntu ROS 2 实际历史 Bag 的 **G3.4 新版** 运行证据仍需一次集中验收。该开发阶段不启动/控制 Hive2，不改变甲方 `robot_motion.yaml` 或在线 SLAM。

## 1. 为什么 G3.3 后还需要 G3.4

已上传的两份 Hive2 `robot_motion.yaml` 一致，设定：
- chassis max linear speed=**2.0 m/s**；
- chassis max linear acceleration=**1.0 m/s²**（常规 **0.6 m/s²**）；
- SCAN 研究参数另要求 max_vel=**0.75 m/s**、max_acc=**0.5 m/s²**；
- N-01 需求速度不高于 1 m/s。

此前 G3 `solve_discrete_mpc_cbf` 采用二维全向点质点、固定速度格 `{0, 0.225, 0.45, 0.75}m/s`、默认研究加速度上限 **3.2 m/s²**，并假设初始 `vx=vy=0`；G3.3 只是事后报告哪些样本超过底盘/SCAN 配置。直接把默认上限改到 0.5，会使首步最快的非零速度档位 0.225 m/s 在 dt=.25s 下需要 **0.9 m/s²**，超过 SCAN 限制，容易出现永远只选 0 的“表面合规但无法前进”。

**G3.4 修复的是候选生成方法本身**，而不是靠更换一行数值。

## 2. 代码变更

### `g3_mpc_cbf_core.solve_discrete_mpc_cbf`

新增 `control_lattice="acceleration_increment"` 模式，依据前一步状态 `(vx, vy)` 生成下个速度候选：保留当前速度、允许减至零（必须符合减速上限），再沿 16 个平面采样方向施加半档或全档 `max_accel*dt` 速度增量；对每个速度候选检查：
- `||v[k]|| <= max_speed`
- `||v[k]-v[k-1]|| <= max_accel*dt`
- 与现有离散 CBF 代理不等式、路径跟踪代价共同求解。

原 `legacy_fixed_speed` 模式**保留为默认**，支持已录制历史 G3 数值与单测对照；它不是 Hive2 轮式模型。

### `g3_pipeline_core.G3HistoricalCoordinator`

**默认真实 Bag 研究链**改为明确调用：
```python
solve_discrete_mpc_cbf(
    historical_path, historical_body_pose, moving_hypotheses,
    max_speed=0.75, max_accel=0.5,
    control_lattice="acceleration_increment"
)
```

数值由已审计 Hive2 `robot_motion.yaml` / SCAN 参数契约引用，且不会超过两者的**线速度和线加速度配置**。每一帧保留独立 `hive2_motion_limit_screen`：如果最终四位小数 XY 候选被检查为超出 Hive2 或 SCAN 平移约束，将`candidate_xy_m=[]`、`status=INFEASIBLE_CONFIG_BOUNDS_SCREEN_REJECTED`，不向**仅作研究展示的** `/n01/offline/g3_candidate_path` 发布该条候选。

当离散 CBF 与限幅求不出满足条件的数值候选，`INFEASIBLE_...` 及空轨迹是**正确的拒绝结果**；严禁为了增加“通过帧数”重新放宽安全/配置数值。

### `g3_frame_evidence.py`

逐帧证据照旧独立记录：原始 Bag 扫描时刻、POST-HOC 标志、G3 数值状态、独立 Hive2 约束复检、拒绝/超限计数；保留零控制输出审计。对有数值求解但无法生成可展示候选的帧，也按真正的求解尝试计数，不误称“可执行轨迹”。

## 3. 已有自动回归检查（无 ROS 与底盘）

专门新增 `test_g34_bounded_generation.py`：
- 静态无动态障碍路径上，限幅求解**必须产生非零正向进展**，不能靠全零轨迹假装满足加速度；
- 解的离散速度与加速度符合 SCAN 0.75m/s 与 0.5m/s²（含四位小数离散化容差）；
- 离散 CBF 仍确实检查运动候选障碍；障碍约束不可满足时保留 INFEASIBLE，而非放出违反限值的路径；
- 注入伪造超限求解结果，确认实际 Coordinator 在发布前清空该候选；
- 历史时间戳必须精确配对，错误路径和不同时间戳不产生可执行假象；
- 旧研究求解器仍可作为可重复对照，**当前运行链的 G3.4 参数才是新选择**。

运行命令：
```bash
python3 -m unittest discover -s src/n01_offline_integration/test -p 'test_g34_bounded_generation.py' -v
```

## 4. 限制：这还不是四轮转向运动学，也没有实机轨迹执行许可

`p[k+1]=p[k]+v[k]*dt` 仍是**平面全向点质点运动学代理**。当前 Bag 桥接的 `/body_pose` 数据没有真实 `twist`，首步初速零是**求解假设**，不是测量事实。模型还没有完成四个 ankle_yaw 转向角、轮速、车体 yaw、控制器换向策略、转向速度/jerk、真实制动与轮胎侧滑约束；没有静态全身碰撞、地面高度函数及未知区通行性验证。

因此，即使 `hive2_motion_limit_screen.status=TRANSLATIONAL_BOUNDS_WITHIN_CONFIG_4WS_UNVERIFIED`，也只代表**有一定配置根据的二维平移必要条件**，绝非可实际驾驶的轨迹。完整轮速/转向分配需要实际 `motion_controller/ChassisController` 实现或同等可验证接口，不可把 `yaw_soft_limit_rad=π` 简单当作真实 `ankle_yaw` 可越过 URDF ±π/2 的许可。

安全标志永远：`safety_authorization=NEVER`、`actuator_output=DISABLED`。不能连接 `/cmd_vel`、`/chassis_controller/rt_command` 或其他电机接口。

## 5. 下一次测试只有一次集中 ROS 历史 Bag 验收

待用户端 ROS 2 Humble 代码同步、确保该电脑与真实运动栈物理隔离后：
```bash
cd /home/demo/lyy/hivecore-navigation
git -c http.version=HTTP/1.1 pull --ff-only origin main
bash scripts/run_n01_bag_functional.sh
```

本命令沿用已有严格隔离环境（`ROS_DOMAIN_ID=88`、`ROS_LOCALHOST_ONLY=1`、`N01_HISTORICAL_BAG_ONLY=1`）与进程归属清理，不重播原始控制指令；并按原本流程生成 `n01_bag_functional_*.tar.gz`。**用户端若有 git 本地未提交改动，应先按需备份，不运行强制覆盖指令。**

该轮验收必须重点看：
- `G3.4` 源码版本、`candidate_generation_profile=HIVE2_SCAN_CONFIG_BOUNDED_XY_ONLY`、`control_lattice=acceleration_increment`；
- Bag 928 帧全部被读取并分类成功/缺 TF 拒绝（接受量不编造目标数）；
- 实际 G3 数值求解帧、可展示 candidate 帧、CBF 约束帧、`INFEASIBLE` 帧；
- `hive2_chassis_limits_exceeded_frames=0` 且 `hive2_scan_limits_exceeded_frames=0`，任何边界超限必须已清空其候选；
- `g3_frame_evidence.jsonl` 每一求解有离线审计且与报告数量可追溯；
- `/cmd_vel` 发布者零、退出时进程与 ROS 图清理 CLEAN；
- 不能把非零 candidate 帧数或 SCAN B-spline 数量写成 4WS 或实机安全通过。

若本轮首次未通过，只修正**明确报告的原因**并运行对应纯单元测试；无需从零重做 Bag/PCD/URDF 参数采集。

完成 G3.4 后，按既定总体计划进入**第二离线阶段：G4.9 PCD 地面支撑与整机碰撞/SCAN 三维高度语义统一**。只有地面可识别、通行性证据充分时才允许从 Bag 起点的 8.036m / 29 点 XY 研究线升级成三维 SCAN 引导路线。
