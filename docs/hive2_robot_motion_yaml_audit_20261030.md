# Hive2 `robot_motion.yaml` 双副本源码审计与 N-01 轮式规划约束（2026-10-30）

**已审计真实上传文件**：`hive2_robot_motion_configs.tar.gz`；仅分析 YAML 文本，不启动 ROS、底盘、EtherCAT、轮/腿控制器，不修改甲方配置。本页是**配置与研发约束审计**，不等于配置已经被运行态激活，更不是实机安全认证。

- 安装副本：`/opt/hivecore/hive2-motion/1.0.0/install/share/hive2_driver/config/robot_motion.yaml`。
- 配置副本：`/opt/hivecore/hive2-motion/1.0.0/config/hive2_driver/robot_motion.yaml`。
- 两者各 **7,000 字节、218 行**，SHA-256 都为 `3f19abe0a65e02b331a0d51c18ec0503278a4d9a6b942332b47c4f7d434584f0`，内容完全一致。
- **仍未核实** `hive2_driver.launch.py` 实际加载哪个绝对路径，以及是否有命令行参数覆盖。因为两份当前字节相同，本次不存在双副本差异导致的数值冲突。

对应结构化、只读数据：[hive2_robot_motion_audit_20261030.json](../config/reference/hive2_robot_motion_audit_20261030.json)。不将甲方完整私有配置复制到仓库。

## 1. 四轮转向运动架构：YAML 已明确绑定

| 角色 | YAML 关节名 | 约束 |
|---|---|---|
| 四轮驱动 | `wheel_fl`、`wheel_fr`、`wheel_rl`、`wheel_rr` | 独立轮驱 `chassis_controller.wheel_joints` |
| 四轮转向 | `fl_ankle_yaw`、`fr_ankle_yaw`、`rl_ankle_yaw`、`rr_ankle_yaw` | `chassis_controller.yaw_joints` |
| 固定腿 pitch | 四腿共 8 个 `*_leg_pitch1/2` | `legs_position_controller` 和 `legs_lift_controller` 的关节组 |
| 控制器 | `motion_controller/ChassisController` | `/chassis_controller/rt_command`、`/chassis_controller/rt_status` |
| 控制周期配置 | `controller_manager.ros__parameters.update_rate: 1000` | 1000 Hz **配置值**，不代表已测到 1000 Hz 实际循环 |
| 命令超时 | `controllers.chassis_controller.command_timeout_s: 0.2` | RT 命令时效配置，不等价于已证明紧急制动距离 |

**工况建议**：当前需求是“腿 pitch1/pitch2 在选定工作姿态保持、不执行步态或抬腿；四个 ankle_yaw 允许转向，四个 wheel 允许旋转”。**不可以**笼统写成“所有腿关节不动”，否则会把车辆转向关节一起锁死。

`spawn.controllers` 含 `chassis_controller` 和 `legs_lift_controller`；`deferred_activate` 含 `chassis_controller` 与 `wheels_gpio_controller`；`inactive` 列出 `legs_lift_controller`。在 `legs_lift_controller` 参数块里 `balance.enabled=true`，但它初始配置为 inactive，**不能据此断言 IMU 平衡当前运行或已经关闭**。最终是否一直保持固定 pitch 必须看控制器激活顺序及关节反馈。

## 2. 确认的配置参数（不要与实测动力学混淆）

| 类别 | YAML 键 | 值 | 注意 |
|---|---|---:|---|
| 几何 | `chassis.wheel_radius` | 0.095 m | 与 Xacro 名义轮半径一致 |
| 几何 | `chassis.wheel_base_x` | 0.56 m | 控制器输入参数 |
| 几何 | `chassis.wheel_base_y` | 0.27 m | 控制器输入参数 |
| 几何 | `chassis.delta_y` | 0.085 m | 轮/踝布局偏置 |
| 车体速度 | `chassis.max_linear_speed` | 2.0 m/s | **底盘配置上限 ≠ N-01 导航许可速度** |
| 转向相关 | `chassis.omega_per_linear` | 1.25 | 比例参数，**不是**已经批准的最大角速度 |
| 线加速度 | `chassis.max_linear_acceleration` | 1.0 m/s² | 最大配置值 |
| 线加速度 | `chassis.linear_acceleration` | 0.6 m/s² | 常规/标称配置值 |
| 角加速度 | `chassis.max_angular_acceleration` | 2.0 rad/s² | 最大配置值 |
| 角加速度 | `chassis.angular_acceleration` | 0.7 rad/s² | 标称配置值 |
| 线 jerk | `chassis.max_linear_jerk` | 10.0 m/s³ | 最大配置值 |
| 线 jerk | `chassis.linear_jerk` | 6.0 m/s³ | 标称配置值 |
| 角 jerk | `chassis.max_angular_jerk` | 20.0 rad/s³ | 最大配置值 |
| 角 jerk | `chassis.angular_jerk` | 7.0 rad/s³ | 标称配置值 |
| 转向 | `chassis.steer.speed` / `max_speed` | 1.2 / 1.57 | 控制器内部速度配置；实际语义待源码核对 |
| 转向 | `chassis.steer.acceleration` / `max_acceleration` | 2.0 / 4.5 | 内部加速度配置 |
| 转向 | `chassis.steer.jerk` / `max_jerk` | 20.0 / 45.0 | 内部 jerk 配置 |
| 轮驱 | `chassis.drive.max_speed` | 20.0 | 配置原值；未从 C++ 确认精确单位与限幅位置 |
| 转向反馈 | `chassis.drive.steer_angle_source` | `feedback` | 使用反馈作为方向信息的配置意图 |
| 开环/硬保护 | `chassis.hard_mode.enable` | `true` | 不等同已通过实际刹车/碰撞安全测试 |
| 轮足补偿 | `chassis.wheel_leg.enabled` | `false` | 与固定 pitch 轮式研究方向一致 |
| 腿升降模块 | `legs_lift_controller.wheel_leg.enabled` | `false` | 不能视为关节已锁定 |
| IMU 轮式里程计 | `chassis.odom.imu_fusion` | `true` | LPMS IMU，非 SLAM 已定位证据 |
| TF 边 | `chassis.odom.publish_tf` | `false` | **不发布 odom TF** 配置；不意味着 `/odom` 话题不存在 |

此外配置 `chassis.wheel_adm.enabled=true` 和全局过流保护参数（移动电流限制 4A、过流进入 15A / 退出 12A）。这些参数是底层电机保护逻辑的配置，**不能**直接替代药厂导航的动态障碍安全距离、制动距离或停车验收。

## 3. 与当前 G1 / SCAN / G3 的直接差异

| 模块 | 当前数值或模型 | 真实底盘 YAML | 研发判断 |
|---|---|---|---|
| N-01 需求限速 | ≤ 1.0m/s | `chassis.max_linear_speed=2.0m/s` | 应保持导航需求限速；底盘上限不是导航目标 |
| SCAN | `manager.max_vel=0.75m/s`、`manager.max_acc=0.5m/s²` | 2.0m/s、常规 0.6 / 最大 1.0m/s² | 数字上不超过配置的线速度和线加速度，但**不证明轨迹可由四轮转向执行** |
| G3 `solve_discrete_mpc_cbf` | 平面全向点质点、`max_speed=0.75m/s`、`max_accel=3.2m/s²` | 四轮独立驱动与四轮转向、`max_linear_acceleration=1.0m/s²` | **确定存在加速度配置不匹配**；G3 仅为离线算法样例 |
| G3 运动自由度 | 任意 `(v_x,v_y)` 离散候选集 | ankle_yaw、wheel 驱动/反馈、转向速率、轮速及模式转换 | **不能**将 G3 输出直接交给底盘；缺转向过渡和执行可行性约束 |
| 碰撞几何 | SCAN 双圆柱及 G3 研究净空 | 29-link Xacro，四轮和固定姿态的腿 | 仍须根据实际固定 pitch 姿态构建包络 |

特别说明：`chassis.omega_per_linear=1.25` 不能作为 `max_angular_velocity_radps` 直接写入 MPC；`chassis.wheel_adm.max_omega_rad_s=2.5` 是轮电流/导纳模块中的局部参数，**不是**机体偏航角速度上限。

## 4. 一项源码语义需要专项核对：转向角限制

从前一包 `quad_leg_kinematics.xacro` 已确认 `*_ankle_yaw` 的 URDF joint limit 为 **[-1.5708,+1.5708] rad**。

本次 YAML `chassis.steer.yaw_soft_limit_rad=3.141592653589793`（约 ±π，具体限制解释依赖代码）。两者并非一致的角度数值，但可能是控制器内部对转向角作 `θ + π` 等价变换并反转轮速，也可能有额外硬限幅。

**在未检查 `motion_controller/ChassisController` 的轮速/角度分配实现前，不推断它一定会越过关节限位，也不声称它已经安全。** 日后必须保证最终送到 `ankle_yaw` 的目标/反馈满足有效的物理关节限制。

## 5. 与 SLAM 的集成：两个 TF 开关不是一回事

- 本次底盘 YAML：`chassis.odom.publish_tf=false` —— 不让**底盘控制器**发 `odom` TF，但不禁止里程计消息；
- 历史 Hive2 启动脚本：`HIVE2_SLAM_MAPPING=1` 向启动参数传 `odom_publish_tf:=false`；
- 当前 SLAM：`launch_robot_state_publisher=true` 的默认行为可能另发 `base_footprint→base_link=0.30m`，与 Hive2 的 0.90m 冲突。

**因此底盘 odom TF 关闭不等于 SLAM 的 0.30m 重复 RSP 关闭。** 集成设计仍应让 Hive2 RSP 对机体安装、腿轮动态关节负责，SLAM 仅对 `map→base_footprint` 有权；在线发布者唯一性尚无验收证据。

## 6. 下一步按现有证据直接开展哪些工作

1. **软件设计不必等待继续收集基本几何**：四轮独立驱动、四轮 ankle_yaw 转向、wheel radius / x,y base / delta_y、配置速度、加速度、jerk、状态接口已知。
2. 可编写独立**离线的四轮转向运动约束研究模型**，输入配置限幅与路径曲率/转向角/轮速约束，输出候选可行性诊断，而不对 `/cmd_vel` 或 `/chassis_controller/rt_command` 写入。
3. 真正要精确复现甲方控制器的四轮转向映射或验证转向角 ±π 等价处理，需要 `ChassisController` 的 C++ 实现/最终参数覆盖；本文件自身不包含算法公式，**不能凭 YAML 断言控制模式、转向模式切换或动态防滑已校准**。
4. 固定 pitch 工作姿态尚未用 `joint_states` 证明，必须区分 MoveHome 设定值、实际保持值及 `legs_lift_controller` 的激活状态；最终三维碰撞仍需真实地面 `ground_z_map(x,y)` 和完整外廓验证。

**安全边界**：本轮仅是只读源码审计；历史 Bag 算法环境保持隔离，`safety_authorization=NEVER`、`actuator_output=DISABLED`，不修改底盘/SLAM 的任何运行配置，也不声称已完成实时闭环或硬件运动验收。
