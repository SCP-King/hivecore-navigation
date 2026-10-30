# Hive2 固定腿 pitch、四轮独立驱动与踝部转向：TF/机体参考模型（2026-10-30）

> **2026-10-30 历史建图 TF 修订**：用户检查原始 Bag 时间轴，报告 /tf_static 中 +0.90m 有 1 条、/tf 中 +0.30m 有 866 条。旧 PCD 低表面约 map Z=-0.50～-0.53m，与 raw 点云经 +0.30m 链后低簇约 -0.61m 一致性较强。**原始 PCD 应优先按 +0.30m 历史约定研究，而 +0.90m 是未来 Hive2 运动 URDF 权威。** 一条静态 TF 不等于只在一瞬间生效，具体 TF 查询仍需源码/在线证据。详见 [G4.12 历史地图与底盘 TF 高度关系](n01_g412_historical_mapping_030_vs_motion_090.md)。

**资料性质**：只读研究参考（不是 ROS 运行参数、实机启用或 TF 在线验收）。本版已直接检查用户上传的 `hive2_slam_sources_20261030_155649.tar.gz`（19 个文件）；以实际 Xacro/YAML/消息定义修正了上一版仅依据 Bag frame 名称推断的若干错误。用户提供的历史 Bag TF 数量及运行情况未在这次重新播放验证。

机器可读主档案：[hive2_wheel_only_tf_contract_20261030.json](../config/reference/hive2_wheel_only_tf_contract_20261030.json)。

## 1. 现阶段工作模式：固定 pitch 姿态，车轮行驶，但 ankle_yaw 可以转向

**固定的是四条腿的 pitch1、pitch2 工作姿态，不是锁死一切腿部关节。** 源码 `quad_leg_kinematics.xacro` 已核实：每腿有 pitch1 / pitch2 / ankle_yaw 三个旋转关节，末端还有一个连续 wheel 旋转关节。四个 `wheel_fl, wheel_fr, wheel_rl, wheel_rr` 独立驱动；四个 `fl/fr/rl/rr_ankle_yaw` 可绕局部 Z 轴转向，硬限位约 ±1.5708 rad。

本阶段不做迈步、跨台阶、主动抬腿或姿态升降；但不能因此禁用轮式转向所必需的 ankle_yaw。`hive2_driver/config/robot_motion.yaml` 的两份副本已于 2026-10-30 读取并证实相同；轮径、四轮几何、速度/加速度/jerk 和转向约束配置已确认。真实轮速/转向分配算法仍待 `motion_controller/ChassisController` 实现核实；不能自动假定阿克曼/差速/麦克纳姆/任意全向控制。

`hive2_leg_parameters.yaml` 记录 MoveHome 的 pitch1=+0.785398rad、pitch2=-1.308997rad（每腿重复），这只是**配置的 Home 目标值**，不是目前车辆真实保持的角度或已接地姿态；`sensor_balance.enabled: true` 和 `body_lift.trajectory_execution_enabled: true` 也意味着仅说“腿不动”**不会自动关闭平衡/升降模块**。开发阶段的禁用/保持策略要依据真实控制权和反馈设计，不能直接替甲方发指令。

## 2. 源码明确的 link 树（29 个名称；不保证 29 个运行时 TF 全部发出）

```text
map                                            由 SLAM 实际定位时发布的动态根
└─ base_footprint                              直接定位 tracking frame
   └─ base_link                                选定 Hive2 URDF 固定 +0.900m
      ├─ lidar_link                            +0.295, 0, 0
      │  └─ imu_link                           用户报告的静态 TF，约 10.8° 俯仰
      │     └─ livox_frame                     用户报告的静态 TF
      ├─ lpms_imu_link                         hive2.config.xacro 中条件启用的固定 IMU
      ├─ fl_hip_link                           fl_leg_pitch1: revolute Y
      │  └─ fl_thigh_link                      fl_leg_pitch2: revolute Y
      │     ├─ fl_foot_link                    kdl_foot_fixed，固定代理足端（IK）
      │     └─ fl_shank_link                   fl_ankle_yaw: revolute Z（转向）
      │        └─ fl_wheel_link                wheel_fl: continuous Y（驱动）
      ├─ fr_hip_link → fr_thigh_link → {fr_foot_link, fr_shank_link → fr_wheel_link}
      ├─ rl_hip_link → rl_thigh_link → {rl_foot_link, rl_shank_link → rl_wheel_link}
      └─ rr_hip_link → rr_thigh_link → {rr_foot_link, rr_shank_link → rr_wheel_link}

fixed_frame                                    独立 IMU 滤波树（来自用户历史 Bag 审计）
└─ imu_filtered
```

核心命名：地图/机身/雷达/双 IMU 7 个；四条腿每腿 **5 个**（hip、thigh、shank、foot、wheel）共 20 个；滤波链 2 个，总共 **29 个**。此前 `25 个` 的清单少了 4 个 wheel_link；此前将 `foot_link` 画在 shank 下也不正确。机器可读契约已按 Xacro 修正。

**特别注意**：`*_foot_link` 是从 `*_thigh_link` 以 `(0,0,-0.48)m` 固定连接的**KDL 足端代理**，而**真正轮子**是从 `*_shank_link` 经过 `wheel_*` continuous joint 到 `*_wheel_link`。碰撞检查须使用轮子实际碰撞模型，不能把虚拟 foot 当成接地轮。

## 3. 已在原始 Xacro 中核对的固定安装、轮腿尺寸

| 名称 | 源码中定义 | 含义 |
|---|---|---|
| `base_footprint → base_link` | (0,0,+0.900)m | Hive2 运动描述的选定安装；实际地面高度仍未知 |
| `base_link → lidar_link` | (+0.295,0,0)m | Livox 过滤云 frame |
| `base_link → lpms_imu_link` | (0,0,0)m | `hive2.config.xacro` 受 `imu_enabled` 影响，默认启用 |
| 机体 collision box | 0.55×0.26×0.12m | 仅 `base_link`，不是含腿/轮/外壳的总包络 |
| 四条腿髋关节中心 | x=±0.28m，y=±0.135m | 名义 pivot 纵距 0.56m，横距 0.27m |
| pitch1 轴到 pitch2 轴 | 0.38m | 大腿段 |
| pitch2 轴到 ankle yaw 轴 | 0.38m | 小腿段 |
| ankle yaw 到轮心 | 0.10m | 踝下段 |
| 轮半径 / 轮宽 | 0.095m / 0.04m | Xacro 定义为长方体碰撞代理，并非精确胎形 |
| ankle_yaw 横向偏移 | 左 -0.085m，右 +0.085m | 轮子外偏量抵消对应偏移 |
| ankle_yaw 转角范围 | [-1.5708,+1.5708]rad | 转向限位，不代表已验证实时运动能力 |

`lidar_link → imu_link → livox_frame` 的静态关系、`map→base_footprint`、`fixed_frame→imu_filtered` 不是以上 Hive2 机体 Xacro 全部生成的；依赖用户提供的实际 Bag 与 SLAM 预处理配置报告，尚未在本包独立读到静态 TF 发布器的参数文件。

## 4. SLAM 与底盘必须如何划分 TF

**运动栈是机体 TF 的权威；SLAM 是地图定位的权威。**

- 运动栈：`base_footprint→base_link` 固定 +0.90m、`base_link→lidar_link`、`base_link→lpms_imu_link`、随关节反馈更新腿轮 TF。
- SLAM：定位有效时 `map→base_footprint`。此接口不经 `odom`，这是**当前用户所报告的这份录制 Bag**，不等于运动栈没有 `/odom` 功能。
- 预处理：输出 `/livox/lidar/geo_filtered` 的 `header.frame_id=lidar_link`；从扫描的 `header.stamp` 求 `T_map_lidar_link`，不得仅改 frame_id。
- 运动控制的 `/odom` **确实存在单独的接口配置**：`hive2_chassis_params.yaml` 的 `chassis_controller` 配置委托、Web 配置的 `odom_topic: /odom`；`start_hive2_motion.sh` 的 `HIVE2_SLAM_MAPPING=1` 会向 `hive2_controller.launch.py` 传 `odom_publish_tf:=false`。这**不是**关闭 SLAM 的 0.30m 重复 RSP。

冲突的源码证据：
`hivecore_sensor_frames.xacro` 显式写 `hive2_footprint_xyz="0 0 0.90"`；
SLAM `hivecore_robot.urdf.xacro` 默认 `footprint_xyz="0 0 0.3"`；
SLAM `console_slam3d_bringup.py` 的 `launch_robot_state_publisher` 默认 `true`。
用户的历史 Bag 同时曾看到 0.30 / 0.90 两种定义；当前不能宣称运行态冲突已解除。

推荐由**甲方的启动集成配置**显式设定 `launch_robot_state_publisher:=false`，让 Hive2 运动 RSP 独占机体链接；是否生效仍要核对最终实际启动与 ROS 图，不能擅改 SLAM 简化 URDF 来掩盖双发布。

## 5. 定位状态字段：`LocalizationStatus.msg` 已获得源码

安装路径 `slam_release/install/hivecore_slam3d/share/hivecore_slam3d/msg/LocalizationStatus.msg`。用于在线严格状态检查的真实**wire 字段**为：

| 字段 | 期望或含义 |
|---|---|
| `control_signal_outer_role` | 字符串 `"localization"`（**小写**） |
| `control_signal_session_mode` | 字符串 `"tracking"` |
| `registration_output_tf_published` | bool `true` |
| `registration_output_pose_published` | bool；可用于进一步证据 |
| `registration_output_tracking_compose_ok` | bool；可用于进一步证据 |
| `registration_output_localization_mode` | uint8，`MODE_TRACKING=0` |
| `registration_output_pose_source` | uint8，`POSE_SOURCE_MEASURED=2` 等枚举 |
| `pipeline_scan_frame_match` | bool，表示预处理扫描帧匹配 |
| `header` | 原始消息时间戳，应与 ROS 时间基准和消息新鲜度区分 |

此前仓库 `offline_bridge.yaml` 的 `outer_path` / `inner_path` 留空，是因为未拿到源码。现在已能根据真实消息名填写，且**仍需保持** `slam_rsp_disabled_confirmed=false`、无动作输出、扫描时刻 TF / 严格状态的新鲜度和失败即阻断。

## 6. 底盘轮式控制：哪些已找到，哪些仍缺一个核心文件

源 `hive2_chassis_params.yaml` 注释明确：轮式运动学、OTG、各类转向模式、`chassis_controller` 参数以 **`hive2_driver/config/robot_motion.yaml` 为唯一真源**；这份文件**没有被本次按文件名搜索的压缩包收集**。

已核对的外围接口：

- 监督器 `steer_drive_node`：`/cmd_vel`（仅记录，当前**严禁发送**），50Hz，超时 0.2s；内部下发 `/chassis_controller/rt_command`，`/chassis_state` 和 `/chassis_dec_stop/state` 为诊断状态。
- 转向关节：`fl/fr/rl/rr_ankle_yaw`。车轮：`wheel_fl/fr/rl/rr`。反馈含 `/joint_states`、`/wheels_can/joint_state`、`/chassis_controller/rt_status`。
- `hive2_leg_parameters.yaml`：MoveHome pitch1=+45°、pitch2=-75°；反馈 `/joint_states`；`sensor_balance.enabled=true`。这不等于固定姿态已经保持不变。
- `wheels_can_controllers.yaml`：控制器类型 `forward_command_controller/ForwardCommandController` 的 velocity interface；文件注释写 250Hz，而实际 `controller_manager.ros__parameters.update_rate` 写 500Hz。必须以最终启用配置为准，不能凭注释推断实际控制周期。
- `hive2.config.xacro` 默认 EtherCAT `ec_freq=1000`，轮组 USB2CAN 配置和四轮转向链已具备，但无需将它们导入导航层。

**已补齐**：`robot_motion.yaml` 的配置和安装副本均已读取（各 7,000 字节、218 行、SHA-256 完全相同）。详细参数和差异见 [Hive2 robot_motion 双副本审计](hive2_robot_motion_yaml_audit_20261030.md)。**剩余缺口不是这个 YAML，而是 `ChassisController` 轮速/转向分配实现与最终有效启动参数**；不能把 YAML 的 `chassis.steer.yaw_soft_limit_rad=π` 直接当作物理 yaw 关节可越过 Xacro ±π/2 限位的许可。

## 7. 后续开发方案（不运行实机）

1. 已知 Xacro 和消息定义可落库为可审计**只读模型**；旧 25-frame 契约和 TF 结构修正为 29。
2. 按源码修正仅离线使用的严格 LocalizationStatus 参数，不自动解除 `slam_rsp_disabled_confirmed=false` 门控。
3. `robot_motion.yaml` 的几何、转向限制及底盘速度/加速度/jerk 数值已明确；下一步可开展离线四轮转向**必要条件筛选**，但要完整实现与硬件一致的轮速/转向分配，仍需核对 `motion_controller/ChassisController` 的实现与实际参数覆盖。G3 当前 3.2 m/s² 加速度上限高于 Hive2 配置的 1.0 m/s²，已有明确模型差异。
4. 使用固定姿态**真实关节反馈**构建完整车体轮/腿碰撞包络，之后评估地图 `ground_z_map(x,y)`，不从 PCD 低分位数直接认定地面。
5. 后续只在物理隔离历史 Bag 中开展 SCAN/G2/G3 验收；未获安全批准以前禁止连接任何轮/腿运动命令。

**总括：源码已经足够支持“以 Hive2 运动模型为权威进行接口设计”，但不足以认定轮式控制律、实际接地姿态和实时闭环已经验证。**
