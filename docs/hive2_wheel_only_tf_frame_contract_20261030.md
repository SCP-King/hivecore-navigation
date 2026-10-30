# Hive2 N-01：25 帧 TF 关系与固定腿姿态轮式导航开发契约（2026-10-30）

> **性质：项目已确认的接口资料及开发工况，不是实机启用许可，也不是新的可执行 ROS 配置。**
>
> 证据来源：用户对录制 Bag 的 `/tf`、`/tf_static` 提取结果，甲方 URDF 构型以及当前 SLAM/预处理帧语义。**本次提交没有再次读取原始录制 Bag 和甲方安装目录**。其中的发布者归属、关节拓扑以及“腿姿态已经被锁定”的实际运行状态，仍须分别用当前 URDF/关节反馈核对。
>
> 对应机器可读版：[config/reference/hive2_wheel_only_tf_contract_20261030.json](../config/reference/hive2_wheel_only_tf_contract_20261030.json)。

## 1. 当前研究工况：四轮驱动、腿关节姿态保持不变

用户明确指定：**现阶段只做轮式行驶导航**，四条腿保持固定工作姿态，**不规划、不执行抬腿、迈步、足式步态或主动调高/跨越**。不应因为硬件叫“轮足机器人”而默认开发足式规划和步态控制。后续 G1–G4 算法讨论、指标和测试以轮式工况为目标；当前历史 Bag/SCAN/G2/G3 仍为**离线只读，不接任何轮驱或腿驱运动指令**。

- `base_footprint` 是当前历史地图定位/平面路径锚点。地图目标和占据地图统一使用 `map`；`base_link` 是机体几何参考，`lidar_link` 是过滤点云坐标系。
- 腿部关节 TF **不参与导航定位主链和滤波点云 `lidar_link → map` 变换**；但**固定姿态的腿、轮、外壳仍然属于机器人外形碰撞占据范围**。必须以真实固定关节角、运动姿态和外廓实测构建完整足轮机体包络，不能只使用 `base_link` 的 0.55×0.26×0.12m 盒子。
- `腿姿态固定` 是用户确定的**研发工况要求**，不是当前代码已验证的关节抱闸/位置保持；尚缺腿关节目标与反馈、实际轮式底盘转向类型和 wheel kinematics。不得将“轮式”自动写成差速、阿克曼或全向模型。
- 当前 G3 `PLANAR_HOLONOMIC_POINT_MASS_UNVERIFIED_FOR_HIVE2` 为研究数值近似；固定腿模式**并不自动使它成为真实轮式动力学模型**。

## 2. 甲方所报告的 TF 树（共 25 个 frame）

```text
map
└─ base_footprint                       动态定位；SLAM/建图
   └─ base_link                          Hive2 选定静态安装 Z=+0.900m
      ├─ lidar_link                      静态平移 (+0.295, 0, 0)m
      │  └─ imu_link                     0 平移，用户报告约 10.8° 俯仰旋转
      │     └─ livox_frame               静态 0 平移，单位旋转
      ├─ lpms_imu_link                   0 平移，LPMS 硬件 IMU
      ├─ fl_hip_link → fl_thigh_link → fl_shank_link → fl_foot_link
      ├─ fr_hip_link → fr_thigh_link → fr_shank_link → fr_foot_link
      ├─ rl_hip_link → rl_thigh_link → rl_shank_link → rl_foot_link
      └─ rr_hip_link → rr_thigh_link → rr_shank_link → rr_foot_link

fixed_frame                              独立、未接入 map 的根
└─ imu_filtered                          互补滤波后的 IMU 输出
```

其中 7 个导航关键帧为 `map`、`base_footprint`、`base_link`、`lidar_link`、`imu_link`、`livox_frame`、`lpms_imu_link`。四条腿各有 hip/thigh/shank/foot 共 16 帧，另有 `fixed_frame` 与 `imu_filtered` 两帧，共 **25 帧**。

**腿链精确 joint 类型有待 URDF 原始定义复核**：用户提供的框图一处描述了 `base_link → hip_link` 固定安装，而表格将 hip 列为动态；末端 `foot_link` 的文字又存在“相对 thigh”与“经 shank”两种描述。本契约只确认用户提供的命名链和**暂报**的 `shank_link → foot_link` (0,0,-0.48)m，不将这些潜在矛盾直接写入真实 TF 广播实现。

## 3. 逐帧角色与导航使用范围

| 范围 | Frame | 主职责 | 当前导航用法 |
|---|---|---|---|
| 地图与定位 | `map → base_footprint` | SLAM 动态定位（用户报告 Bag 内该边约 11 条） | **必需**，严格用点云采集时刻 |
| 机体安装 | `base_footprint → base_link` | Hive2 URDF 选定 Z=+0.90m | **必需**；冲突见下 |
| 雷达安装 | `base_link → lidar_link` | (+0.295,0,0)m | **必需** |
| Livox IMU | `lidar_link → imu_link → livox_frame` | 原始激光与定位 IMU 静态对齐 | 原始数据链必需，预处理后的过滤云不重复变换 |
| LPMS | `base_link → lpms_imu_link` | 独立 LPMS IMU，同机体原点 | 按定位消息设计使用，不能与 `imu_link` 混称 |
| 四条腿 | `{fl,fr,rl,rr}_{hip,thigh,shank,foot}_link` | 关节链和固定姿态的碰撞形状 | **不作为地图定位/点云 TF 主链**，仍参与机体包络审计 |
| 独立滤波链 | `fixed_frame → imu_filtered` | 互补滤波 IMU，用户报告 Bag 内约 296 条 | **不能凭空连接到 map** |

当前记录的关键固定安装（单位 m）：

| TF 边 | 平移 (x,y,z) | 旋转 |
|---|---|---|
| `base_footprint → base_link` | (0,0,+0.900) | 用户所选定 Hive2 名义安装 |
| `base_link → lidar_link` | (+0.295,0,0) | 名义零旋转 |
| `base_link → lpms_imu_link` | (0,0,0) | 待原始 URDF/Bag 逐项对齐 |
| `lidar_link → imu_link` | (0,0,0) | 用户报告约 10.8° 俯仰，**符号和精确四元数待核对** |
| `imu_link → livox_frame` | (0,0,0) | 单位旋转 |
| `{leg}_shank_link → {leg}_foot_link` | (0,0,-0.48) | 用户提供的名义关节几何，待 URDF 核实 |

**不能忘记冲突**：用户发现同一录制 Bag 曾包含 `base_footprint → base_link` 的 0.30m 和 0.90m 两种定义；本次 Hive2 研究选择使用 0.90m，但没有因此消除在线可能同时运行的两个 `robot_state_publisher`。实机前必须核对 SLAM 的 `launch_robot_state_publisher:=false` 生效，以及运动栈该 TF 边唯一权威。

## 4. 点云→地图路径：只用采样时刻的真实 TF

```text
/ livox/lidar                 header.frame_id = livox_frame   原始 Livox
      ↓ 甲方预处理与外参（不能只修改 frame_id）
/ livox/lidar/geo_filtered    header.frame_id = lidar_link   用户报告抽取的 100 帧均一致
      ↓ T_map_lidar_link(header.stamp)：
      map ← base_footprint ← base_link ← lidar_link
/ cloud                        header.frame_id = map          进入地图和 SCAN（仅受控输入）
```

- 用户提供的 Bag TF 审计结论：录制链里**没有 `odom` 帧/边**，定位是 `map → base_footprint` 直接发布。不要假定 `map→odom→base_link`，也不要把 `odom` 和 `map` 当作别名。未来其他录制环境仍应独立校验。
- 当前 `src/n01_bringup/config/n01_sensor.yaml` 原先写 `target_frame: odom`，与这份会话的 TF 证据不符。当前已将这份**只读接口 YAML 的期望目标帧**更正为 `map`；独立 `sensor_only.launch.py` 仍使用 `lidar_link` 作为无全局 TF 的默认查看帧。两种模式不能混用。
- 原生 SCAN `planner.yaml` 和 G1 合成桥中的 `world` 是离线合成场景默认值，真实 Bag/G4 离线 launcher 有明确 `map` 覆盖。**不**因此直接全局批量替换 G1 的 `world`。
- 过滤云的 header 已经是 `lidar_link`，不要再从 `livox_frame` 额外执行一遍转换；原始云需要按其自己的 TF 与预处理条件检查。
- 路径上的 `imu_filtered` 是独立的 `fixed_frame` 坐标树，未经标定不能当 `map` 参考。

## 5. 轮式固定腿模式的实现边界与剩余参数

| 研发项 | 已确定 | 仍未确认 |
|---|---|---|
| locomotion mode | **轮式驱动，腿姿态固定** | 如何可靠保持、腿角目标/关节反馈与上电恢复 |
| 地图定位 | `map → base_footprint` | 最新 LocalizationStatus 类型、在线 TF 唯一权威 |
| 过滤云 | `lidar_link` → `map`（扫描时间戳） | 实时雷达→map 采样延迟与一致性 |
| IMU | 两套 IMU 有独立帧 | 精确旋转和融合/优先级决策 |
| 动态模型 | 研究中 G3 为二维质点模型 | **轮式转向运动学、轮距/轴距、角速度/加速度/制动能力** |
| 碰撞模型 | 四腿轮子虽然不动也有外廓 | **固定姿态真实关节角、最大宽度/长度/高度、离地间隙** |
| 地面 | PCD 与 Bag 同 map（用户确认） | `ground_z_map(x,y)`、地面支撑、可通行性 |
| 安全 | N-01 速度上限 1m/s（需求） | 安全距离、动态停车、机械避障约束 |

**不在本次变更中**：启动底盘/SLAM、修改关节控制、发送 Twist、把 G3 轨迹变成轮式控制指令、更改 SCAN 几何安全距离、认定地面高度或者宣称在线 TF 双发布已解决。

## 6. 记录与更新原则

机器可读契约是 **reference-only**，不是 ROS 参数文件，也不会被 launch 自动装入。后续拿到真实轮式工作姿态关节角、URDF/底盘运动类型与现场测量证据后，再把未确认字段逐项升级为实测，且保留证据来源和适用日期。

这次提交只做**正确的坐标语义和轮式开发范围约束**，不需要额外运行 Bag 才能记录已提供的 frame 结构；ROS 图/全身碰撞/地面数据仍保留为不同的待验收问题。
