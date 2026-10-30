# Hive2 / SLAM / SCAN 参数基线（2026-10-29）

> **性质：只读参考档案，不是可加载的 ROS 运行参数。** 本文按用户于 2026-10-29 提供的 Hive2 运动栈、SLAM、URDF、现场测量和需求文档参数审计整理。当前 GitHub 仓库中已核对 G1 `planner.yaml`、`n01_sensor.yaml`、`isolated.yaml` 的相关值；**运动栈与 SLAM 侧原始配置文件并未随本次变更上传或逐项复核**，因此对应值标为“用户审计报告”，不可视为最终实机标定结论。

与本文对应的机器可读参数清单：[`config/reference/hive2_parameter_inventory_20261029.json`](../config/reference/hive2_parameter_inventory_20261029.json)。`null` 表示未确认，不应填写猜测数值。

## 1. 机器人型号与运动栈

| 项目 | 当前记录 | 证据来源 / 注意 |
| --- | --- | --- |
| 平台 | **Hive2，四轮足式底盘** | `/opt/hivecore/hive2-motion/1.0.0`、`hive2_driver`、`start_hive2_motion.sh`、四腿四轮 URDF；用户审计 |
| Hive3 | **单独配置，不得混用** | 不能把 Hive3 参数直接覆盖 Hive2 |
| 机身 `base_link` collision box | 长 **0.55 m** × 宽 **0.26 m** × 高 **0.12 m** | `quad_leg_kinematics.xacro`；**不是**整机最大外廓 |
| 髋关节中心 | x = ±0.28 m；y = ±0.135 m | 前后间距 **0.56 m**；左右间距 **0.27 m**；URDF、`hive2_chassis_params.yaml` 注释 |
| 大腿 / 小腿 / 踝下段 | 0.38 / 0.38 / 0.10 m | URDF 名义几何 |
| 轮胎 | 半径 0.095 m；直径 0.19 m；轮宽 0.04 m | URDF |
| **整机最大外廓** | **未确认** | 必须计入外壳、腿、轮及多姿态包络 |

## 2. 已选定高度 0.90 m，但 SLAM 最新默认仍发布 0.30 m（P0）

**已确认的配置选择（2026-10-29）：** Hive2 运动栈 `base_footprint → base_link` 目标高度 **z=+0.90 m**；不会再把 0.30 m 作为可选的 Hive2 正式参数。

**最新代码审计纠正（2026-10-26 构建版、用户只读检查 `/home/demo/latest/slam3d_release`）：** `preprocessing.launch.py` 和 `console_slam3d_bringup.py` 中 `launch_robot_state_publisher` **默认仍为 true**；`hivecore_robot.urdf.xacro` 的 `footprint_xyz="0 0 0.3"` **仍然是当前源码默认值**。因此“0.30 m 已停用”这一推断**不成立**：默认同时启动 SLAM 和 Hive2 运动栈，可能出现两家 RSP 发布同一 TF 边。

| 发布者 / 来源 | z 值 | 当前结论 |
| --- | ---: | --- |
| 运动栈 `hive2_driver` RSP，`hivecore_sensor_frames.xacro` | **0.90 m** | **选定的权威配置基准**；实际运行态尚未独立核实 |
| 最新 SLAM `mount_tf_publisher_node`（ROS 节点名 `robot_state_publisher`） | **0.30 m** | **旧值但仍默认启用**，需在 SLAM 启动时显式关闭 |
| 推荐 SLAM 启动覆盖 | `launch_robot_state_publisher:=false` | **下一次联调前必须确认生效**；这里仅记录，不代甲方改文件 |

`mount_tf_publisher_node` 使用节点名 `robot_state_publisher`，可能与运动栈 RSP 同名；即使 TF 值碰巧一致，同名节点与双权威也需要排查。用户审计还提到 `robot_state_publisher/set_enable_publish {data:false}` 运行时开关，可作为备选，但**不应替代可审计的启动参数配置**；实际服务类型、完整名称与生效行为应在对应版本确认后使用。

**解锁 P0-1 的条件**：SLAM 的 mount RSP 确认关闭；Hive2 运动栈 RSP 正常运行；实际 `base_footprint → base_link` 变换为 z=0.90 m，且只剩**一个该 TF 边的权威发布源**。仅统计整个 `/tf_static` 话题发布者数量**不足以证明单一权威**，因为同一话题还可能承载其他边，消息本身也不包含发布者身份；须结合启动配置、ROS 图发布者信息和该边实际消息检查，不可凭一张 TF 树图推断。

用户审计在 `hive2_leg_parameters.yaml` 中记录 `motion.leg.length=[0.38,0.48]` m，`foot_height_min=0.5885` m、`foot_height_max=0.8585` m，home pitch1=+45°、pitch2=-75°。所报 home 几何下轮底约悬空 0.1206 m，与实际接地状态矛盾；**真实最小离地间隙仍未确认**。现场测量雷达球心离地约 0.8943 m 也不能替代最新运行态 TF。

## 3. Livox MID-360 与点云接口

| 项目 | 记录值 | 说明 |
| --- | --- | --- |
| 雷达型号 | **Livox MID-360** | `MID360_config.json`；用户审计，`lidar_type=8` |
| 设备 IP | `192.168.1.196` | 报告中的局域网配置，仅记录，不写入启动配置 |
| `base_link → lidar_link` | 平移 **(+0.295, 0, 0) m**；RPY **(0, 0, 0) rad** | 运动栈和 SLAM 的 URDF 报告一致；z=0 是相对机体，不是离地高度 |
| 默认预处理点云 | `/livox/lidar/geo_filtered` | `sensor_msgs/PointCloud2`；`frame_id=lidar_link` |
| 原始点云 | `/livox/lidar` | `frame_id=livox_frame`；仅显式选择，不允许未校验 TF 自动切换 |
| 雷达实际离地高度 | **未确认** | 0.90 m 已选为配置基准；仍需根据实时姿态及 TF 验证 |

默认传感器输入与本仓库 `src/n01_bringup/config/n01_sensor.yaml` 一致：`input_topic=/livox/lidar/geo_filtered`、`input_frame=lidar_link`、`raw_topic=/livox/lidar`、`publish_raw_fallback=false`。这只能证明本仓库的接口约定，不代表实时真机 TF 已通过检查。

## 4. 定位及 TF 链

预期结构：

```text
map
└── base_footprint     ← SLAM 有效定位时提供动态 TF
    └── base_link      ← Hive2 运动栈 RSP；当前选定高度 0.90 m，待验证唯一发布
        └── lidar_link ← 运动栈 RSP；(0.295, 0, 0)
            └── imu_link   ← SLAM static_tf_publisher
                └── livox_frame ← SLAM static_tf_publisher
```

- 最新 SLAM `slam3d_node` 是 `map → base_footprint` 唯一预期发布者，Mapping Live 或 Localization Tracking 时可发布；**导航就绪必须进一步要求 `/localization_status` 显示 outer=Localization、inner=tracking 且 `registration_output_tf_published=true`**，并核实状态及时有效。单有 `map` TF 不够。
- 2026-10-26 最新 SLAM release **默认依旧开启 z=0.30 m 的 mount RSP**；需要显式 `launch_robot_state_publisher:=false`，之后再核实 Hive2 RSP 是该边唯一发布源。运动栈与 SLAM mount 节点还存在同名 `robot_state_publisher` 的风险。
- 记录中 `bag_20261027_225100` 曾出现完整链及 IMU/Livox 静态边；**不证明当前在线状态存在相同链**。
- 最新预处理链是 `/livox/lidar` (`livox_frame`) → 在 `base_link` 做几何过滤 → `/livox/lidar/geo_filtered` **输出仍是 `lidar_link`**。真实点云须按 **PointCloud2 原始采集时间戳**解析 `map ← lidar_link`，才能作为 SCAN `map` 规划输入。**不能仅把 frame_id 改为 `world` 或用最新时刻 TF 偷换扫描时刻**。本仓库 `n01_sensor.yaml` 还配置了 `target_frame: odom`，它与规划目标 `map` 的关系需要在接口适配时统一，不可暗改。

## 5. 速度、规划响应与药厂安全标准

| 参数 | 数值 / 状态 | 性质 |
| --- | --- | --- |
| N-01 导航速度上限 | **≤ 1.0 m/s** | 需求规格，非底盘物理极限 |
| 单次规划响应时间 | **≤ 500 ms** | 需求指标，尚未据此完成测试 |
| 连续运行能力 | **≥ 8 h** | 需求指标 |
| 静态导航成功率 | **≥ 95%** | 需求指标 |
| `chassis.max_linear_speed` | **2.0 m/s** | Hive2 底盘控制器配置上限，不是导航验收速度 |
| `teleop.linear_speed_max` | **2.0 m/s** | 遥操作配置上限 |
| `teleop.linear_speed` | **0.20 m/s** | 遥操作默认值 |
| `teleop.linear_speed_min` | **0.10 m/s** | 遥操作配置最小值 |
| `chassis.omega_per_linear` | **1.25** | 比例配置，不可解释为已批准的角速度 |
| 药厂最大角速度 | **未确认** | 需甲方确认 |
| 药厂正式障碍安全距离 | **未确认** | 需甲方确认 |
| `safety_radius=0.35 m` | **暂定值** | 乙方设计文档草案，不可直接作为正式验收标准 |
| 真实整机碰撞包络、动态障碍距离、加速度/制动距离、通道宽度 | **未确认** | 需另行验证/批准 |

上述需求来自用户审计所述 `N-01导航算法需求规格说明书_优化版_20261017.docx`，并未在本次提交中直接附带其原件。

## 6. 现有 G1 合成测试参数——不得作为真机配置

本仓库 `src/scan_planner/plan_manage/config/planner.yaml` 与 G1 测试输入使用：

| 参数 | G1 合成值 | 状态 |
| --- | ---: | --- |
| 点云/规划参考系 | `world`；`cloud_is_world=true` | **合成测试约定** |
| body pose z | 0.4 m | **合成测试值** |
| sensor pose z | 0.8 m | **合成测试值** |
| goal z | 0.4 m | **合成测试值** |
| `grid_map.body_height` | 0.4 m | **合成地图配置**，不同于 TF 机身高度 |
| `grid_map.double_cylinder_radius` | 0.25 m | **未按 Hive2 全身包络校准** |
| `grid_map.double_cylinder_offset` | 0.18 m | **未按 Hive2 全身包络校准** |
| 双圆柱水平近似包络 | 约 0.86 × 0.50 m | 由当前合成参数推得，非实际整机外廓 |
| `grid_map.resolution` | 0.05 m | 当前离线配置 |
| `manager.max_vel` | 0.75 m/s | 当前规划上限，满足 1.0 m/s 需求上限但不代表完成验证 |
| `manager.max_acc` | 0.50 m/s² | 当前规划配置 |
| `optimization.dist0` | 0.20 m | 优化器参数，**不是**甲方批准的药厂安全距离 |

**切忌直接把 `body_height=0.4` 改成 `0.90` 或 `0.30`。** `body_height` 与 TF 中 `base_link` 的离地高度并非同义参数。

## 7. 待决问题与修改门槛

| 编号 | 优先级 | 必须先解决 |
| --- | --- | --- |
| TF-HEIGHT | **P0** | 最新 SLAM 默认 **仍开启 0.30 m mount RSP**；启动时显式关闭并核实 motion RSP 唯一发布 0.90 m（含同名节点风险） |
| TF-LOCALIZATION | **P0** | 采用 `/localization_status` 的 `registration_output_tf_published=true` 且 outer=Localization、inner=tracking；状态需新鲜，检查扫描时刻 TF |
| SCAN-COORDS | **P0** | `geo_filtered` 输出是 `lidar_link`；按采集时间转换到 `map`，并对齐现有 `target_frame: odom` 与合成 `world` 的不同约定 |
| BODY-ENVELOPE | **P1** | 核实全身腿轮/外壳/步态碰撞包络 |
| GROUND-CLEARANCE | **P1** | 实测最小离地间隙并核实机身高度是否变化 |
| FACTORY-SAFETY | **P1** | 甲方确认安全距离、角速度、速度相关制动距离等约束 |

**下一次联调的详细只读 preflight 规格：** [`hive2_slam_p0_readiness_preflight.md`](hive2_slam_p0_readiness_preflight.md)。以下状态均为“预检条件已定义，尚未运行/验证”。

**维护规则：** 这是一份独立于可执行配置的**参数台账**。后续每一个参数的确认需补充来源文件、测量条件、责任人和日期；从“用户确认配置选择 / 用户审计报告 / 未确认”变更为“运行态或真机验证完成”须有对应证据。G1 离线场景与实际 Hive2 参数分别保存；在 P0 运行态校验完成前，不改动 `planner.yaml`、SLAM RSP、运动栈 TF 或底盘实际控制值。


## 8. 2026-10-30 补充：用户从实际 Bag 提取的传感器静态 TF（尚未独立复读原始 Bag）

**数据来源与证据等级**：用户提供的 `/tf_static` 提取结果，属于 **USER_REPORTED_RECORDED_BAG_TF**；仓库自身未在这次更新中重新打开原始 rosbag2 或直接读取外部 URDF。这里的“0.90m 已确认”仅指该录制会话采用的 Hive2 名义安装链，**不表示**在线唯一 TF 权威、传感器到真实地面距离或全路线平面已验证。

| 边 | 录制静态平移 (x,y,z) m | 备注 |
| --- | --- | --- |
| `base_footprint → base_link` | (0,0,+0.900) | 本次 Hive2 运动 TF 选定链；录制中也观察到竞争的 0.300m 值 |
| `base_link → lidar_link` | (+0.295,0,0) | 雷达前向偏移 |
| `base_link → lpms_imu_link` | (0,0,0) | 与 base_link 同中心 |
| `lidar_link → imu_link` | (0,0,0) | 据提取有约 10.8° 俯仰旋转，零平移 |
| `imu_link → livox_frame` | (0,0,0) | 零平移 |

据此，`lidar_link`、`imu_link`、`livox_frame`、`lpms_imu_link` 的中心相对于 **base_footprint 名义坐标轴** Z 均为 +0.900m；传感器在地图 Z 方向的实际高度还要经过 `map→base_footprint` 旋转，不能在存在显著 roll/pitch 时无条件做标量加法。10.8° 是零平移的 IMU 姿态差，不会单凭旋转改变传感器坐标原点。

```text
p_map_sensor = p_map_footprint + R_map_footprint * p_footprint_sensor
H_sensor_over_surface =
    p_map_sensor.z - ground_surface_z_at_sensor_xy
```

本次首帧有效扫描用户报告 `map→base_footprint` 的 z = 0.003491m。因此，**仅在 footprint 坐标轴与地图竖直方向对齐**并且已知对应 XY 点的实际地面高程时，才可写成 `H_sensor = 0.903491 - ground_z_map`（单位 m）。`ground_z_map` 未确定；此前 PCD 约 -0.5m 的低层点簇**不得**自动认作地面，也不得给出确定的 1.4m 实际雷达高度。

**场景层面**：路线不一定为水平地面，后续应确定局部地面高度函数 `ground_z_map(x,y)`、法线、支撑连续性与整机净空，不能用一个起点的 Z 常数代替药厂通道的三维通行检测。录制内同时存在 0.30/0.90m 边定义冲突，必须保留审计证据，不可因选定 0.90m 而将双 RSP 冲突标为已修复。离线历史重建可以使用明确选定的 0.90m 覆盖，但**不得宣称真实 SLAM 在线发布者唯一**。



## 9. 2026-10-30：25 帧真实录制关系和固定腿姿态轮式开发工况

用户已补充：录制 Bag 的 `/tf` 与 `/tf_static`、甲方 URDF、SLAM/点云预处理可交叉说明一个 **25-frame TF 森林**：直接 `map→base_footprint`；`base_footprint→base_link` 选用 Hive2 0.90m；`base_link→lidar_link` 为 (0.295,0,0)m；`lidar_link→imu_link→livox_frame` 与独立 `base_link→lpms_imu_link`；四条腿各 4 个关节/末端帧，另有独立 `fixed_frame→imu_filtered`。**录制会话没有 odom 边**；本次不做 `odom=map` 的隐性转换。约 11 条 map 动态边、296 条 IMU 过滤 TF、抽样 100/100 帧的 `geo_filtered` 均为 `lidar_link` 的数字来自用户报告，尚未在本次提交中重新分析 Bag。

**开发工况已定：四条腿固定工作姿态，仅靠轮式移动；不设计抬腿、迈步、步态切换。** 这只是研发约束：不能误写为当前已实现/已验证的腿关节保持控制，也不自动证明轮式底盘是差速、阿克曼或全向。**固定腿仍必须计入机器人碰撞包络**，现有 G3 二维全向点质点仅是离线数值研究模型。

- [完整 25-frame TF 与固定腿轮式导航接口契约](hive2_wheel_only_tf_frame_contract_20261030.md)
- [机器可读契约 JSON](../config/reference/hive2_wheel_only_tf_contract_20261030.json)

代码配套变更：将 `src/n01_bringup/config/n01_sensor.yaml` 的地图定位目标帧从录制中不存在的 `odom` 改为 `map`；`sensor_only.launch.py` 仍默认 `lidar_link` 无地图定位显示，去除必须有 `/odom` 的错误提示。**这些不等于 SLAM/TF 实时验收通过，也未修改 ROS 控制节点或转动任何车轮。**

此前所有未确认的地面 `ground_z_map(x,y)`、整机外廓、轮式运动学、角速度/制动能力、固定腿关节角与锁定反馈仍保持未验证；0.30/0.90m RSP 双发布风险继续是 P0 问题。
