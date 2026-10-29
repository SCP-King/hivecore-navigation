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

## 2. 高度基准已选定：0.90 m；运行态仍需核验（P0）

**2026-10-29 用户决策：Hive2 运动栈最新、已适配的 `base_footprint → base_link` 高度采用 `z=+0.90 m`。** 此值作为后续适配工作的**当前配置基准**。先前 SLAM 栈 `footprint_xyz="0 0 0.3"` 是**历史配置记录**，不再与 0.90 m 并列作为待选方案。

| 来源 | 高度 | 当前地位 |
| --- | ---: | --- |
| Hive2 运动栈 `hivecore_sensor_frames.xacro` | **0.90 m** | **已选定**；用户确认最新系统已适配 |
| 历史 SLAM 栈 `hivecore_robot.urdf.xacro` 默认值 | 0.30 m | **旧值，仅作溯源**；不得作为当前 Hive2 运行配置 |

**注意证据层级：** 用户确认了配置选择和“已适配”，但本次仓库更新未读取最新运动栈/SLAM 原件，也未观测运行时 TF，因此**尚未独立验证**当前唯一发布者、`/tf`/`/tf_static` 实际高度和定位有效状态。若 0.30 m 旧静态边仍被发布，则仍有真实运行冲突；验证完成才能关闭该 P0 运行态风险。

用户审计在 `hive2_leg_parameters.yaml` 中还记录：`motion.leg.length=[0.38,0.48]` m、`foot_height_min=0.5885` m、`foot_height_max=0.8585` m、home 角度 pitch1=+45°、pitch2=-75°。所报 home 几何下轮底约悬空 **0.1206 m**，与实际接地状态矛盾，**真实运行中的最小离地间隙仍未确认**。轮足运动时 `base_link` 相对地面的高度是否变化也需要确认。现场记录雷达球心离地约 **0.8943 m** 是独立测量，不能代替实时 TF。

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

- `map → base_footprint` 仅在 GUI-SLAM **有效定位时**可信；旧离线情形曾无 `map` frame，`tf2_echo map lidar_link` 失败。
- 历史配置中该 TF 边曾出现 z=0.90/0.30 m 两种定义；用户已选定最新适配值 0.90 m。必须核实旧 0.30 m 发布源确已停用，不能仅凭“已适配”宣告运行态无冲突。
- 记录中 `bag_20261027_225100` 曾出现完整链及 IMU/Livox 静态边；**不证明当前在线状态存在相同链**。
- 真实 `lidar_link` 点云需按消息时间戳转换到统一规划坐标系。**不能仅把 frame_id 改为 `world`**。

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
| TF-HEIGHT | **P0** | **高度配置已选定 0.90 m**；核实唯一 TF 发布源、旧 0.30 m 是否停用，以及轮足姿态下是否需要动态高度 |
| TF-LOCALIZATION | **P0** | 验证有效定位状态下的 `map → base_footprint` 及时间同步 |
| SCAN-COORDS | **P0** | 将 `lidar_link` 点云经正确 TF 转换后供 SCAN 使用，不能混用合成 `world` |
| BODY-ENVELOPE | **P1** | 核实全身腿轮/外壳/步态碰撞包络 |
| GROUND-CLEARANCE | **P1** | 实测最小离地间隙并核实机身高度是否变化 |
| FACTORY-SAFETY | **P1** | 甲方确认安全距离、角速度、速度相关制动距离等约束 |

**维护规则：** 这是一份独立于可执行配置的**参数台账**。后续每一个参数的确认需补充来源文件、测量条件、责任人和日期；从“用户确认配置选择 / 用户审计报告 / 未确认”变更为“运行态或真机验证完成”须有对应证据。G1 离线场景与实际 Hive2 参数分别保存；在 P0 运行态校验完成前，不改动 `planner.yaml`、SLAM RSP、运动栈 TF 或底盘实际控制值。
