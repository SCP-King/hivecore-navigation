# 甲方平台完整架构与 SDK 接口

> 学习版本：Hive2 Motion Runtime 1.0.0、Hive2 Dev SDK 1.0.0、`hivecore_slam3d` 0.1.0、`hivecore_console_slam3d` 0.1.0。整理日期：2026-10-13。
>
> 证据状态：安装包文档/配置为源码证据；现场 ROS 结果单独标注“现场已验证”。本文件只来自只读阅读和只读 ROS 检查，不启动/停止服务，不操作底盘。

## 1. 分层架构

```text
乙方/第三方应用
  └─ Hive2 Dev SDK Client 或 ROS 2 话题/服务
       └─ Hive2 Motion Runtime（driver/controller/sensors）
            └─ EtherCAT、USB2CAN、Livox、BMS、gamepad

Livox → /livox/lidar → cloud_preprocessor → /livox/lidar/geo_filtered
                                           └─ static TF /tf_static
geo_filtered + /imu/data + extrinsic TF → hivecore_slam3d/slam3d_node
slam3d_node → /robot_pose、map→base_footprint、/localization_status
```

安装包 `hivecore_slam3d` 明确不启动 Livox、IMU filter、RSP、static TF 或 cloud preprocessor；这些是外部前置节点。

## 2. SDK 公共接口

来源：`/opt/hivecore/hive2-sdk/1.0.0/config/api_contract.yaml`、`docs/architecture.md`、`docs/modules/*.md`。

| 模块 | 接口 | 类型 | 说明 |
|---|---|---|---|
| chassis | `/cmd_wheel` | `geometry_msgs/msg/Twist` | SDK/teleop 应优先发布；经 preprocessor → `/cmd_vel` |
| chassis | `/cmd_vel` | `geometry_msgs/msg/Twist` | `/cmd_wheel` 下游控制接口；乙方本轮禁止连接 |
| chassis | `/odom` | `nav_msgs/msg/Odometry` | 底盘里程计 |
| chassis | `/chassis_state` | `interfaces/msg/ChassisState` | 底盘状态 |
| chassis | `/set_chassis_enable` | `interfaces/srv/SetChassisEnable` | 使能服务，危险操作，本轮不调用 |
| chassis | `/chassis_dec_stop` | `std_msgs/msg/Empty` | 减速停止 |
| chassis | `/estop` | `std_msgs/msg/Bool` | 急停状态 |
| lidar | `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | xfer_format=0 默认点云 |
| legs | `/leg_state` | `interfaces/msg/LegState` | 腿状态 |
| battery | `/battery_state` | `sensor_msgs/msg/BatteryState` | 多 BMS 时用 location/serial 区分 |
| teleop | `/joy` | `sensor_msgs/msg/Joy` | 遥控输入 |

SDK 默认命令发布频率为 20 Hz；示例 `set_velocity`/运动 action 只阅读，不运行。

## 3. 版本和边界

- Runtime：`/opt/hivecore/hive2-motion/1.0.0`，README 标示 Ubuntu 22.04 / ROS 2 Humble。
- SDK：`/opt/hivecore/hive2-sdk/1.0.0`；现场另有 `/home/demo/SDK/*.tar.gz`，未解包、未覆盖安装。
- SLAM：`/home/demo/release_slam_3d/install`，包版本均为 0.1.0。
- 乙方只读检查不等于控制验收；所有 `/cmd_wheel`、`/cmd_vel`、使能、腿 action/service 均保持未调用。

## 4. 已知现场证据

2026-10-13 只读采样：`/odom` 238 帧/3.5 s（约 68 Hz）；原始点云 48 帧/3.5 s；`geo_filtered` 40 帧/3.5 s，frame=`lidar_link`；`base_link→lidar_link` 和 `base_link→livox_frame` 可解析。运行时 `slam3d_node` 当前为 `outer_role=idle`、`localization.session_mode=stopped`，故定位产品未启动。
