# 甲方 ROS 2 数据接口与 TF 关系

> 来源：安装版 `manual_slam3d.md`、`manual_cloud_preprocessor.md`、`manual_robot_description.md`、SDK `api_contract.yaml`、现场只读 `ros2 topic info -v`/真实订阅/`tf2_echo`。状态分为安装文档证据与现场实测，不将预期当实测。

## 接口表

| 数据 | 类型/QoS | 发布者/前置 | frame/条件 | 现场状态 |
|---|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry`；现场 publisher Reliable | `chassis_controller` | frame=odom，child=base_link | 已实测约 68–70 Hz |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2`；SDK默认 | Livox runtime | frame=livox_frame（现场） | 已实测约 10–14 Hz；**显式可选**（乙方默认输入为 geo_filtered） |
| `/livox/lidar/geo_filtered` | PointCloud2，SLAM手册 BestEffort | cloud preprocessor；订阅触发输出 | frame=lidar_link | **乙方默认输入**；2026-10-14 乙方 local_map 实读入成功（/local_pcd width≈2455，~5Hz） |
| `/imu/data` | `sensor_msgs/msg/Imu` | complementary filter | 需 imu/TF 时间对齐 | 进程/接口存在；本轮未做完整同步验收 |
| `/robot_pose` | `nav_msgs/msg/Odometry`，Reliable | `slam3d_node` | 只在 Mapping 或 Localization 内层非 Stopped 发布 | 当前状态为 Idle/Stopped，不能验收 pose |
| `/localization_status` | `hivecore_slam3d/msg/LocalizationStatus` | `slam3d_node` timer | 无需定位有效也可发状态，但字段反映状态 | 已采到，map_loaded=false、pose/TF=false |
| `/mapping_status` | `hivecore_slam3d/msg/MappingStatus` | `slam3d_node` | mapping 状态/outer role | 本轮未把它当定位证据 |
| `/localization_map` | PointCloud2 BestEffort | localization map publish 开关+订阅者 | fixed PCD 可视化 | 未操作加载/发布，未验收 |
| `/tf_static` | TF static | motion RSP + GUI preprocessing static TF | 外参和 robot description | 3 个 PUBLISHER 端点 = **2× robot_state_publisher（冲突）+ 1× static_tf_publisher_node（正常静态边，不算冲突）**；base_footprint→base_link 冲突 |
| map→base_footprint | dynamic TF | `slam3d_node` | 定位/建图内层非 Stopped 且 TF 发布有效 | 当前未发布有效 |
| `/cmd_wheel` | `geometry_msgs/msg/Twist` | SDK/teleop → preprocessor | 底盘命令入口 | 本轮禁止发布 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `/cmd_wheel` 下游 | 真机执行链 | 本轮禁止连接 |

## 定位状态有效判据

`LocalizationStatus.msg` 的强证据字段：

- `config_context_map_loaded=true`：固定 PCD 已加载；
- `control_signal_outer_role=localization`；
- `control_signal_session_mode=relocalization|tracking`；
- `pipeline_scan_frame_match=true`、`pipeline_scan_point_count>0`；
- `registration_output_fitness_evaluation_ok=true`；
- `registration_output_tracking_compose_ok=true`；
- `registration_output_pose_published=true`；
- `registration_output_tf_published=true`。

`/localization_status` 话题存在不代表定位有效；它在 host timer 上可持续发布中间/Idle 状态。

## TF 双 RSP

已确认来源：运动栈 `hive2.config.xacro` 的 `base_footprint_joint` z=0.90；SLAM/robot description `hivecore_robot.urdf.xacro` 默认 footprint z=0.30。现场 `/tf_static` 共 3 个 PUBLISHER 端点 = `robot_state_publisher`×2 + `static_tf_publisher_node`×1，`tf2_echo` 采样 z 在 0.9/0.3 摇摆。**冲突计数只算 2 个 robot_state_publisher**（`static_tf_publisher_node` 发布的 base_link→lidar_link 等静态边不属于冲突）；preflight 5c 与 start_hivecore_deps.sh 均已按此修正。该问题独立于“GUI 是否 Load/Start 定位”，不得把两者合并；也不得新增第三条 TF 或写死高度。
