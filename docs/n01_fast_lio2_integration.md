# N-01 FAST-LIO2 集成：保留甲方 SLAM 的双后端架构

## 1. 集成决策

本项目新增 FAST-LIO2 作为**可选的独立建图/激光惯性里程计后端**，不替换、不覆盖、不修改甲方 SLAM。甲方现有启动方式、消息类型、话题和 TF 契约继续保留。

| 责任 | 甲方 SLAM | N-01 FAST-LIO2 |
|---|---|---|
| 原有节点/启动 | 原样保留，由甲方原脚本启动 | 独立 ROS 2 launch |
| 输入 | 原有预处理及 Livox 数据 | 只读原始 Livox 点云与 Livox IMU |
| 里程计/地图 | 原有 `/robot_pose`、`/localization_status`、`map` TF | 私有 `/n01/fast_lio/*` 输出 |
| TF 权威 | 原契约不变；Hive2 motion RSP 负责机体静态安装 | **不发布 TF**，不会竞争 `map→base_footprint` 或机体安装 TF |
| 地图坐标 | 甲方 `map` | 私有 `n01_fast_lio_map`，不得默认等同甲方旧 `map` |
| 用途 | 现有定位/导航基线与回退 | 离线对比、独立建图、LIO 前端研发 |

可以在开发阶段让两套算法订阅同一份传感器数据进行 shadow comparison；FAST-LIO2 的所有输出独立命名，不会抢占甲方接口。两套 SLAM 不得同时发布相同全局 TF。

## 2. 上游版本与依赖

- 上游：`https://github.com/hku-mars/FAST_LIO`
- 分支：`ROS2`
- 固定 commit：`cc3c9e8b1469b33b8072764d6db2348ed6dc597f`
- ikd-Tree submodule commit：`e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`
- 运行目标：Ubuntu 22.04 / ROS 2 Humble / Livox ROS Driver2
- 上游 FAST-LIO 包依赖 `livox_ros_driver2`、PCL、Eigen、ROS 2。仓库通过 `n01_fast_lio2.repos` 与 `scripts/import_n01_fast_lio2.sh` 固定下载，不把 FAST-LIO 算法源码复制进本项目。

**许可注意**：上游仓库的 LICENSE 文件为 GPL-2.0，但其 `package.xml` 的 license 字段写 BSD，二者不一致。本项目以 LICENSE 文件为准记录为 GPL-2.0，并保留上游独立下载、原始许可与版权声明。商业产品分发前必须单独解决许可兼容/授权问题；本项目新增的 launch、参数模板和导入脚本本身按 Apache-2.0 管理。

## 3. Hive2 传感器输入

当前配置使用：

| 输入 | 配置 | 依据/限制 |
|---|---|---|
| 原始 Livox 点云 | `/livox/lidar` | FAST-LIO Livox 模式订阅 `livox_ros_driver2/msg/CustomMsg`，需要每点时间偏移 |
| Livox 内置 IMU | `/livox/imu` | 预期类型 `sensor_msgs/msg/Imu`；不自动替换成 LPMS |
| 甲方过滤点云 | `/livox/lidar/geo_filtered` | 不作为 FAST-LIO 输入；其 PointCloud2 输出不应被当作带原始每点时间戳的 CustomMsg |
| LiDAR-IMU 外参 | 当前模板为零平移/单位旋转候选 | 依据用户提供的 `imu_link→livox_frame` 零平移/单位旋转 TF；必须用 Bag 的原始话题、frame 语义和点云配准确认 |
| 时间同步 | `time_sync_en=false`、偏移 0 | 未获得独立时钟偏移标定前，不开启 FAST-LIO 软件时间同步 |

特别注意：`lidar_link→imu_link` 的约 10.8° 姿态差属于预处理/水平投影帧关系；FAST-LIO 使用原始 `livox_frame` 与 Livox 内置 IMU `imu_link`，不能把 10.8° 直接误填为 raw LiDAR-IMU 外参。Hive2 的 0.90m `base_footprint→base_link` 也不是 FAST-LIO 的 LiDAR-IMU 外参。

运行前先检查 Bag 是否实际包含 FAST-LIO 所需的两个原始输入：

```bash
python3 scripts/n01_fast_lio_bag_preflight.py \
  --bag /home/demo/lyy/slam_data/bags/bag_20261027_225100
```

该命令只读 rosbag2 `metadata.yaml`，不回放、不启动 ROS 节点。它要求 `/livox/lidar` 类型为 `livox_ros_driver2/msg/CustomMsg`，`/livox/imu` 类型为 `sensor_msgs/msg/Imu`。若不满足，不会自动改用过滤点云或 LPMS IMU。

## 4. 输出隔离与 TF 保护

上游 FAST-LIO 默认使用绝对话题名，并发布 `camera_init→body` TF。本项目导入时运行一个严格版本绑定的 patcher：

- FAST-LIO odometry、path、registered cloud、map 等输出 remap 到 `/n01/fast_lio/*`；
- 私有地图帧设为 `n01_fast_lio_map`；
- 里程计 child frame 设为 `imu_link`；
- FAST-LIO TF broadcaster 默认禁用，且 launch 再次显式传 `common.publish_tf=false`；上游 broadcaster 对象也仅在该参数为 true 时才创建，因此默认不会额外注册 `/tf` publisher endpoint；
- 输入仍只订阅已有 `/livox/lidar` 和 `/livox/imu`；
- 不触碰甲方 `/localization_status`、`/robot_pose`、`/tf`、`/livox/lidar/geo_filtered`，也不发布 `/cmd_vel`、`/chassis_controller/rt_command`。

输出话题：

| 输出 | 话题 |
|---|---|
| LIO 里程计 | `/n01/fast_lio/odometry` |
| 轨迹 | `/n01/fast_lio/path` |
| 地图系注册点云 | `/n01/fast_lio/cloud_registered` |
| IMU/body 系点云 | `/n01/fast_lio/cloud_registered_body` |
| 累积地图 | `/n01/fast_lio/map` |
| 地图保存服务 | `/n01/fast_lio/map_save` |

这使 FAST-LIO2 可以与甲方 SLAM 同时运行做只读对比，但 FAST-LIO 私有 map 与甲方旧 PCD 的坐标**不会自动对齐**。

## 5. 当前能力边界

本阶段接入的是 FAST-LIO2 的实时 LIO、增量局部地图和 PCD 保存能力。**上游 FAST-LIO2 本身不是完整的“加载已有 PCD 后长期定位/重定位”系统，也不自带完整回环闭合地图管理。** 因此当前不把 FAST-LIO 输出标成甲方 `Localization/Tracking`，也不直接将其接入 G4/SCAN 的甲方旧 `map` 坐标。

后续独立开发顺序：
1. 使用原始 Bag 回放 FAST-LIO2，检查时间戳、点数、IMU 频率、初始化、轨迹连续性和地图质量。
2. 将 FAST-LIO 输出与甲方同一 Bag 的 `/robot_pose`、PCD 进行对齐比较，先求出坐标关系，不改甲方 TF。
3. 开发保存地图加载、初始位姿/全局重定位、局部 scan-to-map localization 和失配恢复；必要时加入回环/全局优化。
4. 做统一 N-01 SLAM adapter：保留甲方 wire interface，同时把不同后端映射到内部 odometry/status/cloud 契约。
5. 只有 FAST-LIO 的 map alignment 与 localization status 明确就绪后，才允许将其作为 G4/SCAN 的定位来源。

## 6. 构建与启动

先 source ROS 2 Humble，并确保现有 Livox ROS Driver2 overlay 已 source。若 Livox 包不在当前环境，可通过 `LIVOX_SETUP` 指定已有安装，不会重复安装驱动：

```bash
cd /home/demo/lyy/hivecore-navigation
source /opt/ros/humble/setup.bash
# 如 Livox 包在独立 overlay，示例：
# source /path/to/existing_livox_workspace/install/setup.bash
bash scripts/build_n01_fast_lio2.sh
```

如果需通过脚本指定 Livox overlay：

```bash
LIVOX_SETUP=/path/to/existing_livox_workspace/install/setup.bash \
  bash scripts/build_n01_fast_lio2.sh
```

Bag 回放时在隔离域启动 FAST-LIO2：

终端 A：
```bash
cd /home/demo/lyy/hivecore-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
ros2 launch n01_fast_lio fast_lio_mapping.launch.py use_sim_time:=true
```

终端 B：
```bash
source /opt/ros/humble/setup.bash
source /home/demo/lyy/hivecore-navigation/install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
ros2 bag play /home/demo/lyy/slam_data/bags/bag_20261027_225100 --clock
```

回放后在 FAST-LIO 节点仍运行时保存地图：

```bash
ros2 service call /n01/fast_lio/map_save std_srvs/srv/Trigger {}
```

保存路径由 launch 参数 `map_file_path` 指定，默认 `/home/demo/lyy/n01_runtime/slam/fast_lio_map.pcd`。RViz 默认不启动；需要时加 `rviz:=true`。

**这只是离线算法回放，不会连接 Hive2 运动控制链。不要在真实底盘 ROS 域中直接照搬 ROS_DOMAIN_ID=88 的隔离 Bag 命令；真实在线运行需要单独的传感器/计算资源与 TF preflight。**
