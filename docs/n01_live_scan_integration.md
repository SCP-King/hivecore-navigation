# N-01 实时定位接入原生 SCAN（开发阶段）

## 本次功能

新增独立 ROS 2 包 `n01_live_navigation`，不修改甲方 SLAM、Livox/IMU 驱动、甲方 URDF 或 Hive2 底盘控制包。

`n01_live_scan_bridge` 订阅：
- `/localization_status`（`hivecore_slam3d/msg/LocalizationStatus`）
- `/livox/lidar/geo_filtered`（`sensor_msgs/msg/PointCloud2`）
- TF：在每帧点云的原始 `header.stamp` 查询 `map -> lidar_link` 和 `map -> base_link`

它将同一采集时刻的数据变换并发布为原生 SCAN 输入：
- `/cloud`：XYZ 点云，frame=`map`
- `/sensor_pose`：map 下的 lidar_link 位姿
- `/body_pose`：map 下的 base_link 位姿；速度由连续采样位姿差分估计，无效差分明确置零并在诊断中标记

只有 map 已加载、角色为 localization、会话为 tracking、pose/TF/fitness/compose 标志有效且状态新鲜时，才接收点云进入待处理队列。甲方当前 `fusion_path_imu_sync_ok=false` 单独记录为诊断，不擅自覆盖甲方已发布的 tracking 状态，也不把它伪装成同步成功。

### Scan-time TF 有界等待

当前现场测量显示：点云时间戳年龄约 0.105 秒，而定位 pose/TF 时间戳年龄约 0.38～1.01 秒；`/robot_pose` 与 `map -> base_footprint` TF 的时间戳一致。定位 TF 样本频率约 2.5～2.7 Hz，低于点云输入约 4.16 Hz。

为避免点云回调只等待 0.25 秒后立即丢帧，桥接器现采用有界 FIFO 队列：
- 每 0.02 秒重试一次原始扫描时间的 TF 查询。
- 单帧最多等待 0.45 秒。
- 待处理队列最多 8 帧，满载时丢弃最旧帧。
- `max_cloud_age_s=0.6` 仍是严格年龄上限；等待期间超过年龄上限的点云会被丢弃。
- 只有在原始扫描时间的 `map -> lidar_link` 和 `map -> base_link` 都可查询时才发布。
- 不使用 latest TF，不改写扫描时间戳，不将超龄点云送入 SCAN。

因此，该队列只处理短时 TF 到达抖动；它不会掩盖上游约 0.4～1.0 秒的定位延迟。如果现场延迟持续超过点云年龄门限，诊断中仍会看到 TF timeout / stale drop，必须继续优化 SLAM 上游处理链路，不能通过放宽年龄门限来宣称实时避障已经恢复。

`/n01/live_navigation/readiness` 使用 schema `n01.live_scan_readiness.v2`，新增：
- `pending_clouds`、`oldest_pending_wait_s`
- `cloud_tf_wait_started`、`cloud_tf_wait_completed`
- `drop_tf_timeout`、`drop_stale_after_wait`、`drop_queue_overflow`
- `last_cloud_age_s`、`last_tf_wait_s`、`max_tf_wait_observed_s`
- `cloud_output_fresh`

就绪状态现在要求定位状态新鲜，并且近期确实有成功输出的点云；不再仅凭历史上曾经成功输出过一帧就保持 READY。

该适配器只发布 SCAN 的输入与诊断，不创建 `/cmd_vel`、`/cmd_wheel` 或任何底盘控制 publisher。

## 启动

先构建：
```bash
cd /home/demo/lyy/hivecore-navigation
source /opt/ros/humble/setup.bash
colcon build --packages-up-to n01_live_navigation scan_planner n01_static_map --event-handlers console_direct+
source install/setup.bash
```

在甲方 SLAM 已正常定位、`use_sim_time=false`、`/localization_status` 为 tracking 的情况下启动：
```bash
ros2 launch n01_live_navigation live_scan.launch.py \
  pcd_path:=/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd \
  pcd_map_frame_asserted:=true \
  ground_z_map_m:=0.139 body_height_m:=0.90
```

`pcd_map_frame_asserted:=true` 是显式操作员断言：PCD 与当前甲方定位 map 是同一个坐标系。它不是自动配准证明。若定位当前加载的地图不是该 PCD 对应的 map，不能启用静态层。

## 目标点与输出

SCAN 使用原生 `fsm.navi_mode=1`。RViz 仍向 `/move_base_simple/goal` 发布目标，`n01_live_goal_gate` 只在实时定位/点云适配器已就绪且 `/grid_map/static_overlay_ready=true` 后才转发到 SCAN；目标 frame 必须是 `map`。SCAN 结果发布到 `/planning/bspline`，可同时查看 `/grid_map/occupancy`、`/grid_map/occupancy_inflate`、`/grid_map/static_overlay_ready` 与 `/n01/live_navigation/readiness`。

查看输入/结果：
```bash
ros2 topic echo /n01/live_navigation/readiness
ros2 topic echo /grid_map/static_overlay_ready
ros2 topic info /planning/bspline
ros2 topic echo /planning/bspline --once
```

## 已知限制

- PCD 的 map-frame 关系必须由操作者确认；本包不做 ICP 配准。
- `ground_z_map_m=-0.55` 是已有药厂地图证据支持的候选值，不是现场测量认证值；`body_height_m=0.90` 采用当前 TF/机体安装高度基线。
- SCAN 的双圆柱半径 0.33m、偏移 0.18m 和速度/加速度参数是保守研发参数，不是经过完整四轮转向运动学辨识的 Hive2 执行模型。
- 静态 PCD 以 3D 体素占用层进入 SCAN；地面、顶部、设备混合点仍可能造成过度占用或误判，必须从 RViz 检查地图与规划路径。
- 当前现场定位输出存在约 0.4～1.0 秒的消息时间戳年龄，且定位 pose/TF 约 2.5 Hz。桥接器的 0.45 秒等待与 0.6 秒最大点云年龄是有界缓冲，不是上游 SLAM 延迟修复。需进一步分析 SLAM 的 `/mapping_processing_timings` 和定位处理阶段。
- `fusion_path_imu_sync_ok=false` 仍需甲方解释/修复；本适配器会持续报告该状态。
- 当前阶段只验证定位、地图、实时点云和 SCAN 规划候选；没有接入 G2/G3 执行控制，也不输出底盘指令。不得把 B 样条当作可直接执行的安全轨迹。

## 安全边界

`actuator_output=DISABLED`，`safety_authorization=NEVER`。本 launch 不启动底盘控制器、不发布任何运动指令。实机移动前还必须完成 Hive2 四轮独立驱动/转向运动学、轮胎/机身碰撞外廓、定位失效停车、控制权互斥和急停联锁适配。
