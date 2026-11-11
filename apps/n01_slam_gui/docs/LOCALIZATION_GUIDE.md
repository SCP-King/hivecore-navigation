# SLAM3D Lite 定位操作速查

完整流程及安全说明见 `GUI_USER_MANUAL.md`。

## 启动定位 desk

先确认建图节点未运行，再执行：

```bash
source /opt/slam3d_release/setup_runtime.sh
ros2 launch hivecore_slam3d_lite_console localization_lite_desk.launch.py
```

定位 desk 会启动 Lite 定位及其 desk 所需辅助组件（包括 IMU 互补滤波、cloud_preprocessor；定位 desk 默认还启动 RSP）。如果这些辅助组件已在运行，不要重复启动 desk。

## GUI 检查

1. “状态与地图”页确认定位节点在位。
2. `map_loaded` 为 true，`map_path` 指向正确 PCD。
3. 上游 `/livox/lidar/geo_filtered` 有持续数据。
4. 定位会话进入 Relocalization。

## 地图与初始位姿

- 更换地图时，先确保 `map_path` 参数正确，再调用 `load_map`。
- 在“操作台”填写 x、y（米）、yaw（度），frame 选择 `map` 或 `base_footprint`，点击“发布”。
- `/initialpose` 仅在 Relocalization 会话接受；Tracking/Idle 会忽略。

## 重定位与 Tracking

以 `reloc_ok=true` 且 `reloc_fitness` 满足当前实际配置阈值作为重定位通过条件。已核对的 `/opt/slam3d_release` 配置阈值为 `0.045`，但现场应以运行节点实际参数为准。

确认重定位通过后，点击 `enter_tracking`。随后检查：

```bash
ros2 topic echo /loc_status --once
ros2 topic hz /robot_pose
ros2 topic info /robot_pose --verbose
```

Tracking 与 `reloc_ok` 是不同字段：进入 Tracking 不会自动将 `reloc_ok` 变成 true。不要为了使导航 READY 而伪造状态或绕过质量门控。

## 停止

调用 `deactivate_localization` 会停止定位位姿、TF 和地图输出。确认下游系统已准备好接受这些输出停止后再操作。
