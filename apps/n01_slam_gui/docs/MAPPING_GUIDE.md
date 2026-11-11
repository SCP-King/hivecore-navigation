# SLAM3D Lite 建图操作速查

本页是建图流程速查；完整安全要求见 `GUI_USER_MANUAL.md`。

## 启动前

- 确认定位工作流已停止，`localization_lite_node` 不在运行。
- 确认 Livox 驱动及点云预处理链路已提供 `/livox/lidar/geo_filtered`。
- 确认 IMU 等 desk 所需输入已准备好。
- 确认使用的是 `/opt/slam3d_release` 环境。

## 启动

```bash
source /opt/slam3d_release/setup_runtime.sh
ros2 launch hivecore_slam3d_lite_console mapping_lite_desk.launch.py
```

也可以使用 GUI“操作台”中的节点管理，但必须先勾选启用，且不能重复启动已经运行的辅助节点。

## 建图期间

- GUI“状态与地图”页的建图会话应为 `mapping`。
- `/cloud_map` 点数应随采集增长。
- 可在建图期间调用 `save_map` 保存当前地图快照（时间戳 PCD）。

## 正常结束

在 GUI 点击 `deactivate_mapping` 并确认。该服务结束建图、自动保存全分辨率 PCD，然后清空内存。记录 GUI 的 `save_dir` 和服务响应，并检查 PCD 文件。

**不要把结束后的 `save_map` 当作必要的第二步。** `save_map` 是建图过程中保存当前地图；deactivate 后已进入 Idle，内存地图被清空，再调用可能返回 `no FE local map yet`。

## 切换定位

确认建图节点已退出后，再启动定位 desk。将定位节点的 `map_path` 设置为刚才保存的 PCD，然后调用 `load_map` 并进入 Relocalization。
