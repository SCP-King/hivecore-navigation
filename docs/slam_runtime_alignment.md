# SLAM 运行时对齐（2026-11-19）

## 1. 运行时

| 项 | 当前 |
|---|---|
| SLAM 运行时根 | `/home/demo/lyy/slam3d_20261118` |
| 旧运行时 | `/home/demo/lyy/slam3d_lite_20261106_test`（仅用于兼容/回退） |
| 选择入口 | `scripts/lib/slam_runtime.sh`、`apps/n01_slam_gui/run_slam_lite_gui.sh` |
| GUI 默认 | `_detect_rosk()` 回退到新运行时 |

`n01_source_slam_runtime` 会清理旧运行时前缀，并拒绝混合安装树。

## 2. 定位配置

新运行时定位包的优化参数已合并进：

`apps/n01_slam_gui/config/n01_localization_lite.yaml`

保留 N-01 集成接口：

- `pose_topic: /n01/vendor_localization/pose_raw`
- `initial_pose_topic: /n01/vendor_localization/initialpose`
- `tracking_frame/mola_vehicle_frame: lidar_link`
- `vehicle_to_scan_xyz: [0,0,0]`
- `enable_imu_fusion: false`（低速线去畸变策略）
- `enable_extrinsic_tf_override: true`
- `lidar_imu_params_file: config/n01_static_tf_params.yaml`

吸收的新优化参数：

- `reloc_voxel_leaf_size: 0.06`
- `reloc_fitness_score_threshold: 0.030`
- `milestone_scan_to_map_enable: true`
- `milestone_local_multires_enable: true`
- `recover_score_map_mode: residual`
- `recover_prior_topk: 3`
- `trailing_local_hunt_timeout_sec` 等恢复/自动重定位参数
- `auto_enter_tracking_enable: true`、`auto_enter_tracking_ok_scans: 3`
- `auto_enter_relocalization_enable: false`

## 3. GUI 阈值

`FITNESS_THRESHOLD` 已从 0.045 改为 **0.030**，并用于：

- 进入跟踪校验；
- 定位匹配状态显示；
- 高级状态阈值提示。

## 4. 不变项

- 地图/定位/导航话题接口；
- 适配器 `n01_localization_adapter` 的字段映射；
- 桥接 `/cloud`、`/body_pose`；
- 导航规划/执行链。

## 5. 验证

- `source setup_runtime.sh` 后 `ros2 pkg prefix` 指向新树；
- `LocStatus` 消息可导入；
- GUI 测试通过；
- 仍需现场验证定位 tracking/relocalization 与新恢复参数。

## 6. 注意

新产品默认使用 `base_footprint/base_link` 和 `vehicle_to_scan_xyz [0.295,0,0]`；本工程保留 **lidar_link + 零偏置**，由 `n01_localization_adapter` 统一做一次坐标转换。不要直接混用两套帧参数。
