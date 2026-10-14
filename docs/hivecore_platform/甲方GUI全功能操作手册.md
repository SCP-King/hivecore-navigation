# 甲方 GUI 全功能操作手册（只读学习版）

> 来源：安装包 `/home/demo/release_slam_3d/install/hivecore_console_slam3d/share/hivecore_console_slam3d/doc/manual_console_slam3d.md`、`.../config/slam3d_services_gui_params.yaml`，以及 `/home/demo/lyy/docs/Hivecore+SLAM+Services+GUI+—+用户手册.pdf`。新安装包手册优先于旧 PDF。

## 总原则

GUI 是 `slam3d_node` 的 ROS service client，不运行 SLAM。`gui_slam_console` 启动的是 preprocessing + `slam3d_node` Idle + GUI；`mapping_console` 和 `localization_console` 是独立外层角色，**不能与 gui_slam_console 叠加**。GUI 控件的状态改变必须有明确前置条件和恢复步骤，本轮不点击任何控制按钮。

| 选项卡 | 主要动作 | 主要服务/效果 | 前置与风险 | 成功观察 |
|---|---|---|---|---|
| preprocess | Cloud forward、geo filter | `lidar_cloud_preprocess_node/set_enable_forward`、`set_geo_filter`；改变过滤链 | 点云、参数和 TF 已就绪；Disable 会使 filtered 输出停止/静音 | `/livox/lidar/geo_filtered` 实际有帧 |
| mapping | Session Start/Freeze/Reloc/Resume/Stop、Save/Load | `enter_mapping`、`start_mapping`、`freeze_mapping`、`enter_relocalization`、`continue_mapping`、`stop_mapping`、`save_cloud_map_once`、`load_map` | Mapping 与 Localization 互斥；Start 会清空 album/PGO/local map；Stop 保留地图 | `/mapping_status` 的 outer/inner 状态、地图话题 |
| localization | Load、Session Start/Stop、Tracking、Relocalization、Reset | `enter_localization`、`load_map`、`start_localization`、`enter_tracking`、`enter_relocalization`、`stop_localization` | 必须有固定 PCD；先停止其他外层会话；Load/Start 可能改变会话和地图注册状态 | `/localization_status`、`/robot_pose`、map TF |
| octomap | Save/Reset、Z-filter | `/octomap_server/save_octomap`、`/octomap_server/reset` 等 | 依赖 octomap/helper；Reset 清空八叉树，需确认后操作 | octomap 话题/服务返回 |
| bag | Record/Replay、Pause/Continue/Stop | rosbag recorder/player 与 toggle_paused | use_sim_time/clock 必须一致；Fresh session 会终止并重启 ROS 节点，危险且本轮禁止 | bag 日志、`/clock`、输入话题 |
| viz | Ego/World RViz、Close all | 外部 RViz launch；Close all 会终止 RViz | 只读可视化，但需防止重复 RViz | RViz 进程/显示 |
| debug | Observe、Fitness、plots | debug 观察/绘图 launch | 只读诊断；不把 plot 存在当定位成功 | status 字段和 fitness |

## 定位按钮的精确语义

安装版手册给出的 Idle-hub 顺序是：停止其他 mapping/localization（如有）→ `enter_idle` → `enter_localization` → 若状态显示未加载地图则 `load_map` → `start_localization` → `enter_relocalization`。定位内层 FSM 为 `stopped → relocalization → tracking`；**Session Start 不等于 Tracking**，`enter_tracking` 是单独状态转换。`/initialpose` 是可选重定位种子，未设置时定位 relocalization 默认从 world origin 开始；不能将“收到 initialpose”当作定位成功。

## 危险操作恢复

- Mapping Start：从头清空 album/PGO/local_map；保存地图前不要点。
- Localization Load：替换/加载固定 PCD，使当前定位进入重新注册路径；确认路径和会话后再做。
- Localization Stop：停止定位但保留 PCD。
- Mapping/Localization 外层切换：互斥；不要直接叠加 launch。
- Fresh session：会终止 ROS 节点并重启，现场必须由甲方授权；本轮不执行。
