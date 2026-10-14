# 甲方 SLAM 建图/定位标准 SOP（证据版）

> 适用版本：安装版 `hivecore_console_slam3d` / `hivecore_slam3d` 0.1.0。来源：随包 `manual_console_slam3d.md`、`manual_slam3d.md`、GUI PDF。仅描述流程，不执行控制操作。

## A. 环境与前置

1. 分别 source ROS 2 和甲方 runtime：`source /home/demo/release_slam_3d/setup_runtime.sh`；不要把 host underlay 与该 isolated install 混用。
2. 运动栈、Livox、IMU filter、RSP、static TF、cloud preprocessor 必须按现场 launch 组织运行。
3. 真实运行用 `use_sim_time:=false`；bag 回放必须使所有相关节点一致使用 sim time 和 `/clock`。
4. `gui_slam_console`、`mapping_console`、`localization_console` 不叠加；地图/定位 session 只能有一个外层角色。

## B. 建图

1. 运行 preprocessing + `slam3d_node` Idle + GUI（或按甲方批准的 mapping console 流程）。
2. GUI mapping：Session Start。底层顺序为 `enter_idle → enter_mapping → start_mapping`；Start 是清空当前 album/PGO/local map 后重新建图。
3. 运动和点云过程中，检查 `/mapping_status`，不要仅看 GUI 窗口。
4. Save to disk/`save_cloud_map_once` 生成 PCD；确认实际路径和文件存在。当前现场已有 `/home/demo/lyy/slam_data/maps/nav_map_2026_10_10-20_54_46.pcd`，这证明存在地图文件，但不证明当前定位 session 已加载它。
5. 建图结束后按手册 Stop/Freeze/保存流程收尾，不要用同一 node 同时启动 Localization。

## C. 定位（当前问题的关键流程）

1. 确认不是 Mapping outer role；若当前是 Idle，进入 Localization；若是 Mapping，先按甲方流程停止/切换。
2. 在 localization 页选择实际 PCD，执行 **Load from disk**。安装版等价服务为 `/slam3d_node/load_map`，服务类型 `hivecore_slam3d/srv/LoadMap`。
3. 执行 **Session Start**，等价于 `/slam3d_node/start_localization`；它只把 stopped 变成 relocalization，前提是固定 PCD 已 loaded。
4. 需要时执行 `enter_relocalization`；定位成功后由操作员/GUI执行 `enter_tracking`，因为安装版 FSM 明确把 Tracking 作为独立转换。
5. 若没有有效初始位姿，检查 `/initialpose` 要求。该输入是可选重定位 seed，不是成功信号。
6. 必须用状态字段确认：`config_context_map_loaded=true`；`control_signal_outer_role=localization`；`control_signal_session_mode` 为 `relocalization` 或 `tracking`；`pipeline_scan_frame_match=true` 且 point_count>0；`registration_output_fitness_evaluation_ok=true`；`registration_output_tracking_compose_ok=true`；`registration_output_pose_published=true`；`registration_output_tf_published=true`。Tracking 稳定后再把定位视为有效。

## D. 当前事实与缺口

现场只读值：`outer_role=idle`、`localization.session_mode=stopped`、`localization.map_path=/root/resources_storage/cloud_map_pgo.pcd`、`config_context_map_loaded=false`、`enable_processing=false`、pose/TF published=false。当前默认路径不是 `/home/demo/lyy/slam_data/maps/nav_map_2026_10_10-20_54_46.pcd`；因此“地图已经建立”与“当前定位节点加载该地图”是两个事实。

这直接解释了当前 `/robot_pose` 无数据：按照安装版契约，pose 只在 Mapping 或 Localization 内层非 Stopped 时发布；当前为 Idle + Stopped。尚不能把它判成地图损坏或定位算法故障，因为本轮按边界没有点击 Load/Start。
