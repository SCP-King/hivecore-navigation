# 发布包说明

## 2026-11-09 — 模块化控制台（总览 / 建图 / 定位 / 导航）

- 页面改为“整体项目生命周期 → 共享地图 → 建图 → 定位 → 导航”模块化布局。
- 顶部总览：定位进程、建图进程、导航、录包、RViz 状态；“启动定位 + 导航”“停止全部（保存建图）”“刷新状态”。
- 每个模块独立生命周期和 RViz 按钮：建图 RViz、定位 RViz、导航 RViz（导航自带 RViz 不重复启动）。
- 定位模块增加地图 PCD 加载、初始位姿、匹配 fitness/0.045 阈值、位姿/定位状态/地图/输入点云新鲜度。
- 建图模块增加录包、保存目录、点云数和建图状态/点云/输入新鲜度。
- 导航模块增加导航就绪/阻塞项/目标门控/系统健康/位姿新鲜度。
- 服务按钮全部中文化，内部 ROS 服务名保持不变。
- 保留建图+bag 同步、保存/停止顺序和 GUI 专属进程组清理。

## 2026-11-09 — 单页栅格控制台与统一 TF

- 旧版六个标签页合并为单页：投影栅格地图 + 建图/定位/导航手动控制 + 折叠诊断。
- 地图改为 **2D 障碍投影栅格**（定位 PCD / 建图点云 / 导航占用），不再使用点云散点图；支持点击地图填目标。
- 新增导航目标输入与 `/move_base_simple/goal` 发布；显示 Goal Gate 接受/拒绝原因。
- 新增定位地图 PCD 运行时设置与 `load_map`。
- 统一 TF：定位/建图 desk 使用 `tracking_to_vehicle=0.90` 参数；静态 TF 从 YAML 直接发布，不等待 IMU 标定；N-01 `/body_pose` 为 `map→base_link`，地面为 `base_link - 0.778`。
- 新增诊断页 TF 检查按钮，调用 `n01_frame_contract_check.py` 只读校验。
- 修正 Goal Gate 心跳显示：心跳的 `accepted=false` 不再把最近已接受目标显示成“拒绝 · accepted”；改用 `last_goal_accepted`。
- 新增全局路径 `/planning/global_path`（由 SCAN 全局轨迹发布，transient local）；GUI 同时显示全局路径（蓝色）和局部路径（青色），局部路径只负责当前窗口绕障，不再用它判断目标是否到达。
- 导航目标成功改为“全局路径到目标 + 局部路径存在”；局部 horizon 回到 10 m。
- 导航目标增加终点判定：路径终点距目标 > `N01_PATH_GOAL_TOL_M`（默认 0.30 m）判失败；失败目标红色标记，并显示局部目标调整/规划器反馈。
- 导航目标改为两阶段失败快返回：全局 A* 12 s / 局部 SCAN+CBF 12 s；失败立即报告耗时、清除目标并允许重新选点。
- 全局 A* 改为完整静态 PCD bounds 的 2D A*（不再使用 10 m 滑动地图和固定 300×300 池）。
- 导航目标改为“等待新路径”事务：目标发送后旧路径立即过期，只有新目标之后生成的有效 Path 才算成功；20 s 内无新路径则失败并写日志。
- 导航模块订阅 `/n01/live_navigation/hive2_candidate_path`，状态显示规划点数/新鲜度，地图用青色画候选路径；路径过期后保留最近一次非空路径（灰色），直到下一条路径覆盖；新目标显示“等待新路径”。，解决“日志已 final_plan_success 但界面看不到路径”的误判。
- 导航模块新增导航 PCD、地面 z 手动输入/同步；不一致时显示地图/地面一致性；启动导航通过 `N01_NAV_PCD`、`N01_NAV_GROUND_Z` 传给 `n01_navigation.sh`。
- 导航启动默认 `N01_NAV_RVIZ=false`，避免无 X 显示时启动死掉的 rviz2；GUI 的 RViz 按钮现在用 `xdpyinfo` 实际探测 X/Wayland，并在启动后检查进程，连不上时明确提示而不是留下死进程。
- 建图与 bag 录制合并为同一生命周期：启动建图同时启动录包；结束建图先保存 PCD，再停录包、停 desk；bag 失败不启动 desk，保存失败保留 bag/desk 以便重试。
- bag 由 GUI 专属进程组/看门狗管理，desk 异常退出、GUI 退出或“停止全部”都会收尾，避免孤儿录包。
- 保留生命周期/残留/进程组安全约束，不新增 actuator 输出。


本版本基于用户提供的 SLAM3D Lite GUI 压缩包修复生命周期安全问题，并整理源码结构。

## 本次修复

- GUI 启动的 desk、RViz、Foxglove 使用独立 Session/Process Group 管理。
- 看门狗通过 GUI PID + `/proc` starttime 识别原 GUI，防止 PID 复用导致误判。
- 进程组存活检查读取 `/proc` 并忽略僵尸进程。
- 停止流程先 SIGINT，等待正常退出，必要时才 SIGKILL 专属进程组。
- 单节点停止使用 PID starttime 二次校验，降低 PID 复用误发信号风险。
- ROS worker 退出时增加有界 join。
- 删除按进程名称扫描并批量杀进程的旧停止脚本。
- `PPID=1` 残留改为只读展示；无法证明所有权的进程不再自动清理。

## 结构整理

- `tools/`：看门狗和只读诊断工具。
- `tests/`：生命周期测试。
- `docs/`：用户手册、定位指南、建图指南、故障排查。
- 删除历史 ROS 日志、录屏/Graphviz 导出、缓存、临时编辑器目录和针对已弃用测试包的检查脚本。

## 适用环境

- ROS 2 Humble
- 甲方 SLAM3D Lite 综合包，默认 `/opt/slam3d_release`
- Python 依赖：NiceGUI、rclpy、NumPy，以及 Lite 消息包

本包不包含甲方 SLAM runtime、地图 PCD、ROS 安装或第三方依赖。

## 验证

- Python 语法检查：通过。
- Shell 语法检查：通过。
- desk_guard 独立生命周期集成测试：3 项通过。
- GUI ROS 集成测试需要在机器人已 source ROS 2 Humble + SLAM3D Lite 的环境执行；当前构建环境没有 `rclpy`，未在此处执行。

## Unified lifecycle integration

- Added a unified lifecycle page for Lite localization, mapping and N-01 navigation.
- Navigation start now performs managed stop -> status verification -> start; it fails closed if cleanup/status fails.
- Navigation start is gated by tracking, map_loaded, reloc_ok and configurable pose freshness.
- GUI shutdown stops only the navigation stack explicitly started by this GUI, plus GUI-owned SLAM/RViz/Foxglove groups.
- Added single-instance `flock` lock and fail-closed handling for stale ROS worker threads / pre-initialized rclpy contexts.
- No global killall/pkill behavior; upstream sensors, chassis, controllers, external SLAM and adapter are protected.

- GUI title/header now identify the unified N-01 SLAM + Navigation console.
- Added standalone lifecycle verification tests for managed navigation command ordering and fail-closed behavior.


## 2026-11-08 — Clean runtime and singleton ROS worker

- Start GUI from a clean environment; stale release prefixes from `~/latest` or
  `~/FROZEN` are not inherited.
- Verify Lite localization package prefix belongs to selected release.
- Add two-stage ROS graph census and scoped N-01 navigation cleanup before GUI start.
- Protect Lite SLAM, localization adapter, Livox, IMU, chassis and controllers.
- Keep one process-wide ROS worker across module reloads; reuse valid default
  Context with a dedicated executor and track Context shutdown ownership.
- Add regression tests for worker singleton contract and startup preflight.

## 2026-11-08 — ROS Worker startup-state hardening

- Wrapped the complete ROS worker entrypoint in a top-level exception boundary.
- Startup, node construction, subscription setup, executor registration, and spin failures are now written to `STATE.ros_error` and the GUI log instead of leaving the UI in an endless “starting” state.
- ROS readiness is asserted only after the dedicated executor successfully takes ownership of the GUI node.
- The UI detects a worker thread that exited without a reported error and surfaces a concrete failure state.
- Added best-effort cleanup of the executor, node, and only the rclpy Context owned by this worker.

## 统一生命周期修复 v2.1（ROS 连接状态）

- 修复 GUI 与 ROS Worker 在多模块名加载场景下可能持有不同 `AppState` 实例的问题。现在 AppState 与 Worker 都按 PID 保存在进程级注册表中。
- 新增 Worker `connected` 事件；只有 ROS 节点成功注册到专属 Executor 后才置为已连接。
- 页面状态刷新以存活 Worker + connected 事件进行状态对账，避免节点已创建但页面仍无限显示“等待 ROS 连接”。
- ROS 连接不再依赖 `localization_lite_node`、`mapping_lite_node`、`/robot_pose` 或 `/loc_status` 是否存在；这些属于 SLAM 模块运行状态。
- 不扩大启动前清理范围：启动预检仍只委托 N-01 导航仓库的受管生命周期入口，不会自动停止 Lite SLAM、Livox、IMU 或底盘驱动。
