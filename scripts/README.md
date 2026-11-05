# G1 一键离线测试与进程清理

只在与机器人底盘、真实 SLAM、真实控制 SDK 完全隔离的 Ubuntu 22.04 / ROS 2 Humble 电脑执行：

```bash
cd /home/demo/lyy/hivecore-navigation
git pull --ff-only origin main
bash -n scripts/run_g1_offline.sh && bash scripts/run_g1_offline.sh
```

脚本执行算法单元测试和构建、启动 SCAN/离线模拟/监测器、生成报告，并在退出时执行**进程清理验收**：

1. 给本轮测试启动的进程及其子进程分配唯一环境标识 `N01_G1_RUN_TOKEN`，在 Linux `/proc` 中精确查找。不会执行系统级 `pkill` 或 `killall`，也不会因 ROS 节点名称相同而结束其他项目的进程。
2. 测试正常完成、失败或收到 Ctrl+C/SIGTERM/SIGHUP 后，按 INT、TERM、KILL 逐级停止仅属于这轮测试的进程。
3. 验证本轮进程清零，再通过临时 `rclpy` 进程检查 ROS_DOMAIN_ID=88 的 ROS 节点图。检测到任何进程残留或 ROS 节点残留，就把结果标为 `CLEANUP_FAILED`，不虚报测试通过。
4. 生成 `summary.txt`，包含 `owned_process_cleanup=CLEAN` 和 `post_cleanup_ros_graph=CLEAN`；生成清理前后进程快照、信号记录和 ROS 图记录。

`CLEAN` 只表示脚本在结束时**观察不到本轮可识别的存活进程及隔离域内节点**，不能承诺断电、SIGKILL 强杀脚本、主动清除环境标识的后台进程等异常情况下的绝对保证；未识别或无法停止的进程须人工核实，脚本不会越权结束其他 ROS 项目。

日志保存在源码库之外：

`/home/demo/lyy/n01_runtime/runs/g1_scan/<timestamp>/`

脚本结束时会打印：

`UPLOAD THIS LOG ARCHIVE: /home/demo/lyy/n01_runtime/runs/g1_scan/g1_scan_<timestamp>.tar.gz`

把该压缩包直接上传到聊天，无需复制终端输出。即使测试失败也会保存诊断证据。

**安全前提：**不能在测试期间连接真机或启动真实控制链。此 G1 冒烟测试不等于完整三维避障及实机安全验收。


完整 G1 离线验收请参考 [G1_ACCEPTANCE.md](G1_ACCEPTANCE.md)，命令是 `bash scripts/run_g1_acceptance.sh`；原 `run_g1_offline.sh` 只做冒烟测试。


## N-01 当前规划故障诊断（唯一入口）

当前从 Hivecore Tracking 进入 N-01 / SCAN 规划时，统一使用仓库内这一份完整只读诊断入口：

```bash
cd ~/lyy/hivecore-navigation
git fetch origin fix/n01-pose-freshness-watchdogs
git switch fix/n01-pose-freshness-watchdogs
git pull --ff-only origin fix/n01-pose-freshness-watchdogs
bash scripts/n01_collect_planning_readiness_audit.sh
```

不要把脚本复制到 `/tmp`。主脚本会直接调用同一仓库 `scripts/` 下的两个配套只读探针：

- `n01_live_stack_preflight.py`：发布端点身份、频率、TF 边和实时链路预检。
- `n01_live_planning_ros_probe.py`：点云几何、位姿、IMU 和时间戳对齐。

诊断结果自动保存在 `~/lyy/n01_runtime/runs/planning_readiness_audit/`，并生成包含日志、JSON 探针和 TF 图的 `n01_planning_readiness_bundle_<timestamp>.tar.gz`。上传压缩包即可，不需要复制终端滚屏。

此入口只读取 ROS 图、话题、参数和文件；不发送导航目标、不发布底盘控制命令、不调用 SLAM 业务服务、不修改参数、不停止或重启节点，也不构建或运行机器人。

## Hivecore SLAM 位姿实时性专项验证

当前只验证更新后的 SLAM 是否及时发布全局位姿，不再扫描其他话题、参数、源码或处理耗时。

```bash
cd ~/lyy/hivecore-navigation
git fetch origin fix/n01-pose-freshness-watchdogs
git switch fix/n01-pose-freshness-watchdogs
git pull --ff-only origin fix/n01-pose-freshness-watchdogs
bash scripts/run_n01_slam_pose_pipeline_audit.sh 60
```

脚本只订阅现有的 `/robot_pose`（`nav_msgs/msg/Odometry`），自动统计发布频率、接收间隔 P50/P95/最大值、位姿时间戳年龄 P50/P95/最大值，以及采样结束时最后一条位姿的接收年龄和时间戳年龄。ROS 时间年龄为主指标，同时给出系统墙钟年龄作交叉检查。

当前参考预算：接收年龄 0.7 s、位姿时间戳年龄 1.5 s。脚本只测量、不修改阈值，不创建 publisher/client，不发目标、不调用服务、不写参数、不重启节点、不输出底盘命令。完成后上传终端显示的 `UPLOAD THIS BUNDLE: ...tar.gz`。

## N-01 /robot_pose 双发布端点专项诊断

当前运行日志曾出现 `robot_pose_publisher_count=2`，并伴随 `NAVIGATION READY/BLOCKED` 反复切换。先定位两个发布端点的节点名、DDS endpoint GID、QoS 和节点接线，不放宽 0.7 s 位姿接收新鲜度门限，也不重启 SLAM。

```bash
cd ~/lyy/hivecore-navigation
git fetch origin fix/n01-pose-freshness-watchdogs
git switch fix/n01-pose-freshness-watchdogs
git pull --ff-only origin fix/n01-pose-freshness-watchdogs
bash scripts/run_n01_robot_pose_source_audit.sh 20
```

脚本调用现有只读 ROS 图预检，采样 20 秒，并自动收集：

- `/robot_pose` 每个 publisher endpoint 的节点名、GID、类型和 QoS；
- 合并后的接收频率、接收间隔、消息时间戳年龄及帧 ID；
- bridge 的 `pose_duplicate`、`pose_out_of_order`、`drop_pose_duplicate_conflict` 等计数；
- 每个发布节点的 `ros2 node info`、topic verbose 信息及相关进程快照。

完成后终端会打印 `UPLOAD THIS BUNDLE: ...tar.gz`。直接上传该压缩包即可，不用复制终端输出。此脚本只订阅/读取，不创建 publisher/client，不发目标、不调用服务、不写参数、不重启节点、不输出底盘命令。ROS 2 普通订阅回调不能把单条消息直接归属到 DDS GID，因此报告会把 endpoint 身份与聚合消息时间序列分开呈现，不会伪称逐消息来源归属。

## N-01 READY 窗口内的一次性规划验证

当前实机证据已确认 `/grid_map/static_overlay_ready=true`。主要入口风险是甲方 `/robot_pose` 约 2.6 Hz、最大接收间隔 0.754 s，超过 N-01 0.7 s 接收新鲜度预算；不允许通过放宽门限掩盖上游发布节奏问题。

构建/部署成功且现有 `live_scan.launch.py` 正在运行时，使用：

```bash
bash scripts/run_n01_planning_smoke_test.sh 0.5 15 120
```

参数依次为前向距离（米）、B-spline 等待秒数、READY 窗口等待秒数。默认目标由当前 map-frame `/body_pose` 沿机器人朝向前移 0.5 m，不是固定 map 原点坐标；执行前必须确认该位置属于已知空旷区域。

测试会最多等待 120 s，并在发布前检查直接接收的 `/robot_pose` 年龄不超过 0.7 s、时间戳年龄不超过 1.5 s；还会把 `system_health` 消息自身的接收年龄计入 bridge 快照中的 pose receive age、stamp age 和 live-anchor delta，避免使用过期 READY 快照。runtime bridge 必须报告位姿锚点 fresh、system_health 无 blockers、静态 overlay READY、安全状态保持 `DISABLED/NEVER`，且规划 topic 拓扑正确。检查未通过或 READY 窗口关闭时不发布目标。通过后最多发送一个规划目标，只验证 Goal Gate 和 SCAN B-spline，不发布速度、轮速、底盘或执行器命令，也不改变 operator hold 或执行授权。

此 smoke test 是唯一推荐的 READY 窗口规划入口；不要手工运行未经门控的临时 `/move_base_simple/goal` publisher。

## N-01 live_scan 安全重启说明

`scripts/deploy_n01_planning_fixes.sh --confirm-planning-only` 只重启 N-01 `live_scan.launch.py`，不停止甲方 SLAM 或 Livox。部署脚本按 Linux 进程状态判断旧 launch 是否退出：`Z`（zombie）和 `X` 状态不再被 `kill -0` 误判为仍在运行；若进程仍存活，会打印 PID/PPID/PGID/SID、进程树和最近的 live_scan 日志，然后安全退出，不执行强杀。

旧版本启动的 launch 若没有独立 session，只向已识别的 launch PID 发送 SIGINT，绝不向未知进程组发信号。新版本使用 `setsid` 将 N-01 launch 及其子节点放入专用 session；后续仅在 PGID 与 SID 都等于 launch PID 时，才允许向该隔离进程组发送 SIGINT。launch 退出后逐一确认 SCAN、bridge、static map、supervisor、goal gate、trajectory monitor 和 RViz 等 N-01 节点均已退出，再启动新实例。不会使用 `kill -9`、`pkill` 或 `killall`。
