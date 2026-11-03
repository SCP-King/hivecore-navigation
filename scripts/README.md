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
