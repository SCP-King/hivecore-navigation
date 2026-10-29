# G1 完整离线验收

## 运行条件与范围

在**不连接真实底盘、SLAM、控制 SDK** 的 Ubuntu 22.04 / ROS 2 Humble 开发机运行。先清理以前的 G1 监测器和隔离域 ROS 2 CLI daemon，确认 ROS_DOMAIN_ID=88 内没有其他节点。

```bash
cd /home/demo/lyy/hivecore-navigation
git pull --ff-only origin main
bash -n scripts/run_g1_acceptance.sh && bash scripts/run_g1_acceptance.sh
```

脚本独立启动真实 SCAN Planner（不是合成规划器），自动执行四种场景：

- clear：存在周边障碍，要求获得三维膨胀地图和有效、无碰撞采样的 B 样条。
- blocked：在导航路径中央预先放置三维障碍，要求先有受阻地图，再生成绕障且不穿越膨胀体素的轨迹。
- changed：首次规划后新增中心障碍，要求观察到更新的膨胀地图，发布新 ID 的轨迹，且新轨迹不得穿越更新后的膨胀体素。
- no_odom：故意不给机器人主体里程计；发送目标后不允许产生轨迹。

验收通过的所有场景必须没有实际 `/cmd_vel` 发布者。每场景之间、脚本退出前均强制执行本轮进程清理和隔离 ROS 图清理。清理失败的状态永远不标记通过。

轨迹碰撞采用 B 样条 de Boor 采样与 **真实** `/grid_map/occupancy_inflate` 体素对比（0.05m 体素、约 0.09m 中心距离容差）；它是离线采样检测，**不能代替精确连续碰撞检测、机器人全身模型、动态障碍物与真实底盘安全验收**。任何场景缺乏正面证据将失败。

日志位置：`/home/demo/lyy/n01_runtime/runs/g1_acceptance/`。脚本结束后显示 `UPLOAD THIS LOG ARCHIVE:`，把其后完整路径的压缩包上传到聊天，无须复制终端输出。核心结果位于包内 `g1_acceptance_summary.json`、`cases/*.json`、`summary.txt` 与 `logs/*`。

**重要：** `PASS_OFFLINE_ONLY` 不是实机 G1 最终合格；真实药厂 PCD/Bag 回放、连续碰撞验证、感知缺失、实机里程计和底盘仍为 `NOT_TESTED`。
