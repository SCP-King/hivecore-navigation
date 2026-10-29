# G1 一键离线测试

只在与机器人底盘和真实 SDK 完全隔离的 Ubuntu 22.04 / ROS 2 Humble 开发机运行：

```bash
cd /home/demo/lyy/hivecore-navigation
git pull --ff-only origin main
bash scripts/run_g1_offline.sh
```

脚本自动执行 Python 单元测试、编译 SCAN 和融合接口、隔离 ROS 图预检、真实 SCAN 规划节点和模拟点云/位姿/目标发布器启动、轨迹监听和话题检查，然后仅结束自己启动的子进程。

不需要手动打开三个终端。测试时不要运行之前的 A/B/C 测试程序，也不要连接真机。

即便测试失败，脚本也会把记录保存到 **源码仓库外**：

`/home/demo/lyy/n01_runtime/runs/g1_scan/<timestamp>/`

并输出：

`UPLOAD THIS LOG ARCHIVE: /home/demo/lyy/n01_runtime/runs/g1_scan/g1_scan_<timestamp>.tar.gz`

请把该压缩包直接上传到当前 ChatGPT 对话。我可以根据里面的 `colcon_build.log`、`scan_planner.log`、`feed.log`、`monitor.log`、ROS 图快照、`g1_smoke_report.json` 等结果修复代码。

注意：只表示 G1 冒烟验证，不能当作完整避障验收或实机安全证明。压缩包包含本地终端/主机环境元数据，上传前可以自行检查。
