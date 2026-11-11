# 2026-11-11 甲方离线地图规划性能实测

地图：`/home/demo/lyy/slam_data/maps/mapping_lite_20261109_140536_087.pcd`。
后端：甲方 Lite，Tracking；目标 map `(1.2171870498, -0.0529438771)`。
测试仅发送目标，不启动轨迹执行器。两次优化后测试均产生新 B 样条和 501 点候选路径。

## 改动

- `plan_env`、`path_searching`、`bspline_opt`、`traj_utils`、`scan_planner`
  默认 Release（显式 Debug 保留），实际构建参数 `-O3 -DNDEBUG`。
- 地图可视化导出默认 0.2 秒，可通过 `grid_map.visualization_period_s` 配置；
  占用更新 50 ms、FSM 10 ms、碰撞检查 50 ms 的调度参数不变。
- 新增地图更新与可视化耗时日志。
- 保留地图分辨率、膨胀半径、搜索/优化数学和碰撞判据。

## 实测

| 指标 | 优化前 | 优化后首次 | 优化后复测 |
|---|---:|---:|---:|
| 目标发出到全局路径 | 约 20 秒以上 | 111.9 ms | 61.8 ms |
| 目标发出到 B 样条 | 25 秒窗口内未收到，之后延迟生成 | 123.3 ms | 149.5 ms |
| 目标发出到候选路径 | 25 秒窗口内未收到 | 171.1 ms | 200.4 ms |
| 全局搜索+平滑计算 | 331.6 ms | 日志 9.3 ms | — |
| 局部优化计算 | 5.77 ms | 0.653 ms | — |
| 静态图重建 | 1852.223 ms | 214.118 ms | — |

前后测量来自同一运行现场、同一地图与目标，非受控统计基准。
优化前只有日志及 25 秒观察窗口，不能将其当作精确完整延迟或 p99。
优化后复测返回 `NECESSARY_CONDITIONS_PASS`、`candidate_review_ready=true`；
这只验证规划候选输出，不代表物理执行安全验收。

剩余热点：稳态地图占用更新采样约 76–88 ms，可视化约 45–49 ms。
SCAN 进程主线程累计 CPU 约从 97% 降至 60%（不是瞬时平均基准）。
单线程地图计算仍是下一阶段热点，不能未经共享状态同步设计直接换多线程。

## 验证与复现

五个核心包构建通过；`test_static_inflation` 与 `test_uniform_bspline` 通过。
ROS 使用 domain 55、Fast DDS，保留甲方定位和机器人驱动。

```bash
source /opt/ros/humble/setup.bash
source /opt/slam3d_release/setup_runtime.sh
source install/setup.bash
export ROS_DOMAIN_ID=55 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
python3 scripts/n01_planning_latency_probe.py --goal 1.2171870498 -0.0529438771
```

探针遇到未知 B 样条消费者时拒绝发目标，避免接入未确认执行器。
输出日志：`/tmp/n01_optimized_latency_20261111.txt`、
`/tmp/n01_optimized_latency_repeat_20261111.txt`。
运行日志：`/home/demo/lyy/n01_runtime/logs/n01_navigation_launch_20261111_120753.log`。
