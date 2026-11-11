# N-01 规划时延瓶颈诊断（2026-11-11 现场实测）

> 一句话结论：**A\* 不是瓶颈，局部优化也不是**。时延被三件事吃掉：
> ① 全局 min-snap 平滑按路径点数 **O(n³)** 增长（0.4 m→0.5 ms，6 m→950 ms）；
> ② `scan_planner_node` 是**单线程 executor**，被 80 ms 级建图回调堵住，goal 回调排队 4–357 ms；
> ③ 同一个目标被**重复全局规划** 2–4 次，远目标 B 样条要 2.4–2.5 s 才出来。

## 1. 现场快照

| 项 | 值 |
|---|---|
| 时间 | 2026-11-11 13:46–13:56（launch 13:45:03 启动） |
| 地图 | `/home/demo/lyy/slam_data/maps/mapping_lite_20261109_140536_087.pcd`（496,432 点，窗口内静态源 202,764 点） |
| 机器人位姿 | map 系 (1.005, 0.385)，yaw ≈ −0.07 rad（整个测试期间未移动） |
| 规划链路 | `live_scan.launch.py`：scan_planner_node + goal_gate + supervisor + trajectory_monitor |
| 安全状态 | `operator_hold=true`、`safety_authorization=NEVER`、`execution_authorized=false`；无轨迹跟踪器，未发布 `/cmd_vel` |
| 系统负载 | load 10.7 / 8 核，CPU busy 72%；测试期间 `n01_localization_far_test` 同时在跑，`/cloud` 从 5.8 Hz 掉到 2.7 Hz |

> 测试只改变规划目标，不驱动底盘；`/cmd_vel` 上只有厂商手柄/预处理器，与本链路无关。

## 2. 目标点与实测

起点 (1.005, 0.385)。目标点先用 PCD 2D 投影 + 0.35 m 膨胀 + 连通域验证为**可达空闲格**。
延迟用消息自带时间戳对齐（goal 时间戳由探针在发布前打），比 Python 探针墙钟可靠（后者受进程调度影响有 ±180 ms 抖动）。

| 目标 (x, y) | 距离 | A\* 点数 | A\* ms | min-snap ms | goal→gate | gate→A\*（排队） | A\*+平滑 | goal→global | goal→B 样条 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| (1.400, 0.420) | 0.4 m | 8 | 6.7 | 0.5 | 1.4 | 80 | 7.8 | 264\* | 378 |
| (2.925, −0.175) | 2.0 m | 39 | 4.0 | 34.4 | 1.6 | 92 | 38.5 | 129 | 206 |
| (4.975, 0.875) | 4.0 m | 80 | 3.6 | 512.7 | 1.6 | 357 | 516.3 | 872 | 2364 |
| (−4.775, −1.225) | 6.0 m | 119 | 4.5 | 948.5 | 547 | 4 | 953.5 | 1501 | 2483 |

\* 0.4 m 目标在平滑结束到 `global_path` 发布之间还有约 180 ms，日志里没有目标占用/调整记录，
  属于规划线程在回调内被整机负载抢占（load 10.7/8 核），不是算法耗时；其余三行该间隙为 0–100 ms。

**同一份日志的 80 次全局规划统计**（`n01_navigation_launch_20261111_134503.log`）：

- `astar_ms`：p50 3.5 / p90 4.7 / max 16.3 → **不是瓶颈**
- `smooth_ms`：p50 268 / p90 366 / max 1115
- 局部 B 样条优化 `total time`：p50 0.55 ms / p90 3.9 ms / max 18 ms → **不是瓶颈**
- occupancy：p50 82 ms / p90 125 ms / max 712 ms（/cloud 5.8 Hz）
- visualization：p50 51 ms / p90 72 ms / max 236 ms（5 Hz）
- 静态图 overlay 重建：196 ms，**仅启动时一次**（`static_source_cloud_` 为 immutable）

## 3. 瓶颈一：`minSnapTraj` 立方增长（占规划计算 85–99%）

`planner_manager.cpp:559` 的 `globalAStar2D()` 返回的是**每格一个点**的栅格路径（0.05 m 一格），所以
`n ≈ 20 × 距离(m)`：2 m→39 点，4 m→80 点，6 m→119 点。

`polynomial_traj.cpp:128` 的 `minSnapTraj` 对 `6n×6n` 稠密矩阵做
`A.transpose().inverse()`、`A.inverse()`（R 里两次）、`Rpp.inverse()`（2n−2）、以及 `Px/Py/Pz` 里**又三次** `A.inverse()`。
A 本来是 6×6 块对角，代码没有利用，于是复杂度是 n³：

| n（点数） | 样本数 | min-snap p50 | 折算 `smooth/n³` |
|---:|---:|---:|---:|
| 8 | 2 | 0.6 ms | ~1.2 µs（固定开销为主） |
| 39 | 34 | 36.5 ms | 0.62 µs |
| 80 | 37 | 291.5 ms | 0.57 µs |
| 119 | 7 | 978.3 ms | 0.58 µs |

拟合：`t_smooth ≈ 0.6 µs × n³ ≈ 4.8 ms × d(m)³`。这是"离目标越远越慢得离谱"的根因；
`kMaxGlobalWps = 160`（`planner_manager.cpp:585`）只在 >8 m 时截断，仍允许 ~2.3 s 的单次平滑。

量级自检：n=119 时矩阵 714×714，单次稠密求逆约 `(2/3)n³ ≈ 2.4×10⁸` FLOP，5 次 A⁻¹ 约 1.2 GFLOP，
实测 0.95 s 折合 ~1.3 GFLOP/s——既说明没有利用 6×6 块对角结构，也说明线程在跟整机抢 CPU。

## 4. 瓶颈二：单线程 executor 被建图/可视化堵住

`scan_planner_node.cpp:16` 用的是 `SingleThreadedExecutor`，而同一个节点里：

- `grid_map.cpp:206` 占用更新定时器 50 ms → `raycastProcess()`（`grid_map.cpp:597`）每次 **65–190 ms**，随 `/cloud` ~5.8 Hz 触发，单核占用 ~0.5；
- `grid_map.cpp:214` 可视化定时器 0.2 s → `visCallback()`（`grid_map.cpp:788`）每次 **46–72 ms**，单核占用 ~0.25；
- `scan_replan_fsm.cpp:70/72` 的 FSM 10 ms 和碰撞检查 50 ms 只能插空跑。

合起来该线程 ~0.7–1.0 核，goal 回调平均排队几十 ms，实测 **gate→A\* 4 / 80 / 92 / 357 ms**（上表），
6 m 目标那次连 goal_gate 这个独立 Python 节点都排队了 **547 ms**（CPU 10.7/8 核、72% busy 的直接后果）。

## 5. 瓶颈三：同一目标重复全局规划

- `waypointCallback()`（`scan_replan_fsm.cpp:142`）里已经 `planGlobalTraj()` 并 `publishGlobalPath()`；
- 紧接着状态机进 `REPLAN_TRAJ`，`planFromCurrentTraj()`（`scan_replan_fsm.cpp:742`）**又调一次 `planGlobalTraj()`**；
- 局部优化失败或 50 ms 碰撞检查发现轨迹碰撞时，`planFromCurrentTraj()` 每次重试都**重算全局 A\* + min-snap**，不是只做局部重规划。

实测 4 m 目标：`516 + 340 + 370 + 301 ms` 共 **4 次全局规划**，B 样条 2364 ms 才出来；
6 m 目标 2 次（954 + 932 ms），B 样条 2483 ms。日志里 7 个目标共出现 18 次
`REPLAN_TRAJ → REPLAN_TRAJ`，34 次 "Local target was adjusted"，10 次局部 A\* error。

## 6. 不是瓶颈

- A\*：3–7 ms，与距离几乎无关；
- 局部 B 样条优化：p90 3.9 ms；
- 静态图 overlay：196 ms，只重建一次；
- 点云/定位注入：`/cloud` 起始 5.8 Hz、`/body_pose` 正常，不是"等数据"型瓶颈（不像早期 `cube_side_length` 问题）；
  测试后段被整机负载拖到 2.7 Hz，属于 CPU 争抢（见 P4）。

## 7. 建议（按收益/风险排序）

| 优先级 | 改动 | 预期 |
|---|---|---|
| P0 | `planGlobalTraj` 里对 A\* 路径按 0.25–0.4 m 重采样（或做带碰撞校验的视线捷径）再送 `minSnapTraj` | 6 m 目标 119→20 点，min-snap 950 ms→~6 ms，规划核心→~15 ms；候选路径若仍走 0.25 s 重发则端到端 ~0.3 s（详见 hardware_projection 文档） |
| P1 | 去掉重复全局规划：`REPLAN_TRAJ` 复用刚规划的 `global_data_.global_traj_`；只有换目标或偏离 >0.3 m 才重算；局部失败先局部重试 | 每次目标少一次全程平滑；4 m 目标 1.5 s 额外开销基本消掉 |
| P2 | 降可视化频率 `grid_map.visualization_period_s: 0.2 → 1.0`；`max_ray_length: 5.0 → 3.0`（局部窗只有 5 m，A\* 已覆盖远处） | 单线程占用降 ~30–40%，排队抖动下降；注意这是配置项，不用改代码 |
| P3 | 若 P0/P1 后还不够：把 min-snap 换成块对角/带状求解（复用 A⁻¹ 块、LDLT 代替 inverse），或直接换稀疏 QP | 平滑再降 1 个数量级；改动集中在 `polynomial_traj.cpp` |
| P4 | 系统侧：验收时停掉 SLAM GUI / localization far test，把 scan_planner、定位、网卡中断分核；复核 `ROS_LOCALHOST_ONLY=0` 下订阅者的额外拷贝 | 消除 gate 547 ms 这类整机争抢；需回归验证 |

> 多线程 executor 不是"直接换个 executor"就能上的：`GridMap` 的 occupancy 缓冲同时被 FSM 的碰撞检查和建图回调读写，
> 必须先把共享状态做成快照/双缓冲，否则会引入数据竞争（现有文档也提醒过这一点）。

## 8. 复现

```bash
source /opt/ros/humble/setup.bash
source /opt/slam3d_release/setup_runtime.sh
source /home/demo/lyy/hivecore-navigation/install/setup.bash
export ROS_DOMAIN_ID=55 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOG_DIR=/home/demo/lyy/n01_runtime/tmp/roslog

# 单目标（探针会在目标附近自动选点；--goal 指定本文表中的点）
python3 scripts/n01_planning_latency_probe.py --goal 4.975 0.875 --timeout 15

# 从 launch 日志提取内部耗时
grep -E "Global PCD A\*|Global trajectory smooth|GridMap performance" \
  /home/demo/lyy/n01_runtime/logs/n01_navigation_launch_$(date +%Y%m%d)_*.log | tail
```

目标点可达性校验（本文用的方法）：PCD 按 z∈[0.30,1.50] 投影成 0.05 m 栅格，障碍膨胀 0.35 m，
取机器人所在自由连通域；表中 4 个点均在该连通域内。
