# N-01 规划性能优化实施记录（2026-11-11）

## 1. 改动

### 1.1 A\* 路径重采样（`plan_manage/src/planner_manager.cpp`）

- `globalAStar2D()` 新增 `waypoint_spacing` 参数。
- A\* 原始输出是 **每 0.05 m 一个格点**（4 m 目标 80 点、6 m 目标 124 点），
  现在在返回前按 **0.30 m** 做贪心 LOS 重采样：
  - 每条保留的弦都按 `resolution/2` 在**同一张 inflated 2D 栅格**上采样校验，
    穿障碍的弦不会被保留，所以不会为了降点数去切障碍角；
  - 弯道/窄道处无法拉长时自动退回逐格，安全性与原路径一致。
- 新参数 `manager.global_waypoint_spacing`（默认 0.30 m），已写入
  `src/n01_live_navigation/launch/live_scan.launch.py`。

### 1.2 min-snap 求逆复用（`traj_utils/src/polynomial_traj.cpp`）

原实现对同一个 `A` 求逆 **5 次**、同一个 `Rpp` 求逆 **3 次**、`A.inverse()*Ct` 算 **3 次**。
改为：

- `A_inv = A.inverse()` 一次，R 与 Px/Py/Pz 共用；
- `Rpp_inv_Rfp_T = Rpp.inverse() * Rfp.transpose()` 一次，三轴共用；
- `A_inv_Ct = A_inv * Ct` 一次，三轴共用。

### 1.3 构建

```bash
colcon build --packages-select traj_utils plan_manage scan_planner n01_live_navigation
```

Release `-O3 -DNDEBUG`，`build/scan_planner/scan_planner_node` 已更新（14:20:53）。

## 2. 验证

### 2.1 独立基准（隔离核 cpu7，同一份 `minSnapTraj`）

| 路径点数 | 优化前 | 优化后 | 倍数 | 航点误差 |
|---:|---:|---:|---:|---:|
| 7 | 0.34 ms | 0.10 ms | 3.4× | 8e-15 m |
| 20 | 6.43 ms | 2.24 ms | 2.9× | 1.5e-14 m |
| 39 | 41.1 ms | 15.6 ms | 2.6× | 1.5e-14 m |
| 80 | 311 ms | 118 ms | 2.6× | 1.3e-14 m |
| 119 | 977 ms | 374 ms | 2.6× | 1.6e-14 m |
| 160 | 2306 ms | 886 ms | 2.6× | 1.3e-14 m |

求逆复用本身约 **2.6×**；真正的量级来自重采样：6 m 目标 124 → ~20 点，min-snap 从 ~1 s 降到 **~2 ms**。

### 2.2 隔离真实地图端到端（ROS_DOMAIN_ID=88，不碰 live 栈）

- 地图：`mapping_lite_20261109_140536_087.pcd`（202,764 个静态体素，`ready=true`）。
- 起点 (1.005, 0.385)，目标 **(4.975, 0.875)**，4 m。
- 结果：

| 项 | 旧进程（live 实测） | 新二进制（隔离实测） |
|---|---:|---:|
| A\* 点数 | 80 | **17** |
| A\* 耗时 | 3.6 ms | 3.1–5.9 ms |
| min-snap | 512.7 ms（p50 291） | **1.1–6.0 ms** |
| 目标→global_path | 872 ms | **35.7 ms** |
| 目标→B 样条 | 2364 ms | **39.7 ms** |

隔离测试里 10 s 内做了 7 次重规划，每次 smooth 1.1–1.7 ms，说明 REPLAN 循环的代价也一起消失了。

### 2.3 回归

- `test_uniform_bspline`：PASS（2/2）。
- `test_static_inflation`：PASS。
- `g1-smoke`：FAIL，原因是 `Global PCD A* unavailable: static PCD source is not ready` ——
  该隔离 launch 不加载静态 PCD，和当前 static-PCD 全局规划本来就不兼容，**不是本次改动引入的**。

## 3. 预期 live 效果与边界

- 规划计算：4 m 目标 **~7–12 ms**（A\* 4 + min-snap 1–6 + 局部 2–4）。
- 剩余时延主要不再是算法：
  - 单线程 executor 被 80 ms 建图回调挡住 → goal 派发 4–357 ms（这是下一步的 P2）；
  - `REPLAN_TRAJ` 仍会重复一次全局规划（现在只多 ~5 ms，可后续清掉）；
  - 候选路径 `trajectory_monitor` 有 0.25 s 定时重发，端到端仍可能看到 ~0.25 s。
- 因此重启后 live 的 goal→global 预计 **~40–150 ms（中位）**，而不是现在的 0.9–1.5 s。

## 4. 部署（必须重启导航栈）

运行中的 `n01_navigation_launch_20261111_134503.log` 是 **13:45 启动的旧进程**，
仍在使用旧二进制（日志里 `path=124 / smooth~1000 ms`）。ROS 2 不会热加载 C++ 库，
需要在设备上重启导航栈（不要重启定位/驱动）：

```bash
cd /home/demo/lyy/hivecore-navigation
bash scripts/n01 navigation stop
bash scripts/n01 navigation start   # 或按原流程从 GUI 启动
```

重启后用同一个目标点复测：

```bash
python3 scripts/n01_planning_latency_probe.py --goal 4.975 0.875 --timeout 15
grep -E "Global PCD A\*|Global trajectory smooth" \
  /home/demo/lyy/n01_runtime/logs/n01_navigation_launch_$(date +%Y%m%d)_*.log | tail
```

预期日志变为 `path≈17`、`smooth_ms≈1–6`。

## 5. 复现资料

- 基准程序：`/home/demo/lyy/n01_runtime/tmp/bench/min_snap_check.cpp`
  （`g++ -O3 -I install/traj_utils/include -I /usr/include/eigen3 ... install/traj_utils/lib/libtraj_utils.a`）。
- 隔离测试：`/home/demo/lyy/n01_runtime/tmp/iso_test/run_iso.sh` + `harness.py`；
  日志 `planner.log`、`harness.log`、`static.log` 在同一目录。
- 旧 vs 新数据来源：`n01_navigation_launch_20261111_134503.log` 与隔离 `planner.log`。
