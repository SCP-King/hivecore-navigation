# N-01 导航项目交接与问题文档（给网页版 AI）

> 目标：让接手方在**不改甲方 SLAM 算法、不启动底盘执行**的前提下，把 N-01 导航的**地图一致性、全局/局部规划、可观测性和 GUI**修到可验收。
>
> 当前所有改动均未提交 git；以工作区代码为准。

---

## 1. 项目与环境

- 工作区：`/home/demo/lyy/hivecore-navigation`
- 机器人：HiveCore / Hive2 四足，4WIS 底盘
- ROS 2 Humble，C++17，Python 3.10，NiceGUI
- 新 SLAM 运行时：`/home/demo/lyy/slam3d_20261118`
- 旧运行时：`/home/demo/lyy/slam3d_lite_20261106_test`（仅兼容/回退）
- GUI 启动：`bash scripts/n01 gui`
- 主验收模式：**离线 PCD 导航**；实时建图为试验模式
- 安全边界：**只规划不执行**，不要启动 controller/底盘运动

### 环境

```bash
source /opt/ros/humble/setup.bash
source /home/demo/lyy/slam3d_20261118/setup_runtime.sh
source install/setup.bash
export ROS_DOMAIN_ID=55
```

注意：

- 环境里可能残留 `SLAM_ROSK`，指向旧树；启动前 `unset SLAM_ROSK`。
- 新运行时安装前缀：`/home/demo/lyy/slam3d_20261118/install/hivecore_slam3d_lite_localization`。

---

## 2. 当前数据流

```text
建图：
  /livox/lidar
  → /livox/lidar/geo_filtered（甲方 ego box）
  → mapping_lite
  → /cloud_map
  → 保存 PCD

定位：
  保存的 PCD → 甲方 localization_lite / MOLA
  → /n01/localization/pose / /localization_map

导航：
  PCD → n01_static_map（0.10 m、z −1.2~1.2、地面 6 cm）
  → /grid_map/static_occupancy
  → GridMap 局部 3D / 持久全局 2D
  → 全局 A* → 局部 SCAN → 轨迹包
  → 执行器（默认不启动）
```

---

## 3. 当前功能状态

### 已实现

- 离线 PCD 导航、全局 A*、局部 SCAN、恢复阶梯；
- 起点/终点包络（URDF 足迹 + 6 cm，支持双圆柱切换）；
- 当前足迹自车过滤；
- 本次运行历史足迹过滤；
- 静态源 0.10 m、z −1.2~1.2；
- raycast 0.10 m 体素去重；
- 离线静态层不再每帧刷新；
- 全局层窗口内用局部 reference/live 覆盖（最新代码，已编译，未实机复测）；
- 起点清理并回写持久全局层（最新代码，已编译，未实机复测）；
- 全局诊断：start/goal 格、连通域、free/occupied/unknown；
- 地面变化使全局层失效；
- GUI 可达连通域白底（最新代码改为全局连通域，未实机复测）；
- GUI 紫色静态/全局配色、青色实时、琥珀重合、绿色清除；
- PCD 工具：轨迹录制、bag 提取、历史自车清理；
- 故障域/动作/证据；
- 显示生命周期清理。

### 未实现/未完成

- 定位地图与导航地图的统一身份（map_id/hash）；
- 定位与导航共用同一份预处理地图；
- 全局层完全由局部/静态派生；
- 全局自由空间碎片化修复；
- 静态层异步/双缓冲、2 m 瓦片；
- raycast OpenMP；
- 局部 SCAN 前缀/热启动；
- MultiThreadedExecutor 回调隔离；
- 甲方建图自车过滤覆盖不足；
- 建图在新运行时完整验收；
- 动态/8 h/95% 正式测试。

---

## 4. 实机问题（最重要）

### 4.1 地图不是“同一张”

严格说目前是：

```text
同一份 PCD → 定位副本
            → 导航静态副本
            → 局部 3D 副本
            → 全局 2D 副本
```

不是运行时共享内存地图。

- 定位节点加载自己的 `map_path`；
- 导航 `n01_static_map` 加载自己的 PCD；
- 两者路径由脚本/GUI 约定，但没有统一 `map_id/hash` 运行时校验；
- 定位与导航的过滤规则不同（定位用甲方地图，导航再 0.10 m/z/地面过滤）。

如果定位和导航选到不同 PCD，就是真正的“两张图”。

### 4.2 全局层与局部层不一致

实机证据（远程 10–20 m）：

```text
从全局自由层选 10 个点：
  8 个被局部层判 goal_footprint_occupied

全局 free ≈ 6,300–6,700
全局 occupied ≈ 18,000
全局 unknown ≈ 239,000
global_start_cell = 100
global_goal_component = 15 / 6366
```

含义：

- 全局层认为自由，局部层认为障碍/膨胀；
- 全局起点格仍是 100；
- 目标可能在 15 格级孤立小岛；
- 全局自由空间被地面/自车/旧实时点切碎。

### 4.3 起点被全局层占用

修复前：

```text
机器人位置 (-0.68,-4.83)
global_grid 该格 = 100
```

导致：

```text
10 个近场点：1 LOCAL_READY，9 global_no_route
```

已修：

- `GridMap::clearGlobalStart(x,y,0.40)`；
- 规划前清起点并回写持久 `global_grid_`；
- 修复后近场 10/10 `LOCAL_READY`（在“只清 A* 副本”的版本上验证；最新回写版本未实机复测）。

### 4.4 远程 10–20 m 仍失败

修复起点后，10–20 m：

```text
0/8 成功
失败：goal_occupied / goal_footprint_occupied / global_no_route
```

从局部参考层选 8 个 10–20 m、0.8 m 余量的点，仍全部失败。

原因：

- 远处局部自由空间少（同连通域 10–20 m 只有几十到几百个自由格）；
- 目标在全局层孤立小连通域；
- 全局自由与局部实际地图不一致；
- 地图/地面/自车残留。

### 4.5 地图质量

最新 PCD（以现场为准）：

```text
/home/demo/lyy/slam_data/maps/mapping_lite_20261119_213108_031.pcd
```

统计：

| 项 | 值 |
|---|---|
| 点数 | 约 49 万 |
| 范围 | 约 59 × 42 m |
| z 范围 | 约 −2.1 ~ 3.4 m |
| z > 1.2 | 约 14.9 万点 |
| 全局 free | 约 6k |
| 全局 unknown | 约 239k |

问题：

- 高处点占比高（天花板/上层）；
- 地面/低处点可能被当地面或障碍；
- 历史自车拖影存在；
- 动态物体可能已烘入 PCD；
- 最新地图没有配套轨迹，无法量化拖影。

### 4.6 自车拖影

已量化案例（旧图）：

- bag：`bag_20261119_205530`
- PCD：`mapping_lite_20261119_205848_755.pcd`
- 轨迹 369 位姿
- 清理删除 **3,685 点（0.75%）**，1,015 个历史足迹格

说明：

- 甲方 ego box 开启，但 z 下限 −0.3 可能漏腿/轮；
- 历史点会进入 `/cloud_map`；
- 运行期当前足迹过滤清不了历史；
- 已有离线工具，但需要 bag/轨迹。

### 4.7 地面/天花板导致“突然换图”

现场现象：

- 建图视图看不到地面；
- 导航局部视图也没有明显障碍；
- 发目标后全局层突然出现地面静态障碍。

原因：

- 全局层独立过滤，地面/高度/更新时间与局部不同；
- 4 cm 地面点在不同层结论不同；
- 全局底图把窗口内旧点也画出来（最新代码已限制窗口外）。

已修：

- 地面变化使全局失效；
- 6 cm cutoff；
- 全局底图只画局部窗口外；
- 局部/全局/显示 6 cm。

### 4.8 GUI 断连/带宽

- `Invalid HTTP request received` 是 uvicorn 告警，通常是端口扫描/代理探测，不一定是断连根因；
- 地图图片过大/刷新过快、事件循环阻塞、CPU 抢占、SSH/代理 WebSocket 超时是更可能原因；
- 已做：图幅 420、约 3 Hz、PNG 压缩、binding 0.25 s、重连 15 s、事件循环 watchdog。

---

## 5. 关键代码位置

| 功能 | 文件 |
|---|---|
| GUI 主程序 | `apps/n01_slam_gui/slam_lite_gui.py` |
| 地图显示/连通域/配色 | `slam_lite_gui.py`：`_project_occupancy`、`_reachable_mask`、`_project_reachable`、`_GRID_*` |
| 故障域/动作 | `apps/n01_slam_gui/navigation_flow.py` |
| 导航 launch | `src/n01_live_navigation/launch/live_scan.launch.py` |
| 静态图节点 | `src/n01_static_map/n01_static_map/static_map_node.py` |
| 地面参考 | `src/n01_live_navigation/n01_live_navigation/ground_reference_node.py` |
| GridMap | `src/scan_planner/plan_env/src/grid_map.cpp` |
| GridMap 头 | `src/scan_planner/plan_env/include/plan_env/grid_map.h` |
| 全局 A* / 规划管理 | `src/scan_planner/plan_manage/src/planner_manager.cpp` |
| FSM / 状态 | `src/scan_planner/plan_manage/src/scan_replan_fsm.cpp` |
| PCD 工具 | `apps/n01_slam_gui/tools/` |

### 最近的改动

1. `GridMap::clearGlobalStart`：清起点并更新 `global_cloud_`；
2. `planner_manager::planGlobalTraj`：规划前调用；
3. `GridMap` 缓存 `reference_grid_data_`/`live_grid_data_`；
4. `rebuildGlobalGrid`：窗口内局部 reference/live 覆盖全局；
5. `slam_lite_gui.py`：全局连通域白底、紫色系、诊断显示；
6. `/planning/status`：`fault_evidence` 增加全局诊断；
7. 地面变化清空全局层；6 cm cutoff。

---

## 6. 复现与测试

### 启动

```bash
cd /home/demo/lyy/hivecore-navigation
bash scripts/n01 gui
```

在 GUI 中启动定位/导航，**不要启动执行器**。

### 发目标

- GUI 地图点选；
- 或：

```bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 3.23, y: -0.20, z: 0.0}, orientation: {w: 1.0}}}"
```

### 观察

```bash
ros2 topic echo /planning/status
ros2 topic echo /n01/ground_reference/status
ros2 topic echo /grid_map/global_grid
```

### 已做测试

- 近场 2–9 m：10/10 `LOCAL_READY`（起点清理版本）；
- 远程 10–20 m：0/8 成功；
- 从全局自由层选远程点：8/10 被局部判 `goal_footprint_occupied`；
- 回放 3/4 `LOCAL_READY`；
- GUI 测试 11 项通过；
- 路径/地图测试通过。

---

## 7. 建议修复顺序

### P0：统一地图身份与来源

1. 定义 `map_id = PCD 路径 + sha256 + 时间戳`；
2. 建图保存、定位加载、导航加载都写/校验同一 ID；
3. GUI 显示当前 map_id；
4. 不一致直接拒绝启动/发目标；
5. 定位和导航共用同一份静态源（或至少同一 PCD + 同一预处理）。

### P0：全局/局部真正同图

1. 验证最新“窗口内局部覆盖全局”；
2. 窗口边界规则：
   - 局部已知 0/100 覆盖；
   - 局部未知保留全局；
3. 全局 A* 只用覆盖后的全局层；
4. 目标发布前：
   - 局部足迹复核；
   - 全局连通域复核；
5. 失败原因细分：
   - `global_goal_unreachable_component`
   - `global_start_unreachable`
   - `goal_local_occupied`

### P0：地图质量

1. 用 bag 提取轨迹；
2. 清理 PCD 历史自车；
3. 检查地面高度/坡度；
4. 重新生成静态图；
5. 再做 10–20 m 测试。

### P1：性能

- 静态层异步/双缓冲；
- 2 m 瓦片；
- raycast 并行；
- 全局脏 tile；
- 局部 SCAN 前缀；
- 执行器隔离。

### P1：GUI

- 保持白底 + 紫色配色；
- 图例明确未知/已知空闲不可达/可达/静态/实时；
- 保持 3 Hz/420 图幅，弱网可再降。

---

## 8. 验收标准

1. 定位、导航使用同一 `map_id`；
2. 窗口内全局/局部对同一格结论一致；
3. 近场 10 点 ≥9/10 `LOCAL_READY`；
4. 10–20 m 10 点 ≥8/10 `LOCAL_READY`（在地图连续可通行前提下）；
5. 起点不再 `global_start_cell=100`；
6. 失败原因能定位到层/格/连通域；
7. 无执行器动作；
8. 不回归回放和 GUI 测试。

---

## 9. 已知坑

- `AppState`/`RosWorker` 类归属错误会导致 GUI `has no attribute`；
- 静态层/height filter 曾导致 scan planner 段错误，改线程/高度过滤要谨慎；
- `realtime_reference_topic:=`/`map_id:=` 空参数曾导致 launch exit=5；
- 目标 z 不要传地图地面 z（如 −0.9），用 0，规划器会取地面高度；
- `/body_pose` 是 best effort QoS；
- 新运行时与 N-01 的 frame 适配是 `lidar_link + 零偏移`，不要混用产品 `base_footprint` 配置；
- 旧 PCD 历史拖影必须离线清理，运行期只能清当前/本次运行历史足迹。

---

## 10. 交付给网页版 AI 的第一句话

> 请先读 `docs/HANDOFF_WEB_AI_FIX.md` 和 `docs/planning_scheme_gaps.md`。当前核心问题不是 A* 参数，而是定位/导航/全局/局部不是同一张地图，以及全局自由空间碎片化。请优先做地图 ID 一致性、窗口内局部权威覆盖、目标连通域预检；不要启动底盘执行器。
