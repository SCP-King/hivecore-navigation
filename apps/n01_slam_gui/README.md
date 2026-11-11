# N-01 机器人总控制 GUI

统一管理“建图定位后端 → N-01 适配器 → SCAN 导航”的 Web 控制台。
建图定位后端可在 GUI 中选择：

- **甲方 SLAM3D Lite**：沿用甲方 desk、服务和 PCD 工作流；
- **自部署 FAST-LIO2**：使用 `n01_fast_lio` 的隔离话题、保存地图 ICP
  重定位、统一定位适配器和导航桥接。

FAST-LIO2 是 N-01 自有建图定位后端，提供两种正式模式：

- **离线模式**：建图并保存 PCD → 停止建图 → 加载 PCD/ICP 重定位 →
  tracking → 导航，操作语义与甲方后端一致；
- **实时模式**：本次 FAST-LIO2 会话原点定义为 `map`，建图、位姿适配、
  SCAN 实时点云地图与导航同时运行；不加载旧 PCD 静态层，停止时保存本次地图和 bag。

实时模式不能与历史 PCD/甲方地图坐标混用；切换回离线模式后必须重新加载并重定位。

GUI 实时模式操作：选择“自部署 FAST-LIO2”与“实时（边建图边导航）” →
“启动建图定位” → 等 `/robot_pose`、`/body_pose` 和适配链变绿 → “启动导航”。
此模式下“任务”选择被忽略，“加载地图/初始位姿”不生效，导航直接使用 SCAN
随实时点云更新的滑动三维占用图；顶栏 RViz 自动切换为实时地图视图。

两套后端互斥，最终都归一为 `/robot_pose`、`/loc_status`，再经
`n01_localization_adapter` 和 `n01_live_scan_bridge` 向导航提供
`/n01/localization/*`、`/body_pose` 与 `/cloud`。规划核心不感知后端差异。

2026-11-09 起改为**单页精简控制台**：投影栅格地图 + 建图/定位/导航手动控制 + 只读诊断。
不再使用旧版“状态与地图 / 操作台 / 数据源 / 可视化 / 节点体检 / 统一生命周期”六个标签页。

## 运行环境

- Ubuntu + ROS 2 Humble
- 已安装的 `/opt/slam3d_release` Lite 综合包
- Python 依赖：`nicegui`、`numpy`、`Pillow`，以及 Lite 消息包和 ROS 2 Python 接口

## 启动与退出

```bash
cd /home/demo/lyy/hivecore-navigation
bash scripts/n01 gui              # 默认 0.0.0.0:8123
bash scripts/n01 gui 8090         # 自定义端口
```

浏览器访问 `http://<机器人IP>:8123`。退出 GUI 请在启动终端按 `Ctrl+C` 或发送 SIGTERM。

顶部会显示 ROS 连接状态；未连接时旁边会显示具体原因（环境、worker 异常或旧进程）。代码更新后必须重启 GUI 进程，单纯刷新浏览器不会加载新的 Python 代码。
GUI 正常退出时会停止自己启动的 desk、RViz、Foxglove 和导航；异常退出由 `tools/desk_guard.py` 清理 GUI 专属进程组。

## 页面布局（2026-11-10 总控化）

页面按“共享地图 → 总控/适配器 → 建图 → 定位 → 导航 → 诊断”排列。
生命周期入口集中在总控区，各模块不再重复放置 service、RViz 和状态刷新按钮。
顶栏保留常驻 **RViz 启动/停止**：它按照当前后端和任务自动打开甲方定位/建图
视图或 FAST-LIO2 专属视图，用于发布初始位姿、观察匹配并手动进入跟踪。
高频状态使用紧凑网格展示，完整适配链、高级定位/导航字段和诊断详情默认折叠。

“节点管理”页支持按底盘与执行、传感器、建图定位、适配器、导航规划、
TF与机器人模型、可视化与GUI、其他分类筛选，展示同名数、状态、进程/PID归属、
管理入口与关键发布话题。选中节点可查看完整发布/订阅接口，并填入停止计划。
分类仅用于展示，不授予停止权限；未知/外部进程仍需通过原启动入口管理。
`/tf`、`/tf_static` 多发布者允许分别发布不同坐标边，不再一律显示为多源故障。

甲方定位启动前会检查 Livox：唯一 PointCloud2 发布者复用；没有驱动时启动；
重复或类型冲突时拒绝新增驱动。现有重复驱动需核对来源后从原入口停止。
导航启动前的适配器检查使用三次、每次两秒的直接 DDS 发现，避免 daemon
刚重启时的短暂空图误报。每次导航命令的完整输出保存到
`n01_runtime/logs/gui_navigation_*.log`，失败摘要直接显示于导航页。

### 整体项目生命周期（最上方）

- 状态：定位进程、建图进程、导航、录包、RViz（SLAM/导航）。
- **建图定位后端**：选择甲方 SLAM3D Lite 或自部署 FAST-LIO2；
- **任务 + 启动建图定位 / 启动导航**：分阶段启动，便于确认适配器数据已经就绪；
- **停止全部（保存建图）**：先保存建图 PCD、停录包，再停导航/desk/RViz。
- **刷新状态**；**使用地盘 RSP** 开关。
- “最近操作”显示服务调用/保存结果。

### 共享地图视图

- 地图源：`自动 / 定位地图 / 建图 / 导航栅格`；
- 2D 障碍投影，不是点云散点图；高度带 `ground_z+0.08 … +1.60`；
- 视野 10/20/40 m；点击地图填入导航目标 x/y；
- 橙色三角为机器人，灰线为轨迹，绿色十字为目标；
- 地面 z 默认 0.139，换地图/楼层/腿高时按现场调整。

### ① 建图模块

- 启动建图 + 录包 / 结束建图（保存 + 停录包）；
- 操作员只保留“启动建图”和“停止并保存”；底层服务由生命周期统一协调；
- 状态：建图节点、会话、录包路径、保存目录、建图点云数；
- 新鲜度：`/mapping_status`、`/cloud_map`、`/livox/lidar/geo_filtered`；
- 建图与 bag 是同一生命周期：启动先启 bag；结束先保存 PCD，再停 bag/desk；保存失败保留 bag/desk。

### ② 定位模块

- 启动定位 / 停止定位；
- 只展示“启动、选择/加载地图、开始重定位、初始位姿、进入跟踪、停止”；
- FAST-LIO2 在初始位姿触发 ICP 成功后由适配器自动进入 tracking，不调用甲方服务；
- 地图 PCD 路径 + 设置并加载地图；
- 初始位姿 x/y/yaw/frame + 发布初始位姿；
- 检查 TF 链；
- 状态：定位节点、会话、地图加载、匹配/fitness 阈值 0.045、位姿、局部地面、当前地图路径；
- 新鲜度：`/robot_pose`、`/loc_status`、`/localization_map`、`/livox/lidar/geo_filtered`，≤0.5s 绿、≤2s 黄、>2s 红；
- 地图加载成功只代表 `map_loaded=true`；要 `reloc_ok=true` 才能进入跟踪/导航。

### ③ 导航模块

- 启动导航 / 停止导航；
- 启动导航可视化（RViz）/ 停止导航 RViz（导航启动时已自带 RViz 会提示，不重复开）；
  - RViz 需要本机 X/Wayland 显示；GUI 会先用 `xdpyinfo` 实际探测，不再只看 `DISPLAY` 变量；
  - 无显示/无 Xauthority 时用诊断里的 Foxglove 桥；
- **导航 PCD + 地面 z**：可用“同步定位地图”“同步当前地面”从定位模块复制；
  - 导航 PCD 必须与定位当前 `map_path` 一致；不一致时状态显示“地图一致性：不一致”；
  - 地面偏差 >20 mm 显示“地面一致性：偏差”；
  - 改 PCD/地面后需要重启导航才生效；
- 导航目标 x/y/yaw + 发送导航目标；
- 状态：导航就绪、阻塞项、目标门控、当前位姿、规划路径、导航操作结果；
  - 规划路径来自 `/n01/live_navigation/hive2_candidate_path`，在地图上用青色线画出；
  - 路径按时间窗有效；过期后 GUI **保留最近一次非空路径**并用灰色显示，直到下一条新路径覆盖；
  - 发出新目标后状态显示“等待新路径”，新路径到达后替换旧线；
  - 发布导航目标后 GUI 分两阶段：全局 A*（默认 12 s，`N01_GLOBAL_TIMEOUT_S`）和局部 SCAN/CBF（默认 12 s，`N01_LOCAL_TIMEOUT_S`）；
  - 任一步失败立即报告原因/耗时、清除本次目标并允许重新选点；旧路径/过期路径不计入成功；
  - 上一个目标仍在规划时 GUI 拒绝新目标，事务失败/成功后自动释放；
  - 路径终点必须距目标 ≤ `N01_PATH_GOAL_TOL_M`（默认 0.30 m）；超过时继续等待，最终没有可达路径则失败，并显示终点偏差/规划器原因；
  - 失败目标在地图上用红色十字标记；“规划器反馈”显示 `/n01/live_navigation/hive2_feasibility` 的 status/reason；
  - 规划成功只代表候选路径生成，不代表底盘运动；导航仍是 planning-only；
- 新鲜度：导航就绪、目标门控、系统健康、位姿；
- 规划-only：不发布底盘控制。

### 诊断（折叠）

- 关键话题年龄、节点/重名/孤儿摘要；
- Foxglove 桥启停。

## 坐标系统一（2026-11-09）

详细记录：仓库 `docs/` 与 `config/reference/` 中的当前坐标系契约。

```text
map 原点 = 建图起始的初始 base_link（不是地面）
base_footprint = 厂商虚拟跟踪帧，base_link 下方 0.90，不是地面
base_link = 物理机体；/body_pose、SCAN 碰撞/目标都用它
ground = base_link - body_to_ground（当前 0.778 = 腿高 0.683 + 轮半径 0.095）
0.30 = 历史建图约定，只用于回放
```

GUI 启动定位/建图 desk 时自动传入：

- `<N01_NAV_REPO>/config/n01_static_tf_params.yaml`：不等待 IMU 标定，直接发布 `lidar_link→imu_link→livox_frame`；
- `config/n01_localization_lite.yaml` / `config/n01_mapping_lite.yaml`：`tracking_to_vehicle_xyz=[0,0,0.9]`，与厂商 RSP 的 `base_footprint→base_link=0.90` 一致；
- `launch_robot_state_publisher:=false`：机体 TF 由地盘 Hive2 RSP 独占。

N-01 导航侧 `/body_pose` 输出 `map→base_link`；动态障碍/SCAN 地面使用 `body_pose.z - 0.778`，不再使用历史 `-0.55`。

## 推荐操作顺序

### 定位

1. “启动定位” → desk 启动（含 IMU 滤波、cloud preprocessor、静态 TF）。
2. “activate_localization” → Relocalization。
3. 设置地图 PCD 并“设置并加载”；确认地图已加载。
   注意：`map_path` 是运行中节点的参数，必须先启动定位 desk；定位节点未上线时 GUI 会提示“尚未上线”，不会把原始 `Node not found` 直接抛给用户。
4. 用“初始位姿”给初值，等待 `reloc_ok=true`。
5. “enter_tracking” → Tracking。
6. 需要重新开始时“deactivate_localization”或回 Relocalization。

### 建图 + bag 录制（同一生命周期）

建图按钮和 bag 录制是**同一个动作**：点“启动建图 + 录包”同时启动建图 desk 和 `ros2 bag record`；结束时先保存 PCD，再停止 bag，最后停止 desk。

1. 停止定位（定位/建图互斥）。
2. “启动建图 + 录包” → 先启动 bag，再启动 mapping desk；bag 失败则不会启动 desk。
3. 录制默认话题：
   ```text
   /livox/lidar /livox/imu /livox/lidar/geo_filtered
   /imu/data /imu/rpy/filtered /odom
   /tf /tf_static /robot_pose /mapping_status /cloud_map
   ```
4. “save_map” 只保存地图快照，**不停止录包**。
5. “结束建图（保存+停录包）”或“结束建图 / 停止 desk”会依次：
   ```text
   deactivate_mapping（保存全分辨率 PCD）
   → 停止 bag（SIGINT，写 metadata）
   → 停止 mapping desk
   ```
   保存失败会保留 bag/desk，允许重试，不会静默丢图。
6. PCD 在 `save_dir`（统一参数为 `/home/demo/lyy/slam_data/maps`）；bag 在 `~/lyy/slam_data/bags/bag_YYYYmmdd_HHMMSS`。

可通过环境变量覆盖：

```bash
N01_MAPPING_BAG_DIR=~/lyy/slam_data/bags
N01_MAPPING_BAG_TOPICS=/livox/lidar,/tf,/robot_pose
```

GUI 退出、desk 异常退出或“停止全部 GUI 托管模块”都会先尝试保存建图，再停止 bag/desk，避免孤儿录包。

### 导航

1. 定位进入 Tracking、地图已加载、`reloc_ok=true`、`/robot_pose` 新鲜。
2. 在导航模块确认/同步：
   - 导航 PCD = 定位当前 `map_path`；
   - 地面 z = 当前局部地面（`bf_z + 0.122`，现场参考 0.123）。
3. “启动导航” → 生命周期入口先 stop 旧导航、status 复查、再 start。GUI 会用 `N01_NAV_PCD`、`N01_NAV_GROUND_Z` 传给 `n01_navigation.sh`；headless 默认 `N01_NAV_RVIZ=false`，避免 RViz 启动失败拖一个死进程。
3. 地图上点击或在右侧输入目标 x/y/yaw → “发送目标” → 确认。
4. 观察“导航”READY、Goal Gate、规划日志；规划失败不等于底盘动作。
5. “停止导航”或“停止全部 GUI 托管模块”。

## 安全约束

1. GUI 只对自己登记的独立进程组执行组级停止。
2. 定位与建图互斥；导航依赖定位。
3. 不执行 `killall ros2`、`pkill -f localization` 或全局节点清空。
4. Livox、IMU、底盘/腿控、控制器、外部 SLAM、独立定位适配器是保护对象。
5. 无法证明归属的残留只报告，不自动清理，并阻止重复启动。
6. 导航输出是规划候选，不是运动授权；`actuator_output=DISABLED`、`execution_authorized=false`、`safety_authorization=NEVER`。

## 检查

```bash
source /opt/slam3d_release/setup_runtime.sh
python3 -m py_compile slam_lite_gui.py tools/*.py tests/*.py
bash -n run_slam_lite_gui.sh tools/*.sh
python3 tests/test_lifecycle.py
python3 tests/test_worker_contract.py
python3 tests/test_console_contract.py
python3 tests/test_nav_map_contract.py
python3 tests/test_bag_lifecycle.py
python3 tests/test_navigation_lifecycle.py
```

`tests/test_lifecycle.py` 的 `ps has pgid` 在无 PID namespace 的宿主机上运行才有效；沙箱内 pgid 恒为 0，属环境限制。

## 项目结构

```text
apps/n01_slam_gui/
├── slam_lite_gui.py              # GUI、ROS worker、栅格投影、目标发布
├── run_slam_lite_gui.sh          # 由 scripts/n01 gui 调用
├── config/                       # 统一 0.90 的定位/建图参数
├── tools/                        # 看门狗、导航生命周期桥接、诊断
├── tests/                        # 生命周期/导航/worker 契约测试
└── docs/                         # 详细手册与故障排查
```
