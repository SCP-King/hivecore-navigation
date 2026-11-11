# N-01 药厂导航避障

> 本仓库是 N-01 Navigation 算法与集成工程。当前交付范围是**定位接入、实时感知、静态/动态场景输入、原生 SCAN 规划候选、轨迹审计和安全状态诊断**；不是已认证的自主驾驶系统，不包含运动执行授权。

本文是仓库**唯一维护中的工程说明文档**。架构、接口、开发、运行、测试和安全约束统一维护在这里；历史测试记录以本文件中的结论摘要及 `config/reference/` 机器可读档案为准。运行日志、Bag、PCD、录屏和报告放在 `~/lyy/n01_runtime/`，不得提交到 Git。

---

## 1. 先看这里：唯一入口与工作流

从仓库根目录执行：

```bash
bash scripts/n01 help
```

日常开发只使用远程分支 `dev/n01-current`；`main` 是稳定发布分支。禁止创建任务、日期、feature、fix 或 refactor 临时分支。

```bash
git status --short --branch
git switch dev/n01-current
git pull --ff-only origin dev/n01-current
```

| 命令 | 用途 | 运行边界 |
|---|---|---|
| `bash scripts/n01 environment-preflight` | 主机、ROS、工作区和路径检查 | 只读 |
| `bash scripts/n01 planning-audit` | 规划阻塞项证据汇总 | 只读 |
| `bash scripts/n01 live-preflight [秒数]` | Live Stack 话题、时间戳、TF 预检 | 只读观察 |
| `bash scripts/n01 robot-pose-audit` | `/robot_pose` 发布端点与时间序列 | 只读观察 |
| `bash scripts/n01 gui [端口]` | 启动仓库内 SLAM3D Lite + Navigation Web GUI | 显式按钮管理 GUI 自有进程 |
| `bash scripts/n01 deploy` | 构建并按门控部署 N-01 规划栈 | 会构建、重启 N-01 栈；保留厂商 SLAM |
| `bash scripts/n01 navigation start\|status\|stop` | Live Stack 生命周期管理 | 只管理 N-01 自有进程 |
| `bash scripts/n01 planning-smoke [参数]` | 已运行栈上的 READY 门控单次规划 | 最多发一个规划目标，不发运动命令 |
| `bash scripts/n01 planning-acceptance` | 实时目标到 SCAN 轨迹验收 | 构建、启动、验收并自动停止 N-01 栈 |
| `bash scripts/n01 g1-smoke` | G1 隔离短冒烟 | 离线 ROS domain 88 |
| `bash scripts/n01 g1-acceptance` | G1 四场景完整验收 | 离线 ROS domain 88 |
| `bash scripts/n01 bag-functional` | 历史真实 Bag 功能回放 | 离线、只读 Bag |
| `bash scripts/n01 offline-framework` | 历史离线框架集成 | 隔离运行 |
| `bash scripts/n01 g4-research` | PCD/Bag 自由空间研究工具链 | 离线研究，不输出导航执行命令 |
| `bash scripts/n01 fast-lio-import\|build\|bag` | 可选 FAST-LIO2 后端 | 独立命名空间与私有 TF |
| `bash scripts/n01 demo-replay` | 历史 Bag 的 RViz 演示回放 | 离线演示 |
| `bash scripts/n01 record-demo OUTPUT.mp4 [RVIZ_PID]` | 录制 RViz 窗口 | 仅录屏 |
| `bash scripts/n01 fusion-min-loop\|fusion-phase-c\|fusion-bag-replay` | Map Fusion 研究流程 | 隔离研究 |
| `bash scripts/n01 static-map-bag-replay` | 静态地图/SCAN Bag 回放 | 隔离研究 |

各工作流的参数和副作用以脚本自身的参数校验与运行输出为准。不要绕过统一入口直接启动厂商 SLAM、底盘或控制 SDK。

---

## 2. 系统边界与软件架构

### 2.1 职责分层

```text
外部系统（独立运行，不由本仓库管理）
  HiveCore SLAM3D Lite ─ Livox / IMU ─ Hive2 Motion / Robot State Publisher
             │ /loc_status, /robot_pose, /odom, TF, PointCloud2
             ▼
N-01 Localization Adapter（唯一厂商定位消息边界）
             │ /n01/localization/status
             │ /n01/localization/pose
             ▼
N-01 Live Navigation
  ├─ 定位状态与数据新鲜度门控
  ├─ 扫描时间戳位姿插值、传感器外参组合
  ├─ 静态 PCD 地图输入
  ├─ 原始实时点云进入 native SCAN
  ├─ Goal Gate / runtime supervisor
  ├─ 动态障碍候选诊断（可视化，不是安全传感器）
  ├─ B-spline 轨迹时间域与 Hive2 必要条件筛查
  └─ fail-closed safety status
             ▼
      规划候选、状态、诊断与 RViz 可视化
             X
      不连接底盘，不输出执行器命令
```

核心原则：

- 厂商 SLAM、机器人描述、Livox 驱动、Hive2 Motion SDK 和硬件启动文件均为外部依赖。
- N-01 核心只使用标准 ROS 消息与内部规范接口；只有 Localization Adapter 可以解析厂商定位状态消息。
- 不复制、不构建、不启动第二套厂商 SLAM，不修改厂商 URDF、launch、驱动或运动栈。
- FAST-LIO2 是可选、隔离的研究后端，不是 Lite SLAM 的自动替代品。
- 规划 READY 只表示可以向规划器提交目标，不等于执行授权。

### 2.2 ROS 包职责

| 包 | 责任 |
|---|---|
| `n01_bringup` | 传感器入口、bringup 与 RViz 配置 |
| `n01_perception` | 传感器到地图的感知适配 |
| `n01_scan` | SCAN 输入与在线地图相关适配 |
| `n01_live_navigation` | 定位 Adapter、实时桥接、目标门、运行/安全监督、轨迹审计 |
| `n01_static_map` | 静态 PCD 地图服务与占用层输入 |
| `n01_map_fusion` | 静态/动态地图融合及全局路由研究 |
| `n01_offline_integration` | Bag 回放、离线证据、集成测试 |
| `n01_fast_lio` | 可选隔离 FAST-LIO2 与里程计适配 |
| `scan_planner` | 上游 SCAN Planner；保留上游身份和接口 |

非 ROS 应用统一放在 `apps/`：`apps/n01_slam_gui/` 是 SLAM3D Lite、建图、定位和导航的 Web 操作台。它不参与 `colcon` 包发现，通过 `bash scripts/n01 gui` 启动，并通过仓库相对路径调用导航生命周期脚本。

仓库顶层按职责整理如下：

```text
hivecore-navigation/
├── apps/                   # 非 ROS 操作应用（Web GUI）
├── config/                 # 跨模块机器可读契约与统一参数
├── docs/                   # 工程说明、现场记录与验收文档
├── rviz/                   # 统一 RViz 配置
├── scripts/                # 唯一命令入口及安全生命周期脚本
├── src/                    # ROS 2 包与固定来源的第三方源码
├── build/ install/ log/    # colcon 生成物，不纳入版本控制
└── README.md               # 当前工程唯一维护中的总说明
```

ROS 包名、节点名、topic、frame、QoS、launch 参数都是集成契约。整理文件不得顺手重命名这些接口。

---

## 3. 定位接口与时间同步契约

### 3.1 外部输入与内部接口

当前代码配置文件：`src/n01_live_navigation/config/slam3d_lite_adapter.yaml`。

| 用途 | 外部输入 | N-01 内部输出 |
|---|---|---|
| 定位状态 | `/loc_status`（运行时发现消息类型） | `/n01/localization/status`（`diagnostic_msgs/DiagnosticArray`） |
| 机器人位姿 | `/robot_pose`（`nav_msgs/Odometry`） | `/n01/localization/pose`（`nav_msgs/Odometry`） |

状态 Adapter 只映射已核验的字段：`session`、`map_loaded`、`reloc_ok`、`reloc_fitness`。预期 session 为 `tracking`；状态时间戳必须非零、字段映射有效、地图已加载、重定位成功，消费端还必须检查接收新鲜度。未知字段、映射错误和无效数值 fail-closed。不得臆造状态消息没有提供的 TF 有效、IMU 同步或一般跟踪质量字段。

Odometry Adapter 会原样转发每条 `/robot_pose`，包括 frame/stamp 不符合预期的样本；frame 和时间戳错误计入诊断，由下游门控决定能否使用。输入/输出使用 RELIABLE KeepLast QoS，默认深度 50。默认期望 `map → base_footprint`。

**禁止把旧文档中其他版本的 `/localization_status`、`hivecore_slam3d/msg/LocalizationStatus` 或不同字段映射复制进当前 Lite profile。** 当前有效配置以 YAML 与 Adapter 代码为准；更换后端时新增 Adapter/profile，不修改规划核心。

### 3.2 扫描时刻配准

- 实时点云输入：`/livox/lidar/geo_filtered`，`sensor_msgs/PointCloud2`，预期 frame 为 `lidar_link`。
- 规划器输入：`/cloud`（map frame）、`/sensor_pose`、`/body_pose`。
- `/cloud` 与 `/sensor_pose` 必须保留同一原始扫描时间戳，供 native SCAN ExactTime 同步。
- 全局定位由时间戳 Odometry `map → base_footprint` 提供；传感器外参由 TF 获取 `base_footprint → base_link → lidar_link`。
- 桥接按原始 scan stamp 对位姿做插值（位置线性插值、姿态 quaternion SLERP），再合成传感器位姿；不使用 latest TF，不把最新位姿冒充扫描时刻位姿，不改写源时间戳。
- 当前配置的有界预算包括 pose receive age 0.7 s、pose stamp age 1.5 s、扫描锚点偏差 1.25 s、当前 body pose 锚点偏差 1.4 s、里程计插值相邻样本最大间隔 0.08 s、cloud age 0.9 s、scan lag 0.7 s、TF wait 0.55 s。它们是当前代码配置，不是上游性能保证；不能为获得 READY 随意放宽。

---

## 4. 坐标系统一规范

唯一机器可读参考：`config/reference/n01_frames_unified_20261109.yaml`。

| Frame | 角色 | 关键关系 |
|---|---|---|
| `map` | SLAM 地图坐标系，PCD 规划坐标系 | 建图起始时 `base_link` 位置为原点，重力只对齐一次 |
| `base_footprint` | 厂商 SLAM 跟踪帧 | `map → base_footprint`；不是地面 |
| `base_link` | 真实机体参考帧，N-01 `/body_pose` child | `base_footprint → base_link = +0.90 m` |
| `lidar_link` | 过滤后 Livox 点云 frame | `base_link → lidar_link = (+0.295,0,0) m` |
| `imu_link` | Livox IMU 安装 frame | 相对 `lidar_link` 的旋转按 YAML |
| `livox_frame` | 原始 Livox frame | 子于 `imu_link` |

局部地面定义为：

```text
ground_z = body_pose.z - body_to_ground
body_to_ground = 0.778 m
leg_foot_drop = 0.683 m
wheel_radius = 0.095 m
当前局部 ground_z_map = 0.139 m（需按地图、地面和姿态复测）
```

历史建图使用的 0.30 m 只用于历史 Bag/地图兼容研究，不是当前 Live 机体关系。不得把 `base_footprint` 当地面，不得仅通过改 frame_id 做坐标转换。PCD 与当前 SLAM map 是否同一坐标系必须由操作者确认；`pcd_map_frame_asserted:=true` 是显式断言，不是自动配准证明。

---

## 5. Live Navigation 运行方式

### 5.1 生命周期

常规启停通过生命周期入口：

```bash
bash scripts/n01 navigation start
bash scripts/n01 navigation status
bash scripts/n01 navigation stop
```

只允许一套 N-01 `live_scan.launch.py`。启动有 domain-scoped flock 与 ROS graph 检查；生命周期管理只处理 N-01 自己拥有的进程组。停止时先 SIGINT，再按需 SIGTERM/SIGKILL，并核验 ROS 图清理。不得误停独立运行的 Lite SLAM、Livox、IMU、robot-state publisher 或独立定位 Adapter。

### 5.2 构建与部署

推荐部署入口：

```bash
bash scripts/n01 deploy
```

如需手动只构建相关包：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to n01_live_navigation scan_planner n01_static_map \
  --event-handlers console_direct+
colcon test --packages-select n01_live_navigation
colcon test-result --verbose
```

部署会构建并按安全门控重启 N-01 live stack，不会启动或停止厂商 SLAM。现场部署前先检查 Git 工作树和外部进程；失败时不得通过全局 `pkill`、重启 ROS daemon 或删除整个 build/install 来掩盖问题。

### 5.3 启动 PCD 与 RViz

手动 launch 仅在确认 PCD 与当前 Lite map 已配准后使用：

```bash
ros2 launch n01_live_navigation live_scan.launch.py \
  pcd_path:=/absolute/path/to/registered_map.pcd \
  pcd_map_frame_asserted:=true \
  ground_z_map_m:=0.139 \
  body_height_m:=0.90 \
  rviz:=true
```

PCD 路径必须是绝对路径。未确认 map-frame 一致时，不得设置 assertion 为 true。默认不在 launch 内重复启动 Localization Adapter；只有确认独立 Adapter 已停止后才设置 `start_localization_adapter:=true`。

### 5.4 关键运行话题

| Topic | 含义 |
|---|---|
| `/n01/live_navigation/readiness` | 点云/位姿桥接计数、时间戳与丢帧原因 |
| `/n01/live_navigation/navigation_ready` | fail-closed 目标就绪 Bool |
| `/n01/live_navigation/system_health` | 多阶段运行健康摘要 |
| `/n01/live_navigation/goal_gate_status` | RViz 目标接受/拒绝原因 |
| `/n01/live_navigation/scan_body_pose` | 精确扫描时刻机体位姿诊断 |
| `/planning/bspline` | native SCAN B-spline 候选轨迹 |
| `/n01/live_navigation/hive2_candidate_path` | Hive2 候选路径 |
| `/n01/live_navigation/hive2_wheel_kinematics` | 名义轮转向可视化 |
| `/n01/live_navigation/hive2_feasibility` | 非执行型必要条件审查 |
| `/n01/live_navigation/dynamic_obstacle_candidates` | 静态地图未匹配点候选，可视化用途 |
| `/n01/live_navigation/dynamic_obstacle_tracks` | 候选聚类 Marker |
| `/n01/live_navigation/dynamic_obstacle_status` | 候选跟踪诊断 |
| `/n01/live_navigation/safety_status` | fail-closed 安全审查状态 |

---

## 6. 测试与验收等级

任何一级通过都不能推导更高级别通过。

| 级别 | 验证内容 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| 1 静态检查 | Python 合同测试、Bash 语法、CI | 代码结构/合同未明显破坏 | ROS 构建、现场运行 |
| 2 主机构建 | Ubuntu 22.04、ROS 2 Humble、colcon | 目标主机可编译 | DDS 实时链路 |
| 3 离线隔离 | G1、Bag、G4、Map Fusion 回放 | 指定数据/场景下离线功能 | 当前 SLAM tracking、实机安全 |
| 4 现场只读 | ROS graph、QoS、TF、时间戳、数据新鲜度 | 当前现场输入/接口状态 | 路线无碰撞、硬件安全 |
| 5 实时规划 | Goal Gate、native SCAN、B-spline、候选运动学 | 规划候选链路可工作 | 控制器可执行、物理停车 |
| 6 物理安全验收 | 急停、失联、定位丢失、碰撞包络、制动、控制权互斥 | 经独立评审后可形成硬件安全结论 | 不由本仓库软件测试自动替代 |

### 6.1 正式测试入口

- `g1-smoke`：隔离域快速检查。
- `g1-acceptance`：真实 SCAN Planner 四场景（clear、blocked、changed、no_odom）；验证轨迹对真实膨胀体素的离线采样碰撞检查和清理结果。
- `bag-functional`：真实历史 Bag 只读读取、历史 TF 重建、SCAN 地图、B-spline 与 Path 参考输出。
- `planning-acceptance`：Live Stack 目标链路，操作者在 RViz 选择一个清晰目标，检查 Goal Gate 转发与 B-spline，并在退出时停止 N-01 栈。
- `live-preflight` / `planning-audit` / `robot-pose-audit`：现场只读证据采集。
- `g4-research`、Map Fusion 回放：PCD/Bag 自由空间证据与全局路线研究，不作为导航安全验收。

### 6.2 已有历史证据摘要（不是当前运行状态）

- 2026-10-29 真实 Bag 首轮功能验收曾报告 `BAG_FUNCTIONAL_PATH_SMOKE_PASS`：前 180 条过滤点云中 99 条完成历史 map TF 匹配，81 条因缺少扫描时刻 TF 被拒绝；观察到 SCAN 膨胀地图、有效 B-spline 与 Path 参考输出；该次无 `/cmd_vel` publisher 且进程清理通过。它只证明当次离线功能链，不证明实时 tracking 或碰撞安全。
- G1 完整离线验收规格包括 clear、blocked、changed、no_odom 四场景。离线轨迹碰撞采用 B-spline 采样与 SCAN 膨胀体素对比；不等价于连续碰撞检测、整机包络或实机安全。
- G4 的 PCD/Bag 研究曾发现大量 UNKNOWN 栅格与地图冲突；部分路线只是研究候选。PCD 中“没有点”不能证明“没有障碍”，不得自动把未知区当自由空间。
- Hive2 参数档案是历史只读审计：机身碰撞盒不等于整机外廓；轮胎接触几何、控制分配实现、姿态保持、制动距离和急停链仍需现场验证。
- 历史资料中存在 0.30 m 与 0.90 m 两套 TF 语义；当前 Live frame contract 已统一为 0.90 m 机体关系，历史 Bag 重建仍须遵循其当时的时间轴与记录，不可混用。

---

## 7. N-01 FAST-LIO2 建图定位后端

`n01_fast_lio` 是我们维护和部署的正式建图定位后端；算法上游固定为：

- upstream: `hku-mars/FAST_LIO`
- branch: `ROS2`
- commit: `cc3c9e8b1469b33b8072764d6db2348ed6dc597f`
- ikd-Tree submodule: `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`

约束：

- 离线模式与甲方工作流一致：建图保存 → PCD/ICP 重定位 → tracking → 导航。
- 实时模式同时运行 FAST-LIO2 建图、标准定位适配和 SCAN 实时点云导航；
  当前会话原点定义为 `map`，不启用历史 PCD 静态层。
- 两种模式及甲方 SLAM 互斥；实时会话不能冒充已经配准的甲方/历史地图。

- 输入原始 `/livox/lidar`（CustomMsg）和 `/livox/imu`；不把过滤后的 PointCloud2 当作原始逐点时间数据。
- 所有输出在 `/n01/fast_lio/*`；私有地图 frame 为 `n01_fast_lio_map`。
- TF broadcaster 默认禁用，不竞争 vendor `map → base_footprint` 或 Hive2 机体 TF。
- 未选择 FAST-LIO2 时不覆盖 Lite SLAM 的状态、位姿、TF；任何模式都不直接发布控制命令。
- 离线模式通过保存地图 ICP 显式建立 `map` 对齐；实时模式只对本次新会话有效。
- 上游许可需遵循其原始 GPL-2.0 LICENSE；商业分发需另行处理许可兼容或授权。

---

## 8. 目录与代码治理

```text
config/
  n01_static_tf_params.yaml
  reference/                 # 只读、带日期的机器可读审计档案
rviz/                        # RViz 配置
scripts/
  n01                        # 唯一操作命令路由
  lib/                       # ROS 环境与构建安全公共函数
  *.sh / *.py                # 具体工作流实现和诊断工具
src/
  n01_*                      # N-01 ROS 包
  scan_planner/              # 上游 SCAN Planner
.github/workflows/           # CI 与分支政策
```

代码整理原则：

1. ROS 包按功能域独立，包名保留 `n01_*`；上游 SCAN 保留原名。
2. `scripts/n01` 是唯一操作入口；底层脚本保留独立职责，避免将不同验收语义硬合并。
3. 临时诊断代码不入库；正式测试、只读审计、进程所有权/清理和安全门控必须保留。
4. 不随意移动或删除脚本：先检索 tracked/untracked 调用点、launch、CI、桌面快捷方式和用户机器本地引用，再做有测试覆盖的迁移。
5. 运行输出统一放在 `~/lyy/n01_runtime/`；不把 Bag、PCD、日志、录屏、压缩包、build/install/log、缓存提交到仓库。
6. 公共 ROS 环境加载与 colcon 构建保护放在 `scripts/lib/`；新增入口必须挂入 `scripts/n01` 并通过 CI。

---

## 9. 安全红线与未完成项

当前软件固定保持：

```text
actuator_output=DISABLED
execution_authorized=false
safety_authorization=NEVER
```

不发布 `/cmd_vel`、`/cmd_wheel`、`/chassis_controller/rt_command` 或任何轮/腿/执行器命令；不解除 operator hold，不连接底盘控制 SDK。动态障碍候选跟踪只是诊断/可视化，不是经认证的安全传感器，不可单独作为停车或通行依据。

仍需独立验证：

- SLAM `/robot_pose` 的稳定频率、接收年龄和源时间戳年龄；
- PCD 与当前 vendor map 的刚性配准和地面/未知空间建模；
- Hive2 有效轮速/转向分配算法与最终运行参数；
- 真实轮胎接地点、腿轮/外壳多姿态碰撞包络和离地间隙；
- 动态障碍感知完整性、预测误差、控制器跟踪误差；
- 定位丢失、通信丢失、急停硬件链、制动距离和人工接管；
- 连续运行、规划时延、成功率等正式需求指标。

**任何离线 PASS、READY、B-spline 或候选运动学通过都不构成实机运动许可。** 真机移动必须另行完成物理安全评审与授权。

---

## 10. 维护规则

- 本文件为唯一维护中的 Markdown 工程文档；新架构、接口、运行、测试和验收结论更新到对应章节，不再创建平行说明文件。
- `config/reference/*.json|yaml` 是机器可读证据/契约，不是第二套人工操作手册；更新时必须与本文同步。
- 运行结果只记录可复现的时间、commit、输入数据标识、命令、结果和限制；历史结论不得写成当前状态。
- 修改 ROS 接口或坐标系必须同步修改代码、配置、测试和本文件，并在 CI 中验证。
- 合并前检查 `git status`、Bash 语法、Python 测试、colcon 相关测试与 GitHub Actions；现场 build/runtime 仍需在目标机验证。
