> **2026-11-09 更新：** 本文描述的旧版多标签页（状态与地图 / 操作台 / 数据源 / 可视化 / 节点体检 / 统一生命周期）已合并为单页控制台。新布局、栅格地图、手动建图/定位/导航和统一 TF 以根目录 [README](../README.md) 为准；本文保留为服务语义、定位/建图流程和安全约束的详细参考。
>
> **建图与 bag 已合并为同一生命周期：** “启动建图 + 录包”同时启动 mapping desk 和 `ros2 bag record`；结束建图先 `deactivate_mapping` 保存 PCD，再停 bag、停 desk。保存失败会保留 bag/desk 以便重试；desk 异常退出或 GUI 退出会收尾 bag，避免孤儿录包。

# SLAM3D Lite GUI 用户操作手册

**适用程序：** `n01_slam_gui`  
**默认部署树：** `/opt/slam3d_release`  
**默认 Web 地址：** `http://<机器人IP>:8123`  
**运行方式：** ROS 2 Humble + NiceGUI + rclpy，纯命令行启动 GUI，浏览器操作。

> 本手册按随包 `slam_lite_gui.py` 和 `run_slam_lite_gui.sh` 的实际实现编写。GUI 是观察与显式调用服务的操作台，不是 SLAM 算法本身。GUI 默认不会启动 SLAM，也不会生成 TF、坐标系或 SLAM 话题。

## 1. 安全与操作原则

1. **定位与建图不能同时运行。** `localization_lite_node` 和 `mapping_lite_node` 使用相同的雷达输入，并可能同时发布 `/robot_pose` 与 `map→base_footprint` TF；并行运行会造成权威数据源冲突。
2. 一次只运行一个 SLAM 工作流：建图完成并保存地图后，再停止建图、启动定位。
3. GUI 的“节点管理”默认关闭。优先在独立持久终端启动甲方提供的 launch；只有确认相关 desk 及其 IMU/RSP/cloud_preprocessor 未运行时，才考虑启用 GUI 节点管理。
4. `deactivate_localization` 会停止定位位姿、TF 和地图输出；`deactivate_mapping` 会自动保存全分辨率 PCD 并清空内存。点击前必须确认。
5. `reloc_ok` 是重定位质量结果，不等同于 `session=tracking`。进入 Tracking 前应确认 `reloc_ok=true` 且 fitness 不高于当前配置阈值。GUI 当前显示/提示阈值为 `0.045`；实际部署配置应以正在运行的 Lite 参数为准。
6. GUI 的“初始位姿”只在 Relocalization 会话接受；Tracking 或 Idle 下发布不会产生预期重定位效果。
7. RViz2、Foxglove 是观察工具；它们不替代 SLAM 节点，也不启动 Livox 驱动或 bag 回放。

## 2. 启动前检查

确认甲方 Lite 综合包存在：

```bash
test -f /opt/slam3d_release/setup_runtime.sh && echo "Lite runtime OK"
test -f /opt/slam3d_release/resource/mapping_lite.pcd && echo "default map exists"
```

确认 ROS 2 环境和 Lite 包可解析：

```bash
source /opt/slam3d_release/setup_runtime.sh
ros2 pkg prefix hivecore_slam3d_lite_localization
ros2 pkg prefix hivecore_slam3d_lite_console
```

预期 prefix 指向 `/opt/slam3d_release/install/...`。如果不是，先停止操作，避免混用另一棵 SLAM 安装树。

确认上游输入（需由现场驱动/预处理工作流提供）：

```bash
ros2 topic info /livox/lidar/geo_filtered
ros2 topic hz /livox/lidar/geo_filtered
```

GUI 不负责启动 Livox 驱动、bag 回放或独立上游传感器流程。

## 3. 启动与访问 GUI

在机器人上打开一个**持续保持运行的终端**：

```bash
cd /home/demo/lyy/hivecore-navigation
bash scripts/n01 gui
```

默认监听 `0.0.0.0:8123`。指定端口：

```bash
bash scripts/n01 gui 8090
```

浏览器打开：

```text
http://<机器人IP>:8123
```

本机访问可用 `http://localhost:8123`。远程 SSH 端口转发：

```bash
ssh -L 8123:127.0.0.1:8123 demo@<机器人IP>
```

然后在本地浏览器打开 `http://localhost:8123`。

启动脚本默认 source `/opt/slam3d_release/setup_runtime.sh`，并设置 GUI 所需的 ROS 日志目录。除非明确要测试另一棵 release 树，否则不要设置 `SLAM_ROSK`。如确实需要指定：

```bash
SLAM_ROSK=/path/to/another_release ./run_slam_lite_gui.sh
```

保持启动终端不退出；按 `Ctrl+C` 结束 GUI。GUI 进程退出时，由它启动的 desk/RViz/Bridge 子进程也会受到生命周期管理。

## 4. 页面说明

GUI 有四个页签：

### 4.1 状态与地图

用于观察当前系统，不直接改变 SLAM 状态。

- **定位卡片：** 显示定位会话（Idle / Relocalization / Tracking）、`map_loaded`、`reloc_ok`、`reloc_fitness`、`/robot_pose` 位姿与新鲜度、`map_path`。
- **建图卡片：** 显示 Mapping 会话、累计点云点数和 `save_dir`。
- **上游输入：** 显示 `/livox/lidar/geo_filtered` 最近数据时间。没有上游点云时，定位/建图通常无法正常工作。
- **节点互斥状态：** 显示定位节点和建图节点是否在位；两者同时出现属于冲突，应立即停止其中一个工作流。
- **地图视图：** 选择“定位地图”“建图点云”或“两者”；可选择自动/20m/40m/100m视野并开关轨迹。此视图仅作网页端 2D 俯视展示，点云会下采样用于显示，不代表原始点云被修改。

### 4.2 操作台

提供定位、建图服务按钮、初始位姿发布，以及可选节点管理。

**定位服务：**

| 按钮 | 实际作用 | 使用时机/注意事项 |
|---|---|---|
| `load_map` | 重新读取定位节点的 `map_path` 参数并加载地图；成功后回到 Relocalization | 更换地图或重新加载当前参数指定地图 |
| `activate_localization` | 激活定位，从 Idle 进入 Relocalization | 定位节点已启动且输入/地图准备好后 |
| `deactivate_localization` | 停用定位，不再发布位姿、TF、地图 | 会中断定位输出；GUI 会二次确认 |
| `enter_tracking` | 请求进入 Tracking | 先确认重定位收敛、`reloc_ok=true`，且 fitness 满足当前阈值 |
| `enter_relocalization` | 从 Tracking 回到 Relocalization | 需要重新进行重定位时 |

**建图服务：**

| 按钮 | 实际作用 | 使用时机/注意事项 |
|---|---|---|
| `activate_mapping` | 从零开始激活建图 | 确认定位节点已停止，输入链路正常 |
| `save_map` | 在建图过程中保存当前地图为带时间戳 PCD | 只在 Mapping 会话中使用 |
| `deactivate_mapping` | 结束建图，自动保存全分辨率 PCD，然后清空内存 | 正常结束建图的首选操作；会二次确认 |

**重要：** `deactivate_mapping` 已经自动保存 PCD。结束建图进入 Idle 后，再点击 `save_map` 可能返回 `no FE local map yet`，这是因为建图内存已清空，不代表刚才的自动保存失败。应先检查结束建图时的服务返回和 `save_dir`。

**初始位姿 /initialpose：**

- `x [m]`、`y [m]`：初始位置，单位米。
- `yaw [deg]`：平面航向角，单位度；GUI 内部转换为弧度后发布。
- `frame`：可选 `map` 或 `base_footprint`，必须按界面实际坐标语义谨慎选择；通常给地图中的机器人初值时使用 `map`。
- 点击“发布”后，GUI 将消息发布到 `/initialpose`。
- 仅 Relocalization 会话接受；Tracking/Idle 会忽略。发布后观察 `reloc_ok`、`reloc_fitness` 和会话状态，不要仅凭按钮提示判断重定位成功。

**节点管理（可选，默认关闭）：**

- 必须先勾选“启用节点管理”。
- 选择“定位 desk”或“建图 desk”，再点击“启动”。
- desk 不仅启动 Lite 主节点，还会启动相关 IMU 互补滤波、cloud_preprocessor；定位 desk 默认还会启动 RSP。
- 如果这些辅助节点已由其他终端/系统启动，不要重复启动 desk。
- GUI 会检测定位/建图互斥；外部终端启动的节点也可能被识别。
- “停止”会停止 GUI 管理的 desk 进程并中断其输出。外部终端启动的节点应在原终端按其运行方式停止，不要假设 GUI 能管理所有外部进程。

### 4.3 数据源

显示 GUI 订阅/发布的话题、类型、方向和最近数据时间，以及 ROS 环境状态和服务列表。

主要接口：

| 接口 | 类型 | GUI 方向 | 说明 |
|---|---|---|---|
| `/loc_status` | `hivecore_slam3d_lite_localization/msg/LocStatus` | 订阅 | 定位会话、地图和重定位状态 |
| `/robot_pose` | `nav_msgs/msg/Odometry` | 订阅 | 定位位姿 |
| `/localization_map` | `sensor_msgs/msg/PointCloud2` | 订阅 | 已加载定位地图 |
| `/mapping_status` | `hivecore_slam3d_lite_mapping/msg/MappingStatus` | 订阅 | 建图会话状态 |
| `/cloud_map` | `sensor_msgs/msg/PointCloud2` | 订阅 | 建图累计点云 |
| `/livox/lidar/geo_filtered` | `sensor_msgs/msg/PointCloud2` | 订阅 | SLAM 上游点云输入 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 发布 | 初始位姿，仅重定位接受 |

定位服务均为 `std_srvs/srv/Trigger`：

```text
/localization_lite_node/load_map
/localization_lite_node/activate_localization
/localization_lite_node/deactivate_localization
/localization_lite_node/enter_tracking
/localization_lite_node/enter_relocalization
```

建图服务均为 `std_srvs/srv/Trigger`：

```text
/mapping_lite_node/activate_mapping
/mapping_lite_node/deactivate_mapping
/mapping_lite_node/save_map
```

GUI 对上述话题使用 BEST_EFFORT / VOLATILE 订阅配置以兼容发布端。GUI 只消费甲方数据；不创建 TF、不发布机器人位姿、不替代 N-01 Localization Adapter。

### 4.4 可视化

**网页内置 2D：** 在“状态与地图”页使用，不依赖 X11，远程操作优先使用此视图。

**RViz2 原生窗口：**

1. 在“可视化”页选择“定位视图”或“建图视图”。
2. 点击“启动 RViz2”。
3. 需要时点击“停止”。

RViz2 是 X11 原生窗口，不能嵌入网页。机器人本机桌面、X11 转发或桌面共享必须可用。定位视图应配合定位工作流，建图视图应配合建图工作流。

**Foxglove Bridge：**

1. 点击“启动 Bridge”。
2. 在 Foxglove Studio（网页或桌面版）连接界面输入 GUI 显示的 WebSocket 地址，默认端口 `8765`，例如 `ws://<机器人IP>:8765`。
3. 远程 SSH 场景可转发端口：

   ```bash
   ssh -L 8765:127.0.0.1:8765 demo@<机器人IP>
   ```

   然后连接 `ws://127.0.0.1:8765`。
4. 不再使用时点击“停止”。

RViz2/Foxglove 只观察 ROS 数据，可以与定位或建图并存；它们不会启动 Livox 驱动或 bag 回放。

### 4.5 节点体检（重名检测 / 未托管残留只读检查）

“节点体检”页用 rclpy 图 API 每 2 秒普查一次全图节点（不走 `ros2 node list`，
不依赖 ros2 CLI daemon）：

- **全图节点表**：同一节点名出现 >1 次标“重复”。宿主进程列显示
  `pid（pgid，up 启动时长）`：同 pgid = 一次 launch 起了两份；不同 pgid =
  两次独立启动（通常是残留，较老的那份先停）。
- **关键话题发布者**：`/imu/data`、`/livox/lidar/geo_filtered`、`/tf_static` 等
  出现 >1 个发布者标“多源”，即不止一套 desk 在发数据。
- **未托管残留（只读）**：宿主进程 `ppid=1` 只表示原父进程已退出，不能证明
  该进程由本 GUI 启动。GUI 只列出 PID、PGID、启动时长和命令行，不会自动
  发送信号。请核对来源后回到原启动终端正常 Ctrl+C，或使用对应服务管理器停止。
  GUI 自己启动的 desk / RViz / Foxglove 由独立进程组和看门狗管理，不依赖该扫描。
- **停止节点**：按“查看进程明细 → 停止该节点”操作。只有由当前 GUI 生命周期管理器
  登记的专属进程组才允许停止；外部终端或其他服务管理器启动的进程只展示 PID/PGID，
  不会由 GUI 越权发送信号。停止受管进程时先 SIGINT，等待后必要时才 SIGKILL。
  白名单外节点（例如 N01 导航栈）只展示 pid，本 GUI 不停止。
- **启动 desk 被拦截**：若前置链节点（互补滤波/cloud_preprocessor/static TF/
  RSP）已在图里，或普查结果过期，GUI 会拒绝启动并给出指引，避免再叠一套
  双源数据。若图里的 `robot_state_publisher` 只是地盘 hive2_driver 的
  （z=0.90），勾选“不启动 SLAM 的 RSP”即可放行。

命令行只读诊断：`bash tools/diagnose_dup_nodes.sh`。

## 5. 推荐定位操作流程

以下流程适用于定位节点尚未启动的情况。先确认上游输入和地图文件可用。

1. 启动 GUI，进入“状态与地图”和“数据源”，确认 ROS 已连接、Lite 消息类型可用。
2. 确认 `/livox/lidar/geo_filtered` 有持续数据。
3. 在独立终端启动定位 desk，或在确认辅助节点未运行后使用 GUI 节点管理启动：

   ```bash
   source /opt/slam3d_release/setup_runtime.sh
   ros2 launch hivecore_slam3d_lite_console localization_lite_desk.launch.py
   ```

4. 等待 `localization_lite_node` 出现，状态进入 Relocalization；检查地图加载状态及 `map_path`。
5. 如需切换地图，确认参数 `map_path` 指向正确 PCD，再调用 `load_map`。
6. 根据机器人在 `map` 坐标系中的实际位置和朝向，填写初始位姿并点击“发布”。
7. 等待重定位结果。以 `reloc_ok=true` 且 fitness 满足当前实际配置阈值为通过依据。当前已核对的综合包配置阈值为 `0.045`，如配置发生变化，以实际运行参数为准。
8. 重定位通过后，点击 `enter_tracking`。
9. 确认会话为 Tracking、`/robot_pose` 持续更新、位姿新鲜度正常。
10. 停止定位时调用 `deactivate_localization`，确认下游已知晓位姿/TF/地图输出将停止。

如果系统已经由外部终端启动定位，不要再次启动 desk。GUI 只连接并观察现有节点。

## 6. 推荐建图操作流程

1. 确认定位节点已停止，`localization_lite_node` 不在运行。
2. 确认 Livox 上游点云和所需 IMU/预处理链路已就绪。
3. 启动建图 desk：

   ```bash
   source /opt/slam3d_release/setup_runtime.sh
   ros2 launch hivecore_slam3d_lite_console mapping_lite_desk.launch.py
   ```

   或在确认辅助节点未运行后使用 GUI 节点管理。
4. 在 GUI 检查建图会话为 `mapping`，并观察 `/cloud_map` 点数增长。
5. 完成采集后点击 `deactivate_mapping`。确认后，节点会结束建图、自动保存全分辨率 PCD 并清空内存。
6. 记录服务返回信息及 GUI 显示的 `save_dir`，到该目录核对 PCD 文件是否存在、大小是否合理。
7. 不要在 Idle 状态下把 `save_map` 当作结束建图后的第二次保存；该服务用于 Mapping 过程中保存当前地图。
8. 建图工作流结束后，才能切换到定位工作流并加载生成的 PCD。

## 7. `reloc_ok` 与 fitness 的理解

Lite 状态消息提供 `session`、`map_loaded`、`reloc_ok`、`reloc_fitness`。已知接口说明中，`reloc_ok` 表示最近一次重定位结果是否通过：求解器收敛，并且 fitness 小于配置的 `reloc_fitness_score_threshold`。状态会保持到下一次进入 Relocalization 或停用。

因此：

- `session=tracking` 表示当前会话处于跟踪状态。
- `reloc_ok=true` 表示最近一次重定位通过质量判定。
- `session=tracking` 不会自动把 `reloc_ok` 改成 true。
- `reloc_ok=false` 不应由导航适配器伪造为 true，也不应通过放宽阈值掩盖。
- 单看 fitness 数值不能推断求解器是否收敛；需结合状态字段和甲方实际配置。

## 8. 常见问题

| 现象 | 排查方式 |
|---|---|
| 页面无法打开 | 检查 GUI 启动终端；`ss -ltnp \| grep 8123`；端口占用时换端口启动 |
| 顶部显示 ROS 未连接 | 看页面底部日志；确认通过 `run_slam_lite_gui.sh` 启动且脚本 source 了正确 release |
| Lite 消息类型不可用 | 检查 `/opt/slam3d_release/setup_runtime.sh` 是否存在；确认 package prefix 指向 `/opt/slam3d_release/install` |
| `/loc_status` 无数据 | 检查定位节点是否运行、ROS_DOMAIN_ID 是否一致、节点是否正常启动 |
| 上游输入无数据 | 检查 Livox 驱动、预处理节点和 `/livox/lidar/geo_filtered` |
| 服务不可用/超时 | 查看“数据源”页服务列表；检查对应定位或建图节点是否运行、服务名是否匹配 |
| `/initialpose` 没有效果 | 确认当前是 Relocalization，不是 Tracking/Idle；核对坐标系、x/y/yaw 初值 |
| `reloc_ok=false` | 核对地图、初始位姿、点云输入、外参和实际 fitness 阈值；不要直接绕过状态门控 |
| 地图视图空白 | 检查 `/localization_map` 或 `/cloud_map` 是否发布，以及上游点云是否新鲜 |
| 定位与建图同时显示在位 | 互斥冲突；立即停止其中一个工作流，禁止两者并行 |
| `deactivate_mapping` 后 `save_map` 报无地图 | 结束建图已自动保存并清空内存；检查之前服务返回与 `save_dir`，不要在 Idle 再调用建图中保存服务 |
| RViz2 无法启动 | 检查 DISPLAY/X11；远程无图形会话时改用网页 2D 或 Foxglove |
| Foxglove 无法连接 | 检查 Bridge 是否运行、8765 端口、防火墙和 SSH 转发地址 |
| ROS 日志目录不可写 | 检查 GUI `log/ros` 权限；也可设置可写的 `ROS_LOG_DIR` 后重启 |
| Python/rclpy context 或 worker 线程报错 | 彻底退出旧 GUI/Python 进程，在普通持久终端重新执行启动脚本；不要在同一交互解释器反复运行 |

## 9. 退出与收尾

1. 停止当前不再需要的 RViz2 / Foxglove Bridge。
2. 如果由 GUI 节点管理启动 desk，使用节点管理“停止”并确认；如果从独立终端启动，按该终端的运行管理方式停止。
3. 如正在建图，先正常调用 `deactivate_mapping`，确认自动保存结果后再停止进程。
4. 如正在定位，按现场流程调用 `deactivate_localization`；注意位姿/TF/地图输出会停止。
5. 最后在 GUI 启动终端按 `Ctrl+C` 退出 Web GUI。

## 10. 与 N-01 导航模块的边界

该 GUI 直接观察甲方 Lite 话题并调用甲方 Lite 服务。N-01 导航另由 `hivecore-navigation` 中的 Localization Adapter 对 `/loc_status`、`/robot_pose` 等输入进行接口标准化。GUI 不负责 N-01 适配、不发布 `/n01/localization/*`，也不应通过 GUI 修改导航内部状态。

SLAM 部署树 `/opt/slam3d_release` 与导航仓库应独立升级。不要把甲方 SLAM 动态库、源码或安装目录复制进 `hivecore-navigation`。

## 统一生命周期页面

新增“统一生命周期”页，集中管理 Lite 定位、Lite 建图与 N-01 导航：

1. 定位/建图启动仍走原有 SLAM desk 进程组与节点普查安全检查，两者互斥。
2. 导航启动前要求定位状态为 `tracking`、`map_loaded=true`、`reloc_ok=true`，且位姿年龄小于 `N01_NAV_MAX_POSE_AGE`（默认 2 秒）。
3. 导航启动会先调用 `n01_navigation.sh stop`，再调用 `status`，两步成功后才 `start`。这只处理导航生命周期管理器拥有的 N-01 进程，不会对整个 ROS 图执行 killall。
4. GUI 退出只停止本 GUI 启动的导航栈，以及本 GUI 自己启动的 SLAM desk/RViz/Foxglove。GUI 打开前已运行的导航栈不会因 GUI 退出被停止。
5. 如果启动前发现无法证明归属的残留节点，系统拒绝叠加启动并报告；请从其原始启动入口正常停止。不要对整台机器人执行 `pkill -f ros`。

GUI 位于导航仓库的 `apps/n01_slam_gui/`，默认会从自身位置发现仓库根目录。只有在需要控制另一份导航工作区时，才设置 `N01_NAV_REPO=/path/to/hivecore-navigation`。
