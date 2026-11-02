# N-01 四阶段导航开发交付与调试手册

## 交付边界

本分支把实时定位、静态/动态地图、原生 SCAN 规划、Hive2 候选运动学和安全验收状态统一到四阶段运行框架。它是**规划与验收软件交付**，不是已认证的自主驾驶系统。没有新增 `/cmd_vel`、`/cmd_wheel`、`/chassis_controller/rt_command` 或硬件执行发布者；`execution_authorized=false`、`actuator_output=DISABLED`、`safety_authorization=NEVER` 是固定边界。

## A. 定位、时间同步和目标入口

- `/cloud` 与 `/sensor_pose` 仍使用原始 Livox 扫描时间戳，供 native SCAN 的 ExactTime 同步器使用。
- `/n01/live_navigation/scan_body_pose` 保留每帧扫描时刻的机体位姿。
- `/body_pose` 改为高频当前机体位姿：以新鲜的 `/robot_pose` 全局锚点为基准，使用 `/odom` 中锚点时刻与当前时刻的时间戳插值结果传播；目标发布周期 50 ms。当前位姿传播的 SLAM 锚点年龄单独限制为 0.7 s（且不得大于 `/robot_pose` 最大时间戳年龄），扫描时刻补偿仍严格使用 0.5 s；两者不可混用。若锚点、里程计插值或机体外参不满足对应时间约束，则停止发布，不使用 latest TF 或伪造位姿。
- 最新输入扫描与最近完成输出之间允许最多 450 ms 的有界处理滞后；输出和输入各自仍必须满足 600 ms 最大年龄。超过任一门限时 readiness 关闭。
- `navigation_ready` 仅表示可以向原生规划器提交目标，不表示可以驱动底盘。

## B. 静态地图、动态障碍与原生规划

- 静态 PCD 继续由 `n01_static_map` 以 retained transient-local 点云发布，由 native SCAN 加载到静态占用层。
- 完整 map-frame `/cloud` 仍直接进入 native SCAN，作为动态 raycasting 的权威输入；动态候选跟踪器只提供聚类可视化，不替代或过滤规划器输入。
- 动态候选速度使用点云传感器时间戳计算，不使用 ROS 回调到达时间；无效时间差会把速度标记为无效，不伪造速度。无效坐标系、点云解码失败和静态地图未就绪都会发布明确诊断状态。
- 系统健康分别报告 B 阶段规划入口是否就绪、动态场景诊断是否同时满足接收新鲜度和传感器时间戳新鲜度、规划器轨迹是否新鲜；等待目标导致尚无 B-spline 不会被误报成 A 阶段定位故障。
- RViz 目标经 Goal Gate 后才进入 native SCAN；目标必须使用 `map` frame，且定位/扫描/静态地图 readiness 必须成立。
- `/planning/bspline` 表示规划器候选轨迹；未收到轨迹时属于等待目标或规划器输出，不代表传感器链路故障。

## C. Hive2 候选运动学

- B-spline 继续通过名义四轮转向几何进行必要条件筛查，包括速度、加速度和转向角范围。
- 候选复核还要求新鲜的 `/body_pose`、`/joint_states`，以及 pitch 反馈与当前 home-reference 的匹配。
- 这仍不是执行控制器模型验证。实际轮胎接地点、轮距/轴距、关节映射、固定姿态保持、整机碰撞包络和制动距离必须在实机上测量与验收。

## D. 安全验收状态机

- 新增 `n01_live_safety_supervisor`，订阅系统健康、候选运动学和动态障碍诊断。
- `/n01/live_navigation/operator_hold` 为软件人工保持输入；未收到明确释放消息前保持有效。
- 动态障碍诊断缺失、接收过期、传感器时间戳过期、无效或仍有候选簇时，D 阶段分别报告诊断未就绪或需要人工复核；候选簇不是已验证的障碍物停车信号，也不会被软件监视器解释成硬件急停。
- C 阶段候选轨迹还必须处于 B-spline 的有效时间域；过期轨迹不能因刚收到一条延迟消息而被误判为当前候选。
- 急停链、通信丢失停车、定位丢失停车、动态障碍停车、人工接管、控制器映射、轮胎接触几何、碰撞包络和制动距离等验收项默认均为 false。
- 安全监视器只发布 `/n01/live_navigation/safety_status`，不连接底盘，不控制急停继电器，也不会授权真实运动。即使所有软件检查通过，仍保持 `execution_authorized=false`。

## 构建与启动

停止旧的 `live_scan.launch.py` 实例（Ctrl+C），保留甲方 SLAM、Livox、IMU、robot_state_publisher 和底盘状态节点运行：

```bash
cd /home/demo/lyy/hivecore-navigation
source /opt/ros/humble/setup.bash
git status --short
# 先确认上一步没有未提交改动；若有输出，先备份/提交后再切分支。
git fetch origin
if git show-ref --verify --quiet refs/heads/feat/n01-four-stage-navigation; then
  git switch feat/n01-four-stage-navigation
  git pull --ff-only origin feat/n01-four-stage-navigation
else
  git switch --track -c feat/n01-four-stage-navigation origin/feat/n01-four-stage-navigation
fi
colcon build --packages-up-to n01_live_navigation scan_planner n01_static_map n01_scan_dynacbf_bridge --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select n01_live_navigation
colcon test-result --verbose
```

启动命令：

```bash
ros2 launch n01_live_navigation live_scan.launch.py \
  pcd_path:=/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd \
  pcd_map_frame_asserted:=true \
  ground_z_map_m:=-0.55 \
  body_height_m:=0.90 \
  rviz:=true
```

`pcd_map_frame_asserted:=true` 仍是操作者对 PCD 与当前 vendor map 坐标系一致的显式断言，不是自动配准结果。

## 一次性采集状态

```bash
ros2 run n01_live_navigation n01_navigation_acceptance

# 只读采集 10 秒：话题频率、时间戳年龄、ExactTime 配对、TF 发布者和四阶段状态
python3 scripts/n01_live_stack_preflight.py --duration 10
```

安全监视器的软件保持默认开启。若只需在静态规划测试中解除软件保持，可发送：

```bash
ros2 topic pub --once /n01/live_navigation/operator_hold std_msgs/msg/Bool "{data: false}"
```

重新保持时发送 `{data: true}`。这只是安全状态监视输入，不会启用或停止底盘硬件，也不代表获得运动授权。

## 调试顺序

1. A 阶段：确认桥接状态 `LIVE_SCAN_INPUTS_READY`、`navigation_ready=true`、`/body_pose` 稳定接近 20 Hz，且 scan lag 不超过 450 ms；同时核对 `body_pose_live_anchor_delta_s` 不超过 700 ms，扫描时刻 `motion_compensation_anchor_delta_s` 不超过 500 ms。
2. B 阶段：确认静态地图 READY、实时 `/cloud` 连续进入 SCAN、RViz 中 occupancy/inflation 与动态候选可视化正确；再发送短距离、位于已知空旷区域的 map-frame 目标。
3. C 阶段：检查 B-spline、候选 Path、四轮转向向量、joint feedback 和 pitch reference；不把 necessary-condition pass 当作可执行许可。
4. D 阶段：确认动态障碍诊断的接收时间与点云源时间戳均持续新鲜；出现候选簇时核对 RViz 标记与实际场景，并确认 D 阶段进入 `DYNAMIC_OBSTACLE_REVIEW_REQUIRED`。随后逐项进行物理安全测试并留存证据。任何急停/失联/定位丢失/障碍物测试都应在机器人被机械支撑、驱动禁止或受控测试场地内执行，按现场安全规程进行。


## 第一轮系统级回归范围

- A：定位桥接、点云/位姿时间戳、机体位姿新鲜度和目标门控。
- B：静态 PCD 就绪、原生 SCAN 输入保持原始 `/cloud`、动态候选聚类、基于传感器时间戳的速度估计、异常输入显式诊断。
- C：B-spline 几何必要条件、轨迹起始时间与有效时间域、`/body_pose` 的 map 坐标系与时间新鲜度、`/joint_states` 的时间戳和接收新鲜度、pitch home-reference 反馈。
- D：系统健康、候选轨迹、动态障碍诊断、人工保持和物理验收项统一进入 fail-closed 状态评估；任何路径都不能授权执行。
- 自动化单元测试覆盖动态诊断缺失/过期、传感器时间戳无效、候选簇需要复核、无候选簇仍保持不可执行等状态。真实传感器和底盘场景仍需在 N01 设备上运行验收脚本验证。

## 验收声明

代码、单元测试和 topic 状态不能证明实际无碰撞、完整制动距离、控制器映射正确或急停硬件链有效。真实底盘运动必须在独立完成物理安全验收后另行开发和授权。本分支不包含该授权。
