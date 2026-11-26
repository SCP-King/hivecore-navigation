# N-01 开发指南

## 1. 环境

主机：

```text
Ubuntu 22.04
ROS 2 Humble
C++17
Python 3.10
colcon
```

工作区：

```text
/home/demo/lyy/hivecore-navigation
```

外部运行环境：

```text
/home/demo/lyy/slam3d_20261118/setup_runtime.sh
```

不要修改或重启厂商 SLAM、Livox、IMU、Hive2 Motion SDK。

## 2. 唯一命令入口

```bash
cd /home/demo/lyy/hivecore-navigation
bash scripts/n01 help
```

常用：

```bash
bash scripts/n01 environment-preflight
bash scripts/n01 planning-audit
bash scripts/n01 live-preflight
bash scripts/n01 navigation start
bash scripts/n01 navigation status
bash scripts/n01 navigation stop
bash scripts/n01 deploy
```

GUI：

```bash
ROS_DOMAIN_ID=55 ROS_LOCALHOST_ONLY=0 bash scripts/n01 gui
```

不要绕过统一入口直接启动厂商 SLAM 或底盘 SDK。

## 3. 构建

只构建相关包：

```bash
source /opt/ros/humble/setup.bash
cd /home/demo/lyy/hivecore-navigation
colcon build --packages-select scan_planner n01_live_navigation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

完整相关：

```bash
colcon build --symlink-install \
  --packages-up-to n01_live_navigation scan_planner n01_static_map \
  --event-handlers console_direct+
```

测试：

```bash
colcon test --packages-select scan_planner n01_live_navigation
colcon test-result --verbose
```

如果 launch_test 报：

```text
Read-only file system: /home/demo/.ros/log
```

用可写 HOME/ROS_LOG_DIR：

```bash
mkdir -p /tmp/n01home/.ros/log
HOME=/tmp/n01home ROS_LOG_DIR=/tmp/n01home/.ros/log \
  colcon test --packages-select scan_planner
```

## 4. 启动与停止

```bash
export ROS_DOMAIN_ID=55
export ROS_LOCALHOST_ONLY=0

bash scripts/n01 navigation start
bash scripts/n01 navigation status
bash scripts/n01 navigation stop
```

PCD 必须与当前 map 一致。命令行启动若默认 PCD 不存在，显式指定：

```bash
export N01_NAV_PCD=/home/demo/lyy/slam_data/maps/mapping_lite_20261123_140413_869.pcd
bash scripts/n01 navigation start
```

GUI 会传当前选中的 PCD，通常不需要手动指定。

## 5. 关键代码入口

### controller

```text
src/scan_planner/plan_manage/src/navigation_controller.cpp
```

关键函数/位置：

```text
tick()                     20 ms 控制主循环
receive()                  NavigationTrajectory 接收
fresh()                    反馈/场景/包新鲜度门控
model_limited()            4WS 原生模型限制
clear()                    单姿态 footprint 碰撞
sweep()                    旋转扫掠检查
history_reverse_safe()     历史轨迹倒车证据
```

当前状态机：

```text
ALIGN / ROTATING / SETTLING
FINAL_APPROACH / MOVING
REVERSING
SAFETY_HOLD
WAITING_FOR_REPLAN / FEEDBACK / STEERING
```

### planner FSM

```text
src/scan_planner/plan_manage/src/scan_replan_fsm.cpp
```

关键：

```text
execFSMCallback()
execution progress / target retirement
TERMINAL_HOLD
recovery ladder
auto resume cap
```

### local planner

```text
src/scan_planner/plan_manage/src/planner_manager.cpp
```

关键：

```text
globalAStar2D()            全局搜索 + clearance 代价
smoothGlobalPath()         折线圆角 + 曲率检查
checkDynamicFeasibility()  局部速度/加速度/yaw/曲率
```

### 轨迹审核与交付

```text
src/n01_live_navigation/n01_live_navigation/trajectory_monitor.py
```

负责：

```text
/grid_map/occupancy
→ 场景审核
→ Hive2 原生模型审查
→ NavigationTrajectory
→ hold packet / final handoff
```

### 一致性监控

```text
src/n01_live_navigation/n01_live_navigation/perception_consistency.py
```

输出：

```text
/n01/perception/consistency
```

字段：

```text
raw_near
raw_matched
raw_unmatched
layers
reason
```

它只读，不控制运动。

### GUI 状态翻译

```text
apps/n01_slam_gui/navigation_flow.py
apps/n01_slam_gui/motion_feedback.py
```

新增英文状态/原因时，必须同步中文映射，否则 GUI 显示“未登记原因”。

## 6. 关键参数

主配置：

```text
config/navigation.yaml
```

启动参数：

```text
src/n01_live_navigation/launch/live_scan.launch.py
```

当前重要值：

```yaml
max_speed_mps: 0.4
max_yaw_rate_radps: 0.45
final_approach_radius_m: 0.50
final_approach_stop_m: 0.12
final_approach_speed_mps: 0.08
watchdog.feedback_timeout_s: 3.0
watchdog.replan_timeout_s: 3.0
watchdog.steering_timeout_s: 2.0
watchdog.align_timeout_s: 5.0
recovery.reverse_distance_m: 1.50
recovery.reverse_speed_mps: 0.12
recovery.reverse_timeout_s: 15.0
recovery.reverse_use_trajectory_history: true
recovery.reverse_history_age_s: 10.0
recovery.wait_timeout_s: 5.0
recovery.max_auto_resumes: 3
```

参数不能凭感觉改。先看：

```text
docs/parameter_provenance.md
docs/coordinates_and_chassis.md
```

尤其是：

```text
min_turn_radius = v / max_yaw_rate
```

## 7. 状态与安全约定

```text
safety_state:
  NORMAL / BRAKE / SAFETY_HOLD / ESTOP / MANUAL_OVERRIDE

planning_state:
  IDLE / LOCAL / TERMINAL_HOLD / BLOCKED / PLAN_HOLD

motion_state:
  STOP / PIVOT / STRAIGHT / CURVE / FINAL_APPROACH / REVERSE
```

原则：

```text
安全状态只决定是否允许输出速度
规划状态只决定路径/轨迹阶段
运动状态只决定运动意图
```

不能把“安全等待”塞进规划成功/失败判断里。

## 8. 调试话题

```bash
ros2 topic echo --once --full-length /n01/navigation/execution_status
ros2 topic echo --once --full-length /planning/status
ros2 topic echo --once --full-length /n01/navigation/progress
ros2 topic echo --once --full-length /n01/live_navigation/system_health
ros2 topic echo --once --full-length /n01/perception/consistency
ros2 topic echo --once /n01/perception/consistency
```

日志：

```text
/home/demo/lyy/n01_runtime/logs/
/home/demo/lyy/n01_runtime/local_attempts/*.jsonl
/home/demo/lyy/n01_runtime/run_cmd/*.csv
```

局部统计：

```bash
python3 scripts/local_attempt_stats.py
python3 scripts/plot_local_debug.py
```

## 9. 常见坑

### 1. state_ 不能每 tick 重置

历史 bug：

```cpp
state_ = finished_ ? state_ : "DISABLED";
```

会让 SETTLING/WAITING 分支不可达，所有状态计时被重置。

当前正确：

```cpp
if (!enabled_ && !finished_) state_ = "DISABLED";
```

### 2. 参数校验和配置必须同步

改 `recovery.reverse_distance_m` 后必须同步初始化校验范围，否则启动报：

```text
invalid reverse recovery parameters
```

### 3. 新参数必须先声明

不要直接：

```cpp
node_->get_parameter("recovery.max_auto_resumes")
```

否则参数未声明会 FATAL。使用：

```cpp
load_parameter<int>(node_, "recovery.max_auto_resumes", 3)
```

### 4. 不要把 PGM 改成全高度 3D 障碍

会压死窄通道，导致局部规划失败。

### 5. 不要用“没有障碍点”证明后方安全

后方盲区必须先有 observed_free 或短期历史轨迹证据；实时障碍点始终优先否决。

### 6. 轨迹包无效时不要继续运动

controller 的 `latest_packet_valid_` 必须 fail-closed。

### 7. 不要直接 pkill 全局 ROS 进程

只通过 `scripts/n01 navigation stop` 管理 N-01 自有进程；厂商 SLAM、Livox、底盘必须保留。

## 10. 提交规则

```text
不要自动 commit
不要自动 push
不要创建临时分支
只在用户明确要求时提交
```

当前工作树有大量未提交修改，接手前先：

```bash
git status --short --branch
git diff --stat
```

不要把 build/install/log、PCD、Bag、日志、录屏提交。
