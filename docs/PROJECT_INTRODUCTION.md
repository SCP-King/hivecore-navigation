# N-01 项目介绍

## 1. 项目定位

N-01 是面向药厂四轮独立转向/独立驱动（4WIS/4WS）机器人 Hive2 的导航避障工程。

核心目标：

```text
厂商 SLAM/Livox/底盘保持外部独立
→ N-01 只做定位适配、实时感知、SCAN 规划、轨迹审核、闭环执行和安全监督
```

仓库不是完整机器人发行版，不负责启动厂商 SLAM、Livox 驱动、IMU、Hive2 Motion SDK。它消费这些外部系统的 ROS 接口。

## 2. 硬件与运动学

当前平台：

```text
Hive2 四轮独立转向/独立驱动
wheel_base_x = 0.56 m
wheel_base_y = 0.27 m
delta_y = 0.085 m
yaw_soft_limit_rad = 1.5707963
max_speed = 0.4 m/s（常用任务 0.2 m/s）
max_yaw_rate = 0.45 rad/s
```

四轮原生模型：

```text
ComputeSteerTargets(vx, vy, omega, current_steer)
ComputeDriveFromSteer(...)
```

导航控制器只发布机体 `Twist`：

```text
/cmd_vel.linear.x
/cmd_vel.angular.z
linear.y = 0
```

不使用横移、蟹行或后轮锁止 Ackermann。

当前运动能力：

```text
v = 0.20, wmax = 0.45 -> Rmin ≈ 0.44 m
v = 0.40, wmax = 0.45 -> Rmin ≈ 0.89 m
```

工程内使用 `min_turn_radius = 0.45 m`，与当前低速配置匹配。

## 3. 软件分层

```text
外部系统
  HiveCore SLAM3D Lite / Livox / IMU / Hive2 Motion / RSP
        │
        ▼
N-01 Localization Adapter（唯一厂商定位消息边界）
        │ /n01/localization/status, /n01/localization/pose
        ▼
N-01 Live Navigation
  ├─ 定位/扫描/位姿时间门控
  ├─ 静态 PCD 地图输入
  ├─ 实时点云进入 native SCAN
  ├─ Goal Gate / runtime supervisor
  ├─ 动态障碍候选诊断（不是安全传感器）
  ├─ NavigationTrajectory 原子交付
  └─ GUI 任务和状态显示
        │
        ▼
scan_planner
  ├─ 全局 A* / 折线参考 / 圆角
  ├─ 局部 B 样条优化
  ├─ 曲率/速度/加速度可行性检查
  ├─ TERMINAL_HOLD
  └─ 恢复阶梯
        │
        ▼
n01_navigation_controller
  ├─ ALIGN / ROTATING / SETTLING
  ├─ FINAL_APPROACH / MOVING
  ├─ REVERSING
  ├─ 安全保持/看门狗
  └─ 唯一正常导航 /cmd_vel
        │
        ▼
桌面独立底盘控制器
```

## 4. 关键 ROS 包

| 包 | 职责 |
|---|---|
| `n01_bringup` | 传感器入口、bringup、RViz |
| `n01_perception` | 传感器到地图的感知适配 |
| `n01_scan` | SCAN 输入和在线地图适配 |
| `n01_live_navigation` | 定位 Adapter、实时桥接、目标门、运行监督、轨迹审核、一致性监控 |
| `n01_static_map` | 静态 PCD 地图服务 |
| `n01_map_fusion` | 静态/动态地图融合研究 |
| `n01_offline_integration` | Bag 回放、离线证据、集成测试 |
| `n01_fast_lio` | 可选隔离 FAST-LIO2 后端 |
| `scan_planner` | SCAN 全局/局部规划与统一控制器 |

`apps/n01_slam_gui/` 是 Web 操作台，不参与 colcon 包发现，通过 `bash scripts/n01 gui` 启动。

## 5. 目录

```text
apps/                   Web GUI
config/                 跨模块参数和几何/地图配置
docs/                   架构、运行、坐标、验证和交接文档
rviz/                   统一 RViz 配置
scripts/                唯一命令入口和生命周期脚本
src/                    ROS 2 包
build/ install/ log/    colcon 生成物
```

运行数据：

```text
/home/demo/lyy/n01_runtime/
  logs/
  local_attempts/
  run_cmd/
  reports/
```

原始 PCD、PGM、Bag、录屏和日志不得提交 Git。

## 6. 地图和坐标

唯一机器可读参考：

```text
config/reference/n01_frames_unified_20261109.yaml
```

主要 frame：

| Frame | 角色 |
|---|---|
| `map` | SLAM 地图、PCD 规划坐标系 |
| `base_footprint` | 厂商 SLAM 跟踪帧 |
| `base_link` | 真实机体帧 |
| `lidar_link` | 过滤后 Livox 点云帧 |
| `imu_link` | Livox IMU 帧 |
| `livox_frame` | 原始 Livox 帧 |

注意：

```text
base_footprint 不是地面
base_footprint → base_link ≈ +0.90 m
base_link → lidar_link ≈ (+0.295, 0, 0.146366) m
```

当前 PCD：

```text
/home/demo/lyy/slam_data/maps/mapping_lite_20261123_140413_869.pcd
```

PGM 离线占用图：

```text
/home/demo/lyy/slam_data/occupancy_grid_maps/
```

当前架构：

```text
PCD → SCAN 规划/定位
PGM → GUI 低成本底图/诊断
```

不要把 PGM 直接转成全高度 3D 障碍列。这会在窄通道中把可通行空间压死，历史上有明确失败。

## 7. 状态机

controller 当前状态（18 个）：

```text
DISABLED
WAITING_FOR_FEEDBACK
PAUSED
MANUAL_OVERRIDE
ESTOP
WAITING_FOR_REPLAN
WAITING_FOR_STEERING
SAFETY_HOLD
ALIGN
ROTATING
SETTLING
FINAL_APPROACH
MOVING
REVERSING
VERIFYING_ARRIVAL
ARRIVED
FAILED
GOAL_CHANGED
```

planner FSM（6 个）：

```text
INIT
WAIT_TARGET
GEN_NEW_TRAJ
REPLAN_TRAJ
EXEC_TRAJ
EMERGENCY_STOP
```

额外 planning phase：

```text
TERMINAL_HOLD
PLAN_HOLD
```

状态输出字段：

```text
state
safety_state
planning_state
motion_state
state_elapsed_s
safety_hold_reason
```

## 8. 当前核心行为

```text
目标受理
→ 全局 A* + clearance 代价
→ 全局折线 + 圆角
→ 局部 B 样条
→ 曲率/速度/加速度检查
→ controller 跟踪
→ 最终 0.50 m FINAL_APPROACH
→ 0.12 m 最终 yaw ALIGN
→ SETTLING
→ VERIFYING_ARRIVAL
→ ARRIVED
```

失败时：

```text
局部 detour
→ 全局重规划
→ WAITING
→ 有界自动恢复
→ 安全保持
→ 有历史/观测证据时 REVERSING
```

## 9. 安全边界

```text
规划 READY ≠ 执行授权
GUI 的“开始行驶”才可能使能 controller
controller 是唯一正常导航 /cmd_vel owner
manual/estop/chassis stop 始终优先
```

重要原则：

```text
局部未知不覆盖全局
局部实时冲突覆盖全局
双方都未知才是未知
```

但执行准入仍然 fail-closed：场景/反馈/轨迹包过期时不允许继续运动。

## 10. 当前交接状态

```text
代码有大量未提交修改
最新关键修复：controller state_ 每 tick 被重置为 DISABLED
最新构建：scan_planner PASS
未完成最终实车验收
```

接手者先读：

```text
docs/AI_HANDOFF_INDEX.md
docs/DEVELOPMENT_GUIDE.md
docs/KNOWN_ISSUES.md
```
