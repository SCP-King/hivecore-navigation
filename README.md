# hivecore-navigation

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![License](https://img.shields.io/badge/License-Apache-2.0-blue)
![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Jetson-orange)

**N-01 动态障碍物避障导航**真机版 —— 把 DynaCBF-Planner 的导航栈（感知 / 跟踪 / 规划 / MPC-D-CBF 控制）"夹"进甲方 **HiveCore** 平台（Hive2 全向底盘 + 3D SLAM + SDK），形成一条可真机运行的导航闭环。

> 本仓库只包含**真机对接**所需的节点与编排 launch，**不含** Gazebo / turtlebot3 / velodyne 等仿真组件。

## 数据闭环

```
甲方 SLAM 定位(map TF + /robot_pose + /localization_status)   甲方 Livox(/livox/lidar/geo_filtered)
        │                                                                    │
        └──────────────┐                                                     │
                       ▼                                                     ▼
              ┌────────────────────────────────────────────────────────────────────┐
              │  local_map (感知) → obs_kf (跟踪)                                    │
              │  planner (栅格→A*→B样条) → bspline_path_sampler → local_planner      │
              │  controller (MPC-D-CBF) → /cmd_vel_nav                               │
              │  gate_node (定位就绪门控) → /cmd_vel  →  甲方底盘(Hive2)               │
              └────────────────────────────────────────────────────────────────────┘
```

关键点：**controller 不直连底盘**。其输出先经 `gate_node`，只有甲方 `LocalizationStatus` 确认稳定（跟踪模式 + 位姿/TF 已发布 + 拟合误差 ≤ 门限）才透传到底盘 `/cmd_vel`，否则零速——防止定位未就绪时误动。

## 目录结构

```
src/
  dynacbf_msgs          自定义消息（椭圆/轨迹/障碍预测等）
  dynacbf_perception    点云地面/背景过滤、DBSCAN 聚类 → 最小包围椭圆
  dynacbf_tracking      门限+匈牙利关联、常速度 Kalman 滤波、未来椭圆预测
  dynacbf_planner       滑动栅格 + 动力学 A* + B 样条优化，重规划
  dynacbf_dcbf          MPC-D-CBF 控制器（local/controller/sampler 三节点）
  dynacbf_bringup       真机总装 launch + 真机覆盖参数 + 真机 RViz 配置
  dynacbf_gate          定位就绪门控节点（/cmd_vel 透传互锁）   ← 真机新增
scripts/install_deps.sh 系统依赖安装（无仿真依赖）
docs/                   现场自测 SOP + 甲方已实现模块说明
```

## 从零构建

首次环境缺包请（由你本人执行，需要 sudo）：
```bash
sudo bash scripts/install_deps.sh
```

依赖 Python 包走镜像源：
```bash
pip3 install --break-system-packages -i https://pypi.tuna.tsinghua.edu.cn/simple casadi
```

构建（约定**不使用** `--symlink-install`）：
```bash
source /opt/ros/humble/setup.bash
source ~/release_slam_3d/install/setup.bash     # 甲方 SLAM overlay：提供消息与定位
export ROS_DOMAIN_ID=55
cd ~/ROBOT/hivecore-navigation
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 启动真机导航

```bash
ros2 launch dynacbf_bringup dynacbf_real.launch.py \
    navi_mode:=1 pose_topic:=/robot_pose rviz:=true
```

- `navi_mode:=1` 等待 RViz 的 `2D Goal Pose` 下发目标；`2` 用预设航点；`3` 用 reference_path。
- `pose_topic` 默认 `/robot_pose`（甲方 SLAM 定位，map 系）；M1 阶段可改 `/odom` 先验证链路。
- `lidar_topic` 默认 `/livox/lidar/geo_filtered`；现场无预处理时改 `/livox/lidar`。

## N-01 对接要点（与甲方平台的三大不匹配的处理）

| 不匹配 | 处理 |
|---|---|
| 差速→全向运动模型 | 全向底盘的 `linear/angular` 接口在 `controller.py` 中以 6 维 Twist 透传，`yaw` 由甲方底盘上层处理；真机验证留到 M3 |
| MPC 求解 0.2–2.2s > 500ms | 已在 `controller.py` 加 `command_timeout(0.15s)` 缺点：求解超时即发零速并降频提示；若现场仍不满足，需降 N 预测步长 |
| 需求限速 1.0 vs Hive2 默认 2.0 | `max_linear_speed=1.0` 强制收敛；甲方底盘参数需同步设置为 ≤ 1.0 |

`controller.py` 的帧系/话题已参数化（`global_frame/base_frame/command_timeout/max_linear_speed`），`local_map` 的点云输入与 TF 帧系已参数化（`lidar_topic/map_frame/lidar_frame`）。

## 真机运行前提（详见 docs/现场自测SOP）

甲方 SLAM 定位成功（map→base_footprint TF 连续）、点云有数据、底盘 /cmd_vel 被吸收、/odom ~50Hz；速度对齐 ≤ 1.0 m/s；需先 source 甲方 overlay。

## 许可与来源

DynaCBF-Planner 的许可与第 / 三人声明沿用 `LICENSE` 与 `NOTICE`；甲方 HiveCore 相关资料（PDF、SDK、SLAM 配置）不在本仓库，按现场项目单独管理。

> 来源、改动与第三方依赖声明见 `NOTICE`，许可证见 `LICENSE`。