# N-01 项目总览与唯一开发流程

## 项目定位

本仓库是乙方 N-01 Navigation 算法与集成工程。甲方 HiveCore SLAM、Hive2 运动栈、Livox 驱动及底盘 SDK 是外部依赖；本仓库负责定位接口适配、感知/地图输入、原生 SCAN 规划、动态障碍诊断、轨迹监控和安全状态审计。不得复制甲方 SLAM 源码，也不得在规划核心中绑定厂商私有消息、服务或共享库。

## 当前软件边界

```text
甲方 HiveCore SLAM / 其他定位后端
          │ 标准化 Adapter
          ▼
/n01/localization/status + /n01/localization/pose
          │
          ▼
n01_live_navigation
  ├─ 时间戳与位姿新鲜度门控
  ├─ 点云 / sensor_pose / body_pose 桥接
  ├─ Goal Gate 与 runtime supervisor
  ├─ 动态障碍诊断、轨迹监控、安全状态
  └─ native SCAN 输入
          │
          ▼
静态 PCD / 动态点云 / native SCAN
          │
          ▼
规划轨迹与诊断（不等于运动授权）
```

FAST-LIO2 是可选隔离研究后端，输出位于 /n01/fast_lio/*，私有坐标系为 n01_fast_lio_map，默认不发布 TF。当前集成不代表它已具备历史地图定位、重定位或与甲方 PCD 的自动坐标对齐。

## ROS 包职责

| 包 | 职责 |
|---|---|
| n01_bringup | N-01 bringup、RViz 与组合启动入口 |
| n01_live_navigation | 实时定位桥接、时间门控、Goal Gate、运行与安全诊断 |
| n01_static_map | 历史 PCD 静态地图输入 |
| n01_map_fusion | 静态/动态地图融合及全局路由研究 |
| n01_fast_lio | 可选隔离 FAST-LIO2 后端及里程计适配 |
| n01_offline_integration | 离线回放、证据采集和集成测试 |
| scan_planner | 上游 SCAN Planner，保留上游包名和接口 |

ROS 包名、节点名、topic、frame、QoS、launch 参数都是集成契约。脚本治理不得顺手重命名这些接口。

## 文档索引

- docs/n01_localization_backend_contract.md：当前唯一有效的 backend-neutral localization status 契约。
- docs/n01_four_stage_navigation_delivery.md：定位时序、静态/动态地图、SCAN、轨迹和安全状态。
- docs/n01_live_scan_integration.md、docs/n01_live_navigation_full_stack.md：实时集成和全栈运行结构。
- docs/n01_planning_readiness_audit.md：只读现场证据采集及阻塞项判定。
- docs/n01_development_environment.md：开发环境基线，以新鲜预检报告为准。
- docs/n01_fast_lio2_integration.md：可选后端隔离边界与限制。
- docs/n01_repository_structure_and_script_policy.md：脚本命名、清理和双分支约束。
- n01_g1/g2/g3/g4 与 Hive2 参数/运动学文档是特定时间、Bag、地图下的历史证据，不等于当前接口规范，也不能替代新验收。

## 旧分支文档冲突处理

旧 feat/n01-interface-decoupling 分支中的 n01_localization_adapter_contract.md 提出 /n01/localization/pose 与 /n01/localization/diagnostics。该提案与当前实现的 /n01/localization/status + /robot_pose 不一致，不作为有效接口复制；其厂商状态隔离在 Adapter、规划核心只依赖标准消息的原则已并入当前定位后端契约。不得同时维护两套 topic 命名。

旧分支的脚本目录重排提供 scripts/lib/ros_env.sh、colcon_build_guard.sh 等公共模块思路；现有可用实现已在唯一迭代线保留。进一步移动脚本前，必须先更新统一入口、所有引用、桌面快捷方式和 CI。

## 测试与安全层级

1. 静态语法与纯 Python 合同测试。
2. Ubuntu 22.04 / ROS 2 Humble 主机编译和包测试。
3. 隔离 ROS domain 的 Bag 回放与离线链路验证。
4. 真实 ROS 图只读检查：话题类型、QoS、时间戳、TF、发布端点和数据新鲜度。
5. 规划目标与轨迹的受控现场验证。
6. 真机运动验收须另行授权和现场安全评审，验证急停、制动、碰撞包络及底盘控制链。

前一级通过不能推导后一级通过。当前安全边界仍为 planning-only：不得发布底盘速度、轮速或执行器命令，也不得把 navigation_ready 解释成 execution_authorized。

## 唯一长期分支策略

远程仓库只允许 main 和 dev/n01-current。所有功能、修复、重构、测试、文档都直接在 dev/n01-current 上完成。严禁创建 feature/fix/refactor、临时、日期或任务分支。发布时仅从 dev/n01-current 向 main 发 PR；合并后继续在原 dev/n01-current 工作。

常用命令：

```bash
bash scripts/n01 help
git status --short --branch
git switch dev/n01-current
git pull --ff-only origin dev/n01-current
git ls-remote --heads origin
```
