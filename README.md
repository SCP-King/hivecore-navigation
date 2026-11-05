# hivecore-navigation

N-01 药厂 DOG 导航避障工程。核心架构原则：**导航算法与定位/SLAM 后端解耦**。

当前部署连接甲方无 GUI 的 HiveCore SLAM3D Lite。Lite 由甲方 CLI 独立管理；本仓库不包含、不构建、不启动旧全量 SLAM3D，也不依赖其内部消息包或 GUI。

所有外部定位输入必须先经过 N-01 Localization Adapter：

- 外部输入：`/loc_status`、`/robot_pose`。
- 内部规范接口：`/n01/localization/status`（DiagnosticArray）、`/n01/localization/pose`（Odometry）。
- 导航桥接、地图融合、动态障碍物和 SCAN 只依赖内部规范接口与标准 ROS 消息，不得直接引用 SLAM 私有消息或字段。
- Lite 状态字段映射必须以目标机真实接口定义为准；未配置或未验证时 fail-closed。

## 快速入口

```bash
bash scripts/n01 help
git status --short --branch
```

## 文档

- [项目总览](docs/N01_PROJECT_GUIDE.md)
- [定位后端解耦接口契约](docs/n01_localization_backend_contract.md)
- [实时导航全栈](docs/n01_live_navigation_full_stack.md)
- [实时 SCAN 集成](docs/n01_live_scan_integration.md)
- [四阶段导航交付](docs/n01_four_stage_navigation_delivery.md)
- [脚本与分支治理](docs/n01_repository_structure_and_script_policy.md)

## 安全边界

当前只输出规划候选、诊断与可视化，不发布底盘速度、轮速或执行器命令。RViz（如启用）仅为 N-01 可视化，不是 SLAM GUI 或 Lite 的运行依赖。

## 分支

日常开发仅在 `dev/n01-current`，稳定发布分支为 `main`。禁止创建其他长期或临时开发分支。
