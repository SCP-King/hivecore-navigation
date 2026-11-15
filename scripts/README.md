# 启动入口

日常在项目根目录执行：

```bash
bash scripts/n01 gui
```

只启动界面和系统监控，默认地址 `http://localhost:8123`。
自定义端口：`bash scripts/n01 gui 8130`。
实际GUI脚本为 `apps/n01_slam_gui/run_slam_lite_gui.sh`，桌面“GUI-SLAM启动”
也指向它。建图、定位、导航、RViz与行驶由GUI按钮手动触发。
底盘和雷达由桌面“ Hive2底盘启动 ”独立管理，GUI不接管。

## 目录分工

| 位置 | 用途 |
|---|---|
| `n01` | 统一命令路由；`bash scripts/n01 help`查看命令 |
| `runtime/` | GUI内部调用的定位、FAST-LIO2、执行器环境辅助脚本 |
| `n01_navigation.sh`、`n01_stop_live_stack.py` | 导航生命周期与精确清理；兼容既有验收工具 |
| `lib/` | 公共ROS环境、构建保护 |
| `archive/*.disabled` | 历史自动启动/清场入口；已加退出保护 |
| 其余 `n01_*audit/probe/preflight*` | 诊断工具，不是GUI启动入口 |
| `run_*`、`g*`、`*bag*`、`*demo*` | 离线回放、验收、演示；保留不同前置和退出语义 |
| `test_*` | 脚本本身的回归检查 |

## 命令行手动控制

```bash
bash scripts/n01 navigation status
bash scripts/n01 localization           # 需底盘/雷达先运行
bash scripts/n01 fast-lio-backend localization
```

日常不需要直接执行`runtime/`。`motion`仅为内部执行器入口，默认不使能行驶，
请通过GUI行驶按钮操作。命令行导航管理用于明确要管理该仓库导航实例时；
GUI仍检查自己的实例归属。

## 本次迁移

| 旧位置 | 新位置/替代方式 |
|---|---|
| `start_localization_stack.sh` | `runtime/start_localization_stack.sh`；入口 `n01 localization` |
| `start_fast_lio_gui_backend.sh` | `runtime/start_fast_lio_gui_backend.sh`；入口 `n01 fast-lio-backend` |
| `start_slow_motion.sh` | `runtime/start_slow_motion.sh`；入口 `n01 motion` |
| `n01_cold_start.sh` | 归档停用；使用 `n01 gui` 后手动启动模块 |
| `restart_n01_live_navigation_verify.sh` | 归档停用；使用 `n01 navigation` |
| `start_chassis.sh`、`stop_chassis.sh`、`n01_stop_chassis.py` | 归档停用；底盘由桌面入口独立管理 |
| `start_livox_driver.sh`、`start_livox_custom.sh` | 归档停用；不从GUI补起第二个雷达驱动 |

旧硬件自动启动/清场逻辑已不符合当前管理边界，因此不保留自动转发入口。
历史报告中的旧路径为当时证据，当前操作以本页为准。
仍被CI引用的 `rebuild_and_run_n01_planning_smoke_test.sh` 等专用验收入口保留，
不为减少文件数而破坏离线验收、所有权清理或诊断链路。
