# N-01 工程脚本目录规范

本目录只保留可重复使用的工程入口、运维入口和测试入口。临时诊断脚本不得提交到仓库；运行报告、Bag、PCD、录屏和压缩包统一放在 ~/lyy/n01_runtime/，不进入 Git。

## 项目命名与目录边界

- 产品/工程名称：N-01 Navigation。
- Git 仓库暂时保留 hivecore-navigation，避免破坏现有 clone、remote、桌面快捷方式和部署路径。
- ROS 包统一使用 n01_ 前缀；第三方原生算法包保留上游名称（例如 scan_planner）。
- ROS 节点统一使用 n01_<功能名>；N-01 内部话题统一使用 /n01/<domain>/<name>。
- 外部传感器/定位输入话题保持原样，由 adapter 负责接入；禁止把厂商消息类型直接带入规划核心。
- Shell 入口统一采用 <domain>_<action>.sh，内部公共函数放 scripts/lib/；新增脚本必须登记在本文件。

## 唯一推荐入口

| 场景 | 唯一推荐入口 | 用途 |
|---|---|---|
| 环境检查 | n01_environment_preflight.sh | 主机、ROS、工作区和数据路径只读检查 |
| 规划阻塞诊断 | n01_collect_planning_readiness_audit.sh | 汇总当前 live stack 规划阻塞证据 |
| 位姿发布源专项 | run_n01_robot_pose_source_audit.sh | 检查 /robot_pose publisher endpoints 与聚合时间序列 |
| Live Stack 快速预检 | run_n01_live_stack_preflight.sh | 单独运行只读 ROS 预检节点 |
| 构建/部署规划修复 | deploy_n01_planning_fixes.sh | 构建并按安全规则重启 N-01 live_scan |
| 单次规划冒烟 | run_n01_planning_smoke_test.sh | 已有 live_scan 运行时，只做 READY 门控的规划测试 |

不要为同一个场景再增加第二个包装脚本。需要新功能时优先扩展对应 Python 工具或已有入口。

## 脚本目录清单

### 工程维护
- lib/ros_env.sh：ROS setup 安全加载及环境路径处理公共函数。
- lib/colcon_build_guard.sh：构建隔离与安全检查公共函数。
- n01_environment_preflight.sh：环境只读预检。
- deploy_n01_planning_fixes.sh：N-01 规划相关包构建/部署入口。

### 实机 Live Stack
- n01_collect_planning_readiness_audit.sh：完整规划就绪审计入口。
- run_n01_live_stack_preflight.sh：单独运行实时栈只读预检。
- run_n01_robot_pose_source_audit.sh：位姿发布端点专项审计。
- run_n01_planning_smoke_test.sh：READY 窗口内单次规划冒烟。
- record_n01_demo.sh：演示录制入口。

### 离线验收与历史数据
- run_g1_offline.sh：G1 隔离冒烟。
- run_g1_acceptance.sh：G1 完整离线验收。
- run_n01_bag_functional.sh：历史 Bag 功能验收。
- run_n01_demo_replay.sh：演示回放。
- run_n01_g4_full_chain.sh：G4 全链离线验证。
- run_n01_offline_framework.sh：历史离线框架验证。

### 可选 FAST-LIO2 后端研究
- import_n01_fast_lio2.sh：导入固定版本上游代码。
- build_n01_fast_lio2.sh：构建可选 FAST-LIO2。
- run_n01_fast_lio2_bag.sh：隔离 Bag 回放。

### 脚本自测
- test_colcon_build_guard.sh：构建保护函数自测。
- test_ros_env_compat.sh：ROS 环境加载兼容性自测。

### 子包自有脚本
- src/n01_map_fusion/scripts/ 与 src/n01_static_map/scripts/：对应研究子包的回放/阶段脚本，不作为全仓库通用入口。

## 冗余清理规则

当前先完成入口分类和唯一推荐入口登记。历史离线脚本分别覆盖不同 Bag、G4、演示或隔离域，不因文件名相似就直接删除。下一轮清理必须先检查是否被 launch、README、测试、桌面快捷方式或其他脚本引用；是否与另一脚本执行完全相同的工作；是否有独有安全检查、数据采集或清理语义。只有满足无引用、功能重复、无独有安全行为才删除，并同步删掉对应测试和文档引用。

## 当前约束

- 任何会构建、重启、发布目标或影响设备状态的入口，必须在文件头说明副作用和安全边界。
- 只读诊断必须明确不创建 publisher/client、不调用服务/动作、不写参数、不重启进程。
- 每次脚本变更必须通过 bash -n、静态合同测试及 GitHub Actions；不得把基础语法检查交给现场操作者。
