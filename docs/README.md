# 项目文档

更新：2026-11-25。统一维护核心架构、运行、交接与问题清单，过程性排查草稿统一归档至 `/home/demo/lyy/n01_runtime/archive_20261125/`，不为单次排查重复堆积日期文档。

| 文档 | 内容 |
|---|---|
| [AI 交接索引](AI_HANDOFF_INDEX.md) | 接手阅读顺序、关键运行入口、监控话题与红线警告 |
| [项目介绍](PROJECT_INTRODUCTION.md) | 项目定位、硬件运动学、软件分层、状态机与安全边界 |
| [开发指南](DEVELOPMENT_GUIDE.md) | 构建、启动、核心代码入口、关键参数与调试方法 |
| [问题清单](KNOWN_ISSUES.md) | 当前 P0/P1/P2 问题、验证状态与现场验收优先级 |
| [当前方案总览](current_scheme_summary.md) | 动态地面链路、坐标帧定义、静态与动态配置汇总 |
| [当前约束说明](current_constraints.md) | 任务显示隔离、离线地图点分类、规划与执行约束 |
| [总体架构](architecture.md) | 整体框架、类型化接口、代码职责与实机验收边界 |
| [运行与操作](operations.md) | 启动入口、GUI 所有权、建图定位导航、路线管理与排障 |
| [坐标与底盘模型](coordinates_and_chassis.md) | map/机身/雷达/地面、碰撞包络、四轮运动学与限制 |
| [参数溯源](parameter_provenance.md) | 机械、感知、规划与控制参数的实测或 URDF 来源 |
| [导航与位姿连续性](navigation.md) | 静态参考与实时地图、原生 SCAN 交付、C++ 闭环执行 |
| [验证结果与待办](validation.md) | 已完成隔离/现场测试、性能证据与待执行验收项 |

仓库 [README](../README.md) 保留工程入口、目录、构建和集成接口索引。脚本用法见 [scripts/README](../scripts/README.md)。原始日志、PCD、Bag、采样和图表保存在 `/home/demo/lyy/n01_runtime/`，不混入工程说明。

原33份日期文档已归并：地面/TF/感知/模型审计并入坐标文档，路线方案/进度/暂停记录并入运行和导航文档，规划时延/硬件/优化记录并入导航和验证文档。被后续实现替代的固定腿高、错误帧解释、旧模式限制和性能推算不再作为当前操作依据。清理前原文及SHA256清单保存在 `/home/demo/lyy/n01_runtime/reports/docs_consolidation_20261113/`，可追溯恢复。

工作区上级 `/home/demo/lyy/docs/` 的甲方手册PDF、需求原件、开发运行准则及接口对应表是原始依据，本次未删除或改写。
