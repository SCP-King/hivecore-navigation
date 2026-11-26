# N-01 AI 交接索引

给接手开发者的阅读顺序：

1. [项目介绍](PROJECT_INTRODUCTION.md)
2. [开发指南](DEVELOPMENT_GUIDE.md)
3. [问题清单](KNOWN_ISSUES.md)
4. [总体架构](architecture.md)
5. [运行与操作](operations.md)
6. [坐标与底盘](coordinates_and_chassis.md)
7. [导航与位姿](navigation.md)
8. [验证与待办](validation.md)

## 一句话现状

```text
系统能构建、能启动、能规划；
最新修复了 controller 状态每 tick 重置和倒车参数校验；
最后一轮状态机/倒车修改还没有完整实车验收。
```

## 先看这三个运行入口

```bash
cd /home/demo/lyy/hivecore-navigation
bash scripts/n01 help
bash scripts/n01 navigation status
bash scripts/n01 gui
```

## 当前最关键的现场观察

```text
/N01/navigation/execution_status:
  state
  safety_state
  planning_state
  motion_state
  state_elapsed_s
  safety_hold_reason
  reverse_active/attempts

/planning/status:
  phase
  dominant_local_failure
  local_failure_counts
  recovery_stage

/n01/perception/consistency:
  raw_matched/raw_unmatched
```

## 交接警告

```text
工作树有大量未提交修改
不要覆盖、不要自动 commit
不要启动第二套厂商 SLAM
不要绕过 scripts/n01 生命周期
离线 PASS 不等于实车安全
```
