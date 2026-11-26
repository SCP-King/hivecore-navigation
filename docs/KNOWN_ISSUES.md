# N-01 当前问题清单（交接版）

更新：2026-11-25

## 0. 先读

```text
这些是当前工程问题，不是已完成验收结论。
最新代码构建通过，但最后一轮状态机/倒车修改尚未做完整实车验收。
工作树有未提交修改，不要直接覆盖。
```

## P0 级问题

### P0-1 controller state 每 tick 被重置（已修复，待实车确认）

历史代码：

```cpp
state_ = finished_ ? state_ : "DISABLED";
```

后果：

```text
SETTLING 分支不可达
WAITING 状态反复重入
settle_enter_s_ 每 20 ms 重置
状态机看起来一直在等待
```

修复：

```cpp
if (!enabled_ && !finished_) state_ = "DISABLED";
```

待确认：

```text
SETTLING 是否能稳定 0.2 s 后退出
SAFETY_HOLD 是否能稳定保持/释放
REVERSING 是否能从 WAITING/SAFETY_HOLD 正确进入
```

### P0-2 后方盲区与倒车安全

当前倒车条件：

```text
observed_free 可用
或最近 10 s / 3 m 历史轨迹
且后方 1.5 m 内 >=4 个历史点
```

不足：

```text
历史轨迹不能证明现在没有人突然进入
没有后方实时传感器时不能做到绝对安全
```

策略：

```text
低速 0.12 m/s
最大 1.5 m
最大 15 s
最多 2 次
实时障碍一票否决
```

需要现场确认：

```text
后方盲区实际范围
历史轨迹倒车是否可用
是否需要后向超声波/雷达
```

### P0-3 障碍源一致性

当前：

```text
raw occupancy 来自 observation_times_/occupancy buffer
live_grid 使用 getLiveObstaclePoints()
reference_grid 使用 static_window_prior_cells_
```

注意：

```text
live_grid 和 reference_grid 是分层显示
不是两套完全独立来源
```

新增只读监控：

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

仍需确认：

```text
raw occupancy 与 live+reference 的 near 阈值
静态层/动态层在机器人附近的覆盖
ego filter 是否把所有自车点去掉
```

历史现象：

```text
前方 0.72 m 有 raw occupancy 点
live_grid 起点附近 free
```

需要先判断：

```text
自车点
静态层点
真实动态障碍
z/ego filter 过滤差异
```

## P1 级问题

### P1-1 等待状态的恢复出口不完整

当前：

```text
WAITING_FOR_REPLAN 3 s -> SAFETY_HOLD
WAITING_FOR_FEEDBACK 3 s -> SAFETY_HOLD
WAITING_FOR_STEERING 2 s -> SAFETY_HOLD
ALIGN/ROTATING 5 s -> SAFETY_HOLD
SAFETY_HOLD 满足条件 -> MOVING
SAFETY_HOLD + 历史证据 -> REVERSING
```

仍不足：

```text
SAFETY_HOLD 在无 scene / 无历史时只能保持
没有统一的人工 resume 话题
没有安全状态到恢复监督器的完整责任链
```

### P1-2 安全/规划/运动仍是输出分层，不是内部独立状态机

当前 `execution_status` 有：

```text
safety_state
planning_state
motion_state
```

但内部仍主要由一个 `state_` 驱动。

目标：

```text
安全状态只控制速度门控
规划状态只描述路径阶段
运动状态只描述运动意图
```

### P1-3 终端保持/最终进近仍依赖 controller 行为

当前：

```text
planner TERMINAL_HOLD
monitor 在 0.5 m 内发 hold packet
controller FINAL_APPROACH/ALIGN/SETTLING
```

风险：

```text
hold packet 失效
final yaw 无法完成
SETTLING 卡住
```

### P1-4 恢复阶梯会反复尝试

当前：

```text
recovery.wait_timeout_s = 5.0
recovery.max_auto_resumes = 3
```

超过后：

```text
PLAN_HOLD / auto_resume_exhausted
```

需要确认：

```text
PLAN_HOLD 后 GUI 是否有明确提示
是否允许操作员重新发目标/继续
```

## P2 级问题

### P2-1 GUI/底盘启动顺序

现象：

```text
先启动 GUI 再启动底盘，通常正常
先启动底盘再启动 GUI，GUI 可能加载不到底盘
```

怀疑：

```text
ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY
DDS discovery
进程环境被 source 覆盖
端口/运行时锁
```

尚未彻底隔离。

### P2-2 原生 IK/FK 与关节反馈

历史：

```text
native_ik_fk_twist_mismatch
```

常见于：

```text
ALIGN
ROTATING
轮角接近软限位
关节反馈不一致
```

需要现场记录：

```text
q / qd
target steer
target drive
feedback twist
yaw_soft_limit
```

### P2-3 全向扫描/旋转观测

历史：

```text
waiting for full-azimuth scan
rotation_observed false
rotation_stamp 过期
```

影响：

```text
ALIGN 等待
最终 yaw 对准
native rotation 拒绝
```

### P2-4 地面平面歧义

历史：

```text
floor_unavailable_or_stale
存在高度不同且支持相近的两个平面
```

影响：

```text
scene / collision / ground cutoff
```

### P2-5 静态地图/PGM/PCD 语义

当前：

```text
PCD -> planner
PGM -> GUI
```

不要把 PGM 黑格转全高度 3D 障碍列；会导致窄通道局部规划失败。

## P3 级问题

```text
日志原因码仍是英文，GUI 中文映射不完整
local_attempts JSONL 需要定期清理
离线测试与现场运行仍有差距
没有完整回归场景库
```

## 当前验证状态

```text
scan_planner build: PASS
n01_live_navigation build: PASS
n01_live_navigation tests: 78/78 PASS（较早一轮）
scan_planner tests: 4/4 PASS（需可写 HOME/ROS_LOG_DIR）
最新 state_/reverse 修改后未重跑完整在线验收
```

## 接手后的第一优先级

```text
1. git status/diff，确认未提交修改范围
2. 重启 navigation，观察 SETTLING 是否退出
3. 测试前方突然障碍：
   WAITING_FOR_REPLAN -> SAFETY_HOLD -> REVERSING
4. 测试倒车 1.5 m 的后方盲区/历史轨迹表现
5. 检查 /n01/perception/consistency
6. 再决定是否加后向传感器或收紧参数
```

## 不允许的做法

```text
不要用 pkill 全局杀 ROS/厂商进程
不要把未知区域直接当 free
不要用静态地图替代实时后方安全
不要为了通过测试放宽曲率/速度限制
不要自动 commit/push
```
