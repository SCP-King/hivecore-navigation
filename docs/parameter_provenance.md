# 参数来源与修改纪律（2026-11-21）

## 1. 原则

1. 甲方 vendor 参数不改；
2. 乙方参数必须有来源：
   - 甲方配置；
   - 真实 URDF/机械尺寸；
   - 现场测量/标定；
   - 明确的验收要求；
   - 可复现实验。
3. 不允许用“先改一个试试”“为了让它跑起来”作为参数依据；
4. 参数改动必须同时写：来源、负责人、影响、回退点；
5. 代码适配与参数修改分开。

## 2. 本次错误

为了尽快恢复定位/RViz，我做了两处没有充分依据的改动：

| 改动 | 问题 | 处理 |
|---|---|---|
| `n01_static_tf_params.yaml` 改为甲方 `-0.307199/13.325699` | 甲方值是甲方产品标定；N-01 自己的 5586 帧静置标定是 `-0.838236/10.326584`，不能直接替换 | **已回退到 git 基线** |
| 在 localization launch 增加 `base_link→lidar_link` 静态 TF | 值 0.295 有机械依据，但甲方契约规定该 TF 由运动 RSP 发布；乙方抢发会重复/冲突 | **已移除** |

这两个改动都不符合“设计必须有依据”。

## 3. 当前参数来源

| 参数 | 当前值 | 来源 | 状态 |
|---|---|---|---|
| `lidar_imu.roll/pitch` | `-0.307199 / 13.325699` | 甲方 vendor static_tf_publisher 2026-10-30 live TF | A 方案启用 |
| `reloc_unseated_hunt_enable` | `false` | 甲方默认 | git 基线 |
| `tracking_frame` | `lidar_link` | N-01 适配器真实数据 | 已恢复 |
| `mola_vehicle_frame` | `lidar_link` | N-01 适配器真实数据 | 已恢复 |
| `vehicle_to_scan_xyz` | `[0,0,0]` | N-01 适配器 | 已恢复 |
| `base_footprint→base_link` | 0.90 | N-01 frame contract / motion RSP | A 兜底 |
| `base_link→lidar_link` | `0.295,0,0.146` | 实机 RSP TF（2026-11-21 读取） | 不兜底 |
| mapping tracking frame | base_footprint | 甲方 mapping_lite | A 方案 |
| mapping vehicle frame | base_link | 甲方 mapping_lite | A 方案 |
| mapping tracking_to_vehicle | [0,0,0.90] | N-01 frame contract | A 方案（甲方 .3 与实车 .9 冲突，采用实车契约） |
| `base_link→lidar_link` | 0.295 m | `config/reference/hive2_parameter_inventory` | 应由运动 RSP 发布，当前缺失，未由乙方抢发 |
| `base_footprint→base_link` | 0.90 m | N-01 frame contract / motion RSP | 运行时缺失，未由乙方抢发 |
| `body_to_ground_m` | 0.775 | N-01 当前配置 | 未改 |
| 局部前瞻 | 自动 = 局部地图半宽 −1 m | 用户明确要求“局部地图最远” | 新增，待实机验证 |
| 静态 PCD floor clearance | 0.06 m | 用户明确要求 A 方案 | 新增，待实机验证 |
| 导航/显示/动态障碍地面过滤 | 0.06 m | 用户指定统一标准 | `GROUND_CLEARANCE_M` |
| 局部 unknown 回退全局 2D | true | 用户要求长短期记忆 | `grid_map.local_global_fallback` |
| 静态源 z | −1.2~1.2 | 现有配置 | 未改 |

## 4. TF 所有权

甲方契约：

```text
base_footprint -> base_link   运动 RSP
base_link -> lidar_link       运动 RSP/URDF
lidar_link -> imu_link        甲方 static_tf_publisher
imu_link -> livox_frame       甲方 static_tf_publisher
```

当前运行时：

```text
base_link -> 腿            有
lidar_link -> imu_link      有
imu_link -> livox_frame     有
base_footprint -> base_link 缺
base_link -> lidar_link     缺
```

因此 RViz 不能变换 `lidar_link` 点云，是 **TF 所有权/运行时问题**，不能用乙方临时 static TF 掩盖。

## 5. 后续规则

- 先确认 RSP 是否应该发布缺的两条 TF；
- 如果 RSP 不发，再决定是否由乙方补，并记录唯一发布者；
- 不能直接改甲方 vendor 配置；
- 不能把“甲方产品标定值”直接套到 N-01 实车；
- 每次参数修改必须有本文件对应条目。
