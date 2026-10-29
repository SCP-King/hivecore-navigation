# Hive2 / SLAM / SCAN P0 只读 Preflight 规格

**版本记录：** 2026-10-29；根据用户对 `/home/demo/latest/slam3d_release`（2026-10-26 构建版）已安装源码的只读审计形成。**本文件不是已执行的检查报告，也不是允许真机运动的验收凭证。**

**目标：** 把 Hive2 唯一 TF 发布者、SLAM 有效定位状态、Livox 点云扫描时刻坐标转换写成下一次集成时必须满足的失败即阻断条件。**不启动真机、SLAM、底盘或导航，不修改甲方配置**。G1 当前合成测试不覆盖这些真实接口。

## 输入与权威来源

| 项目 | 已知来源与接口 | 采集要求 |
| --- | --- | --- |
| Motion TF | Hive2 运动栈 RSP（`hive2_driver`），`base_footprint → base_link` 选定 z=0.90 m | 只读检查实际发布源和所见值 |
| SLAM mount TF | `mount_tf_publisher_node`，ROS 节点名 `robot_state_publisher`，SLAM URDF 默认 z=0.30 m | **确认启动覆盖 `launch_robot_state_publisher:=false` 已生效** |
| SLAM 定位 TF | `slam3d_node`：`map → base_footprint`，与 `/robot_pose` 同源 | 只在 Mapping Live / Localization Tracking 阶段出现；不能只以 TF 存在判定就绪 |
| 定位状态 | `/localization_status`；`LocalizationStatus.msg` 定义中 `registration_output_tf_published` | 期望 outer=Localization、inner=tracking，布尔值 true；宿主约 200 ms 发布 |
| 实际点云 | `/livox/lidar/geo_filtered`，`sensor_msgs/PointCloud2`，`frame_id=lidar_link` | 点云过滤用过 base_link，但**输出不是 map** |
| 静态链 | Motion RSP：`base_link → lidar_link`；SLAM static_tf_publisher：`lidar_link → imu_link → livox_frame` | 完整检查无缺失、无相互覆盖 |

**接口实现前仍需查验：** `LocalizationStatus.msg` 在当前 ROS 环境中的完整包名、outer/inner 字段实际名字与枚举值、时间戳定义；`mount_tf_publisher_node` 开关的 launch 传参路径。不能凭本文猜测 ROS 消息字段直接编译订阅节点。

## 硬性门控（全部满足才能标记“P0 只读就绪”）

### Gate 1：唯一 RSP 与 TF 边归属

1. 启动配置证据显示 **SLAM 的 mount RSP 已禁用**（显式 `launch_robot_state_publisher:=false`，不依赖默认值）；不由乙方脚本自行调用服务替甲方关闭。
2. Hive2 motion RSP 是唯一授权发布 `base_footprint → base_link`、`base_link → lidar_link` 的来源；对实际 `/tf_static` 和必要的 `/tf` 抽样，检查选定边的数值及重复/冲突。
3. `base_footprint → base_link` 目标 z=0.90 m，旧值 z=0.30 m 不得再作为有效发布出现；校验发布数据与发布源，检查同名 `robot_state_publisher` 风险。
4. **不可只用 `/tf_static` 的总 publisher count=1 判定通过**。TFMessage 的 `TransformStamped` 本身没有“谁发布”的元数据；某发布者可能发多条边，某 ROS 图中也可能有同名节点。需综合启动时的开关、ROS 图的 GID/节点信息和针对目标边的实测数据；无法排除多权威时标为 `INCONCLUSIVE`，禁止通过。

### Gate 2：SLAM Localization Tracking 就绪

1. `/localization_status` 持续可读，状态 **outer=Localization、inner=tracking**，且 `registration_output_tf_published=true`。
2. 状态消息必须足够新鲜、同一时间基准，连续观测而非偶然单帧；**超时阈值与需要的连续样本数待按正式接口确定**，不得把约 200ms 发布周期直接当成安全阈值。
3. `map → base_footprint` 的 TF 与状态、`/robot_pose` 同一定位源，且在**点云对应时间戳**可解析。状态丢失、非 tracking、配准标志 false、时间倒退或 TF 不可用时全部 `NOT_READY`。
4. 即使 `map` TF 仍存在，**也不能**绕过 `/localization_status` 门控。

### Gate 3：带时间戳的 `lidar_link → map` 转换

1. 点云消息类型、实际 `header.frame_id=lidar_link`、有效时间戳、坐标点格式与非空状态均记录。
2. 使用点云**原始采集时刻**求出完整 `map ← lidar_link` 变换，经 `map → base_footprint → base_link → lidar_link` 实际 TF 链核验；绝不使用 `Time(0)` 最新变换来冒充点云时刻。
3. 缺失 `map`、状态非 tracking、TF 变换超时或坐标链不一致时，**不发布任何转换后的 SCAN 点云，不允许放行导航控制**。
4. 当前本仓库 `n01_sensor.yaml` 为 `target_frame: odom`，G1 合成为 `world`，真机 SCAN 拟用 `map`：**这三项不是自动等价的 frame**。应通过一份明确的接口映射/坐标转换适配来统一，不允许仅更改 `frame_id` 字符串。
5. 要记录转换成功/失败、点云时间、TF 时间差/异常与坐标帧证据，不记录完整点云原始数据到 Git 仓库。

### Gate 4：零运动命令与证据清理

- 预检为**只读观察程序**，禁止调用底盘 SDK、禁止发布 `/cmd_vel` 或其他实际运动指令，禁止启动 SCAN 导航控制。
- 必须检查受控 ROS 域中是否出现实际控制输出发布者；有则中止，不将其归咎为“测试程序正常”。
- 预检程序若由乙方脚本启动，结束后只清理由本轮标识产生的进程，独立检查是否有 ROS 节点残留，失败不得标记成功；不能通过 `pkill`/全局 ROS 重启代替。
- 日后执行时必须**一键收集报告和日志 `.tar.gz`**，记录每个 Gate 的 `PASS/FAIL/INCONCLUSIVE/NOT_RUN` 及原因。当前只是规格，不是运行结果。

## 建议的摘要结构（计划，不是运行结果）

```text
preflight: NOT_RUN
tf_authority_single: NOT_RUN
slam_tracking_status_fresh: NOT_RUN
registration_output_tf_published: NOT_RUN
map_to_base_footprint_at_cloud_stamp: NOT_RUN
lidar_link_to_map_at_cloud_stamp: NOT_RUN
cmd_vel_absent: NOT_RUN
owned_process_cleanup: NOT_RUN
```

任何关键状态为 `FAIL`、`INCONCLUSIVE`、`NOT_RUN` 时不得输出“P0 Ready”。这不等价于 G1 算法碰撞验收，更不等价于真机运动许可。

## P1 开口项与 G1 现状

- 整机腿轮/外壳随姿态变化的真实碰撞包络、实际最小离地间隙、正式药厂安全距离、最大角速度、制动约束：**仍待现场/甲方批准**。
- G1 合成 `world` 点云 + z=0.4m pose 没有覆盖 `lidar_link → map`；不能用新的几何测试通过代替实际感知链验证。
- 已有 G1 合成日志报告首轨迹可能延迟（用户审计约 16–26 秒）。根因**未确定**，不能将其归因于某一个配置或用降低安全裕量来掩盖。

**执行时机：** 等甲方和 Hive2 运动栈确认 SLAM mount RSP 关闭的启动方法、核实 `LocalizationStatus.msg` 真实字段，并满足受控只读运行条件后，再设计/运行一键 Preflight；目前无需新增 G1 实验。
