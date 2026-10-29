# G3：已有药厂 Bag 全量离线功能链与数值 MPC-CBF 原型（未实测版）

日期：2026-10-29。**目标：不做 RViz，优先让已经录制的真实 Bag 贯穿点云、SCAN、动态跟踪、预测、数值约束优化、候选 Path、验收报告整个链。** 本提交仅完成开发与源码检查，尚未在用户 Ubuntu 22.04 / ROS 2 Humble 运行，不得宣称已通过。

## 一、实际执行的数据链

```text
已有药厂 Bag（预计 928 帧 /livox/lidar/geo_filtered）
   │ 白名单只读，不执行 ros2 bag play，不广播原始 /tf 和控制话题
   ↓
历史 map TF + 选定机身静态高度 0.90m（仅离线重建）
   ↓
/cloud(map) + /body_pose(map, 历史 scan stamp)
   ├── SCAN 真实三维膨胀地图 → B-spline → /nav/test/global_path
   └── 聚类 + 静态背景过滤 + 跨帧关联 + 速度估计
          ↓
       /n01/offline/dynamic_tracks (同一历史 scan stamp)
          │                                 /nav/test/global_path
          └────────────┬──────────────────────────┘
                       ↓
           G3HistoricalCoordinator
           * 历史 /body_pose 与 tracks 精确 scan stamp 配对
           * SCAN Path 是保留参考几何，header 不能当历史时刻
           * 如果 Path 晚出现：缓冲的已配对帧明确标记 POST_HOC
           * 机器人不在 Path 走廊内时不伪造优化结果
                       ↓
           有限控制集合 + 束搜索 MPC-CBF 数值原型
           * 简化平面点机器人模型（非真实 Hive2 动力学）
           * 有限时域状态推进 p[k+1]=p[k]+dt*v[k]
           * 速度/加速度约束
           * 有可信*候选*时检查 h[k+1] ≥ (1−α) h[k] 和 h[k+1] ≥ 0
           * 数值候选不代表实体安全、不保证连续时间约束
                       ↓
      /n01/offline/g3_solver_report (JSON)
      /n01/offline/g3_candidate_path (只作离线参考，绝非执行目标)
                       ↓
           G3 数字记录 → observer → 自动打包日志
```

* “可信候选”仅表示通过了当前 G2.2 阈值的运动假设，并无人工真值认证。静态地图障碍已交给 SCAN，不应据此断言 G3 是完整静态/动态碰撞安全验证。

## 二、与 G2 的实质区别

G2 的风险接口只计算 `h_proxy`，**没有优化新的路径**。G3 第一次实际枚举多步速度控制序列、积分预测候选位置、对运动目标逐步检查离散 CBF 约束，并以有限束搜索最小化路径跟踪/速度变化代价，从而输出新的 `candidate_xy_m` 和候选 Path。默认研究参数为 `dt=0.25s`、6 步/1.5s、名义 0.45m/s、限速 0.75m/s、假定加速度 3.2m/s²、CBF α=0.35、假定基础净空 0.55m。

该方法是**有限候选集上的 MPC-CBF 近似搜索**：已经执行数值优化与约束过滤，但没有证明全局最优、连续时间碰撞安全、实际驱动模型正确或真实制动可行。候选 Path 绝不能连接到 Hive2 控制接口。

- `FEASIBLE_CBF_CONSTRAINED_CANDIDATE_UNVERIFIED`：有通过 G2.2 的动态候选且得到满足采样约束的多步轨迹；不表示真实环境安全。
- `FEASIBLE_NOMINAL_ONLY_NO_DYNAMIC_CONSTRAINTS`：真实 Bag 没有当前合格的运动目标；只证明名义轨迹优化成功，**不是动态避障成功**。
- `INFEASIBLE_INITIAL_CBF_PROXY_OVERLAP`、`INFEASIBLE_DISCRETE_CBF_OR_DYNAMICS`：已检查但当前约束下没有找到合适轨迹，不得改写为成功。
- `WAITING_FOR_SCAN_PATH`、`BODY_OUTSIDE_REFERENCE_CORRIDOR`：真实数据不能提供可用路径时停止求解；不移动参考路径、不过度扩大走廊、不制造路径结果。
- `post_hoc_reference_comparison=true`：Path 出现后拿以前真实扫描的位姿和目标状态**事后比较**。这不是历史当时已经计划/控制的证据。

## 三、全量 Bag 输入设置

本阶段把现有真实 Bag 主入口从**只用最先 180 帧**扩大为：最多 2000 帧或读到原始 Bag 末尾（已知这份 Bag 的过滤后点云总量是 928 帧），8Hz 离线回放，观察运行 165s，等待时间上限随之调整。**历史 map TF 找不到的点云仍严格拒绝；绝不通过最近的无关 TF 伪造位置。**

验收脚本还会读取 Bag 的 `bag_inventory.json`，**强制核对 `finished=true`、`input_clouds==inventory 真实总量` 且 `output_clouds+rejected_clouds==真实总量`**，以防只处理一部分录制点云却意外宣称全量通过；部分点云因 TF 缺失被正确拒绝不构成伪造通过。

`ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 N01_HISTORICAL_BAG_ONLY=1` 是既有隔离条件；未经明确批准不会连接真实机器狗运动控制、真实定位或 SDK。

## 四、两套不同性质的验收证据

一键脚本在进入 ROS 2 回放前，先执行已有全部单元测试和新的 `scripts/g3_offline_solver_demo.py`。该确定性场景给定一个**已知的合成运动障碍**，要求原始名义路径的 CBF 代理值负、受约束候选值非负、至少有被拒绝的约束样本、候选轨迹真实不同于名义轨迹；将结果写入 `logs/g3_synthetic_solver_evidence.json`。这是“算法有执行真实约束优化”的正向证据，**不是药厂里真的出现了移动人或车辆**。

随后完整药厂 Bag 进入 ROS 2 全链：新观察器必须看到 `g3_exact_stamp_pairs > 0`、`g3_solver_attempts > 0`、`g3_candidate_paths > 0`。同时旧的 SCAN 规划、G2.3 路径有效投影和无控制输出条件保持严格。如果真实路径不在机器人历史位置的 1.5m 走廊内，本轮将正确失败，不会创造一个虚假的 G3 功能 PASS。

G3 统计明确区分：
- `g3_nominal_only_frames`：完整药厂 Bag 进入了求解器，但当帧没有可信运动约束。
- `g3_dynamic_cbf_candidate_frames`：真实药厂数据中存在经筛选候选，数值求解器实际返回受约束轨迹。**仍需运动真值标注才能评价动态避障能力**。
- `g3_posthoc_solver_frames`：Path 晚到后对历史已精确配对帧做事后求解，不可计入实时导航安全。
- `g3_candidate_paths`：独立只读诊断 Path 输出数量；**不等于执行运动轨迹**。

## 五、Ubuntu 上一次性执行

保证真机底盘运动控制链断开，在隔离环境中运行：

```bash
cd /home/demo/lyy/hivecore-navigation && git -c http.version=HTTP/1.1 pull --ff-only origin main && bash scripts/run_n01_bag_functional.sh
```

本轮时间比以往更长，是因为读取整份 Bag，而不是只读 180 帧。生成 `n01_bag_functional_*.tar.gz` 后直接上传，报告中应包含 `logs/g3_synthetic_solver_evidence.json`、`recorded_bag_report.json`、节点日志和清理证据。不能根据编译成功、topic 出现或一个合成场景 PASS 就声称真实 Bag 全链通过。

## 六、尚未完成的最终要求

- 旧 Bag `/localization_status` 不存在，实时定位 tracking **未通过**；
- 历史 TF `base_footprint→base_link` 同时含 0.30 / 0.90 m，仍是待解决的权威冲突；
- 录制的药厂 Bag 缺乏被标注的运动人员/车辆真值；
- G3 简化二维 point-mass 假设不是 Hive2 轮足实际动力学/全身碰撞包络，不具备连续时间约束认证、实机紧急制动或控制安全联锁；
- 没有真实机器人运动控制话题，`safety_authorization=NEVER`，`actuator_output=DISABLED`；
- 暂不做 RViz、性能优化或真机运动测试。

**完成本轮全链数据流后，再集中把数值模型、碰撞安全判据和真实动态障碍真值验证逐步升级，而不是退回 G2 无限细分。**
