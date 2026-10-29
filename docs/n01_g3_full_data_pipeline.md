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

## 2026-10-29 22:40：G3 合成正向检查失败的修复及脏目录保护

**用户终端证据：** 原工作目录的 `git pull --ff-only` 因多个已修改跟踪文件及同名未跟踪 G3 文件会被覆盖而中止。此时原 HEAD 仍可能是旧版，但工作树含有部分 G3 新文件。随后直接运行 `bash scripts/run_n01_bag_functional.sh`，在 `1.5/5 Running G3 positive synthetic moving-obstacle optimization` 阶段报 `TypeError: G3HistoricalCoordinator.set_path() got an unexpected keyword argument 'header_stamp_ns'`；`result=FAIL`、`reason=g3_synthetic_constrained_solver_failed`，进程和 ROS 图清理 CLEAN。

**直接根因：** `g3_pipeline_core.py` 中定义的是 `set_path(..., path_header_ns=0)`，但 `scripts/g3_offline_solver_demo.py` 用 `header_stamp_ns=999` 调用；这是代码参数命名不一致，不代表 G3 数值求解器已经求解失败，也没有开始运行完整 Bag。

**修复：** 正向演示脚本改为 `path_header_ns=999`，配套单元测试也显式用这个关键字调用，阻止同类接口漂移。对 `run_n01_bag_functional.sh` 增加离线 `scripts` / `src/n01_offline_integration` 的 Git 工作树脏文件检查；若发现文件已修改或未跟踪就 FAIL，禁止混合代码产生“成功”报告，**该检查不会修改、重置或删除用户文件**。

**安全拉取建议：** 不对原工作目录执行 `git reset --hard` 或 `git clean -fd`。如果有本地未合并修改，使用新的并列克隆目录获取 `main` 并运行一键脚本；既不覆盖原文件，也能得到干净、可复现的 G3 版本。**本修复尚待用户 ROS 2 环境复验。**

## 2026-10-29 22:52：G3 全量历史 Bag 真实功能验收 PASS

**用户提供原始日志：** `n01_bag_functional_20261029_225201_1049079.tar.gz`；代码 HEAD `1a8a36f`。这是**用户 Ubuntu 22.04 + ROS 2 Humble、真实药厂录制 Bag 的只读离线功能验收**，不是 Hive2 真机避障安全认证。

| 证据 | 日志结果 |
| --- | --- |
| 一键结果 | `result=BAG_FUNCTIONAL_PATH_SMOKE_PASS`，`exit_code=0`，observer `issues=[]` |
| 集成包单测 | **82/82 PASS**；SCAN/桥接相关 9/9；进程守护 1/1 |
| ROS 2 build | **8 packages finished** |
| 全量 Bag 原始读取 | `input_clouds=928`、`finished=true`，与 inventory 总量吻合 |
| map 点云 | **571 成功**，**357 因历史 map TF 不可用被拒绝**；无错误 frame/畸形数据；不得将 TF 缺失改成无依据的插值 |
| SCAN 输出 | 25 条膨胀地图、3 个测试目标、4/4 有效 B 样条、11 条参考 Path |
| G2 真实点云处理 | **571 帧**，其中 92 帧出现未经真值认证的运动候选、单帧最多 4 |
| G3 按历史 stamp 配对 | **569 帧**；未配对的 2 帧没有强行使用最近 pose 冒充对齐 |
| G3 实际数值优化 | **61 帧**触发有限候选时域约束搜索；**51** 帧只得到名义轨迹、**10** 帧得到带未验证动态候选约束的轨迹 |
| G3 Path 输出 | 节点自身累计 `g3_candidate_paths_published=61` 条（以此为准），独立 topic `/n01/offline/g3_candidate_path`；不是执行轨迹 |
| G2.3 路径评估覆盖 | Path 在历史数据流中出现后有 **2 帧**有效几何投影；另有 **59 帧 POST_HOC** 有效投影，其中 10 帧得到事后路径/动态候选配对；LIVE 动态距离配对仍为 0 |
| 合成确定运动障碍 | 独立 `G3_SYNTHETIC_DISCRETE_CBF_SOLVER_PASS`；5,082 次离散约束检查、2,196 次候选拒绝；名义最小 h=-0.60938m²，受约束候选最小 h=+0.05196m²，轨迹发生变化 |
| 只读与收尾 | `/cmd_vel` 发布者 0，`actuator_output=DISABLED`，`safety_authorization=NEVER`，自有进程/ROS 图 `CLEAN` |

**验收结论：** 整份用户已有药厂 Bag 已经被读取并逐帧按有效 TF 接收/拒绝；**可以确认有效真实点云经过 SCAN + G2 动态处理 + G3 数值候选求解并产生实际轨迹输出**。这就是“已有数据走完全部离线功能原型流程”的阶段性达成。**不能表述为全部 928 帧都进入求解器**：仅 571 条具 map 变换、569 条精确配对，其中 61 条具足够的参考路径和状态供数值求解。

**审计事项，不影响此次软件链路 PASS，但不能忽略：**
1. **G3 运动约束真值未知**：10 条动态候选求解来自未经人工标注、未做真实运动一致性实测认证的目标；不能断言成功避开真实人员/叉车或达成真实路径安全。合成数据则明确独立标记。
2. **路径多数不具实时对应**：G2.3 的 POST_HOC 是事后几何对比，不能宣称录制时已有这条轨迹。G3 `posthoc_solver_frames=160` 是**尝试事后比较的输入帧数**，不代表这 160 帧都求解成功；G3 全部实际数值求解仍为 61。
3. **仪表计数差异**：observer 的 `counts.g3_candidate_paths=70` 高于 G3 节点自身的 `g3_candidate_paths_published=61`，观察器同时使用消息回调递增与节点累计报告取最大值，可能重复计算；本次以发布节点自身的 61 条为准。后续可修正观察器计数，不应将 70 用作独立优化成功次数。
4. **SCAN 局部规划曾警告**：日志包含 `Local target progress mismatch: distance=10.129 horizon=7.500 progress_time=0.000` 和局部目标受碰撞限制的诊断；但节点之后生成了 4 条有效 B 样条，未发生关键子进程崩溃。该历史异常不等于完整轨迹避障安全通过。
5. **P0 及动力学限制不变**：原 Bag `/localization_status` 缺失，历史高度出现 0.30/0.90m 冲突；G3 仅平面点质量近似 + 有限控制集束搜索，未核实 Hive2 整机尺寸、轮足动力学、制动能力与连续时间碰撞安全；无底盘输出。

**下一步工作重点**：停止重复 G2 小版本回归，把 G3 的真实输出证据升级为逐帧可审计的 `nominal/candidate/h_min/constraint_checks/path_origin` 结构化报告，并在有真值的动态目标与已确认机器人动力学下增强约束可行性验证。当前仍不急于制作 RViz 或进行性能调优。此次 **已经 PASS 的功能链不必为了文档更新而重新运行**。
