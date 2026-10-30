# N-01 G4.13：历史地图坐标对齐 + 静止定位/导航只读联调（2026-10-30）

## 0. 结论 / 运行边界

按本次用户提供的原始 Bag 时间轴审计：
- 原建图会话的动态 `/tf base_footprint→base_link=+0.30m` 有 866 条；甲方运动栈静态 `/tf_static` 的 +0.90m 一条。**静态 TF 一次发布本来可以一直有效**，两条竞争属于需要修复的双作者风险。持续发布 0.30m 与 raw LiDAR 重投影至旧 PCD 的 Z 数据相互支持，**不代表已对 SLAM 内部每一次 TF 查询做源码级证明**。
- 旧地图起点周围候选低层 map Z≈-0.50～-0.53，按历史 0.30m 变换的 raw LiDAR 低层约 -0.61m。可使用 **-0.55m** 作为**仅用于断开硬件的计算链调试假设**；既不是全球地面函数，也不是实际可行驶平面的校准结果。
- 甲方 Hive2 实机必须最终使用 +0.90m 机体运动 URDF，而旧 PCD 仍按旧建图 map frame。在线定位 TF 必须与旧地图重新配准；**不允许将旧 map→bf 数值原封不动与 +0.90m 链相乘**（将造成约 +0.60m LiDAR 偏差）。
- 之前历史 Bag 357 帧 TF 失败是历史 `map→base_footprint` 在扫描时刻缺少有效样本；**不是**本项目 `offline_bridge.yaml` 的 0.90m gate 造成的。该 bridge 目前默认未用于离线 Bag 重建，仍是未来严格实时门控接口。

## 1. 当前代码变更

- `historical_bag_node.py` / `historical_tf.py` 已加入明确的 `MAPPING_SLAM_DYNAMIC_0P30` 历史 Bag 回放模式，且必须先发现原 Bag 中动态 0.30m TF；既有 `LEGACY_HIVE2_0P90` 仍保留为默认以供老基线可复现。
- `bag_pipeline.launch.py` 和 `g4_offline_scan_reference.launch.py` 显式接收 `scan_ground_height_m`、`scan_body_height_m`，**同一个 body_height 同时送至原生 SCAN 和 G4 参考发布器**。这依据 SCAN `scan_replan_fsm.cpp`：mode3 对每个初始参考点做 `wp.z = reference.z + body_height`。
- 纯历史测试 `N01_G4_MAPPING_SMOKE=1` 会显式选择：
  - 历史定位/点云几何：`bf→bl=+0.30m`；
  - 本地研究路线的 `map ground Z=-0.55m`；
  - 原生 SCAN 网格 `grid_map.ground_height=-0.55m`（该项**控制栅格 Z 原点**；不等于整个障碍膨胀均按同一个地面高程推断）；
  - SCAN 参考轨迹 `grid_map.body_height=+0.85m`，因此名义目标 `-0.55+0.85=+0.30m`，与离线历史 0.30m 机体坐标对应；
  - `N01_G4_FLOOR_Z_VERIFIED` 必须不设，`N01_G4_GROUND_Z_MAP_M` 必须不设。该独立 smoke 参数**不会**将候选地面声明为核准地面。
- `n01_static_map/config/static_map.yaml` 的历史 PCD Z 窗口扩至 **[-1.75,+3.35]m**，覆盖本次提供的全局样本范围约 [-1.500,+3.078]m；`static_map_node.py` 新增真实 Z 窗口裁剪和接收/排除计数。此前实现**实际上没有按 Z 窗口过滤**，不是静默过滤了边界点。
- `scripts/n01_live_localization_readonly_audit.py` 在实机静止、甲方启动的 SLAM 环境里只**订阅** `/tf`、`/tf_static`、`/localization_status`、`/livox/lidar/geo_filtered`，并读 ROS 图中的 TF、`/cmd_vel` 发布端信息；绝不创建任何 Publisher、Client、TF broadcaster、运动命令。日志包含所有观察的 mount edge 值和 TF 发布端节点名/endpoint GID、定位 `outer/inner/tf` 字段与接收时时戳新鲜度。消息本身没有 Publisher GID，因此**单条 TF 的准确作者仍不能仅凭 topic publisher 列表完全归因**；该脚本据此永远保留 `unique_mount_tf_publisher_attributed=false`。
- `offline_bridge.yaml` 仍要求未来真实运动 URDF +0.90m，`slam_rsp_disabled_confirmed=false` 不会被脚本自动设为 true，历史回放独立使用 `n01_historical_bag`，避免用同一 Boolean 同时宣称旧地图和实机单 RSP 验证成功。

**重要遗漏项（未被本轮改动掩盖）**：`n01_static_map` 发布 `/grid_map/static_occupancy`，而原生 SCAN `plan_env/src/grid_map.cpp` 目前没有订阅这个话题。因此上述历史 Bag G4→SCAN 功能 smoke 使用**回放的 Livox 点云与研究 G4 路由**，并未验证完整静态 PCD 融合进原生 SCAN 的占用地图。下一阶段需单独开发/验收静态地图桥与更新策略，不能凭 static_map 节点启动宣称已经融合。

## 2. 首先：断开实机运动链的历史 Bag G4→SCAN→G2→G3 功能测试

**执行环境**：同之前第 1 轮一致，必须是真实 Hive2 控制网络与电机完全隔离的 Ubuntu 开发环境；脚本自己采用 `ROS_DOMAIN_ID=88`、`ROS_LOCALHOST_ONLY=1` 和 `N01_HISTORICAL_BAG_ONLY=1`，检查运行前图为空、禁止 `/cmd_vel` 发布者。该流程不会给真实底盘发送命令，但在有真实硬件的同网/同机系统里不能把逻辑隔离当作物理安全证明。

```bash
cd /home/demo/lyy/hivecore-navigation
git -c http.version=HTTP/1.1 pull --ff-only origin main
env -u N01_G4_GROUND_Z_MAP_M -u N01_G4_FLOOR_Z_VERIFIED \
  N01_G4_MAPPING_SMOKE=1 bash scripts/run_n01_g4_full_chain.sh
```

运行会自动构建 `scan_planner/n01_scan_dynacbf_bridge/n01_offline_integration` 并压缩 `n01_bag_functional_*.tar.gz`；运行日志中旧重建轮廓必须为 `MAPPING_SLAM_DYNAMIC_0P30`，SCAN 需要用 `grid_map.ground_height=-0.55`、`grid_map.body_height=0.85`，且 `g4_prepared/manifest.json` 声明 `floor_verified=false`、`safe_for_real_robot=false`。期望有真实先后顺序 `/initial_path→/planning/bspline→/nav/test/global_path→G2→G3`；如果 G4 observer 报 `G4_CHAIN_INCOMPLETE`，这是一个真实失败，需要分析调试，不得伪造链路通过。

这**不是**实时 SLAM tracking 验收，也不是可驱动的安全路线。

## 3. 然后：机器人静止状态的实时 SLAM 定位只读检查

由有权操作的甲方工程师**按原甲方流程启动**运动与定位组件，但保持机器狗静止，电机/车轮仍不接受乙方指令；如果旧 SLAM 还在发布 +0.30m 的重复 RSP，应先在甲方 SLAM 启动参数里禁用其 RSP（已有源码参数 `launch_robot_state_publisher=false`），以 Hive2 +0.90m RSP 为唯一机体 TF 候选。**不要**为了测试而关闭运动栈全部关节 TF 或向它发送新关节命令。

在同一个能正常看到甲方 ROS 图、已经 source ROS2 Humble 和 SLAM 消息 overlay 的终端里运行以下只读 Python 脚本；**不要**将历史 Bag 的 domain 88 / localhost 设置复制到这个真实在线终端：

```bash
cd /home/demo/lyy/hivecore-navigation
python3 scripts/n01_live_localization_readonly_audit.py \
  --seconds 15 --output "$HOME/n01_live_localization_$(date +%Y%m%d_%H%M%S).json"
```

上传生成 JSON。正常情况下应看到：仅 +0.90m 源坐标值（没有动态 +0.30）、`map→base_footprint` 发生、`/livox/lidar/geo_filtered` 为 `lidar_link` 且有真实 timestamp；`control_signal_outer_role=localization`、`control_signal_session_mode=tracking`、`registration_output_tf_published=true` 和有效的新鲜度；`/cmd_vel` 无未授权发布者。任何一项不满足则报告 `BLOCKED_...`，**不做运动**。即便报告观察项满足，仍不证明历史 PCD 与在线 TF 完全配准、单个 TF 的精确发布者或轮式控制可行。

## 4. 真正需要下一轮闭环导航前解决的剩余接口

- 已知 Hive2 +0.90m URDF 与旧地图 SLAM +0.30m TF 混用时约 0.60m 偏移。只有在线**用原 PCD 与真实点云配准**并验证地图一致、tracking 正常后，才可按当时 SLAM 定位重新赋予权威；不可用 `-0.6m` 常数命令偷偷挪地图。
- 现有 G3 为受加速度约束的二维平移必要条件，不是四轮独立驱动/ankle_yaw 真正执行分配。且实机固定腿 pitch、整机碰撞、物理停车、失定位保护均未完成。
- 在甲方授权、独立急停/断电方案、低速工位和检查通过前，下一步可以启动**只读定位+路径可视化**，不能让机器狗自主行走。这些不妨碍先展示整条算法链。
- 对这份已录历史 Bag，缺 TF 的 357 帧是扫描时刻 `map→base_footprint` 历史有效样本不足，不是严格 online bridge 的 0.90m gate 造成的。

**阶段状态**：代码与纯 Python CI 验证通过，不代表本次新 G4.13 历史 Bag/在线静止 SLAM 已执行；需要以上两类实际数据报告才能给出下一步真实运行判断。
