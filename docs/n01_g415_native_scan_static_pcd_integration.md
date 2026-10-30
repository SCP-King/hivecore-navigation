# G4.15：原生 SCAN 读取历史 PCD 静态障碍地图（核心功能开发）

## 新增的“真正数据消费者”

此前 `n01_static_map` 已将建图 PCD 转成 `/grid_map/static_occupancy`，但原生 SCAN `plan_env/src/grid_map.cpp` **根本没有订阅此话题**。因此原本的 G4→SCAN→G2→G3 成功仅是顺序话题联通，尚不包含整份静态 PCD 的地图障碍约束。

本次直接修改原生 SCAN：
- `GridMap::initMap` 增加受参数控制的 `/grid_map/static_occupancy` PointCloud2 订阅（默认关闭，旧模式保持不变）。
- 严格要求静态云 `header.frame_id=map`，且有限点、点数有界；不直接把 TF 高度常量强行写入静态点。
- 将静态占用 `static_occupancy_buffer_` 和三维膨胀 `static_inflate_buffer_` 与当前实时激光射线占用概率 **分离**。实时光线落空不能删除静态历史墙；滚动地图窗口移动后自动从原 PCD 重新投影。
- SCAN 原生 `getOccupancy`、`getInflateOccupancy`、`isKnownOccupied`、`isKnownFree` 和占用可视化同时读取静态层；因此真正参与原生 SCAN 局部碰撞搜索，而不是仅生成单独话题。
- 静态图被加载且当前本地窗口确实有静态占用点后，SCAN 才通过 transient-local `/grid_map/static_overlay_ready=true` 报告成功。
- 隔离的 G4 研究发布器现在可被配置为必须等待该 readiness 才发送 `/initial_path`。独立 G4 观察器核对“静态图先到 → G4 路线 → SCAN → G2/G3”的顺序，并记下标志。

## 当前研发使用方式

**此功能仅在隔离历史 Bag 的 `N01_G4_MAPPING_SMOKE=1` 模式自动打开**，与此前 +0.30m 历史映射和 Z=-0.55m 局部高度假设一致。原 `n01_static_map` 继续从甲方原始 PCD 只读加载，Z 窗口 [-1.75,3.35]m。不是用户下达的实机运动命令。

入口仍为 `scripts/run_n01_g4_full_chain.sh`；已把 `n01_static_map` 加入 colcon 构建，把 `scan_static_pcd_enable=true` 和 PCD 路径传给 SCAN 和静态图供给节点。额外消息 `/grid_map/static_overlay_ready` 只有在 SCAN 自己完成地图处理后才为真。完整链路仍只产生研究 `/initial_path`、`/planning/bspline`、G2/G3 报告；**无 /cmd_vel 或甲方 chassis_controller 输出**。

不要求用户再次重录任何 Bag、重新读 URDF 或逐项重复先前回归。仅待未来**一次**在 Ubuntu ROS 2 Humble 的完整 colcon 编译及原始 Bag 回放，以确认 C++ 编译、PCD 数据尺寸和 ROS 通讯。GitHub CI 为源代码/纯 Python 合约自动回归，**未在 GitHub 上完成 ROS/SCAN C++ 构建，也未运行新的完整 ROS 联通**。

## 新的运行验收核心指标（只看核心）

1. 日志有 `[GridMap] immutable static PCD overlay: N occupied voxels...` 且 N 大于 0。
2. `/grid_map/static_overlay_ready=true` **早于**匹配 G4 `/initial_path`。
3. 后续 SCAN 产生新 B-spline 与 G2/G3 报告，且没有新的运动指令输出。
4. 如果 PCD 碰撞导致原 29 点研究路径无法通过，正确结果是 SCAN 拒绝该路径；**下一项功能开发**应让 G4 全局搜索使用完整 PCD 障碍分布重新绕障，而不是关闭静态障碍图、跳过碰撞。

**注意**：静态 PCD 仍包含地面/天花板/设备混合层，只是作为占用障碍，当前模型尚无完整机体/腿部碰撞、轮式运动学、实际跟踪闭环、人员动态障碍安全认证。 `native_scan_static_pcd_occupancy_loaded_before_reference=true` 仅表明代码在本地窗口加载了 PCD，不是整地图可驾驶性结论。

这是一个实际规划器功能改动，而非再次做数据统计。 
