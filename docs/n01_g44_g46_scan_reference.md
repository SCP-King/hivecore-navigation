# G4.4–G4.6: PCD+Bag 实际证据到 SCAN reference-path 的离线接口

2026-10-30 开发版本。用户只有 PCD + 同次采集的 Bag。**先开发，集中验收**；下述代码已提交 GitHub，但未在用户的 ROS 2 Humble 机器上通过 G4.6 端到端验收。任何路线不能发给实体机器狗。

## 实现了哪些明确的软件接口

- G4.3 `scripts/n01_pcd_bag_evidence_audit.py` 已支持对两份真实导出栅格进行配准一致性检查、冲突转 UNKNOWN、保守 A* 研究路径。PCD 高度切片不可作为安全占据真值。
- G4.4 `pcd_route_geometry_audit.py` 对候选参考路径的每个采样点检查实测 PCD 高度候选地面点和机身高度区间的潜在障碍表面。输出冲突数量和“未知地面证据”数量；**没有抽样障碍 ≠ 证明安全净空**。地面高度是人工假设，尚未验收。
- G4.5 `scan_reference_artifact.py` 检查 G4.3 来源、算法、无控制许可、连续坐标、最大 200 路径点、几何长度，输出 `n01.offline.scan_reference.v1` JSON。Z 用 **地面地图高度假设**，而非机体中心高度；SCAN 的 `pathCallback` 会自行加 `grid_map.body_height`（当前 0.4m）。
- 一键离线准备 `scripts/n01_prepare_g4_research.py`：直接复用 G4.2/G4.0 的 PGM+JSON + 原始 PCD，生成四份文件：`g43_evidence.json/.svg`、`g44_geometry_audit.json`、`g45_scan_reference.json`、`manifest.json`。严格不创建 ROS 节点。
- G4.6 `n01_offline_integration.g4_reference_node`：显式隔离研究模式下，等待历史 `/body_pose` 与参考起点 XY（≤0.75m）、Z（≤0.5m，相对参考地面+SCAN body_height=0.4m）一致，同时观察到 SCAN 订阅 `/initial_path` 后**只发布一次**。检测到任何 `/cmd_vel` 发布者立即阻断。仅发布 `/initial_path` 和 `/n01/offline/g4_reference_status`；没有运动命令。
- 新 launch `g4_offline_scan_reference.launch.py` 显式以 `fsm.navi_mode=3` 启动既有历史 Bag→SCAN→G2→G3，而常规 `bag_pipeline.launch.py` 仍默认 mode=1，不改变此前 smoke 行为。

## 当前尚未实现的内容（不可混为已完成）

**整厂安全导航、SCAN 固定 PCD 全图障碍融合、已验证 3D 可通行性、沿新路线的实际物理运动、实机 Hive2 控制，均未完成。**

在 G4.6 里，SCAN 仍使用历史 Bag 原有的 **局部点云滑动地图**，只是研究性地接收了 G4.5 的全局候选参考路线。其生成的 `/planning/bspline` 应以 ROS 启动日志及逐帧报告确认，不能因为 G4.5 文件存在就称为 SCAN 求解通过。

历史 Bag 中的机器人姿态不会自动匹配用户后来选择的 A* 起点。如果 XY/Z 不吻合，G4.6 会保持 `WAITING_MATCHING_BAG_START_POSE` 而不发布参考线；这是正确的 fail-closed。不可为演示效果直接放宽匹配阈值而宣称真实回放。

## 统一验收：先离线文件，再 SCAN

以下命令留待代码开发完成后的**一次集中验收**。不要在隔离不明或物理机器狗可接收 ROS 指令的环境执行 SCAN launch。

### A. 纯 Python 研究性导出

```bash
cd /home/demo/lyy/hivecore-navigation
git -c http.version=HTTP/1.1 pull --ff-only origin main
python3 -m unittest discover -s src/n01_offline_integration/test -p 'test_g4_scan_reference.py' -v
python3 -m unittest discover -s src/n01_offline_integration/test -p 'test_pcd_bag_evidence_audit.py' -v

OUT=/home/demo/lyy/n01_runtime/pcd_goal/g4_pipeline_$(date +%Y%m%d_%H%M%S)
python3 scripts/n01_prepare_g4_research.py \
  --ray-prefix /home/demo/lyy/n01_runtime/pcd_goal/bag_ray_20261030_103843 \
  --pcd-prefix /home/demo/lyy/n01_runtime/pcd_goal/factory_map \
  --pcd /home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd \
  --output-dir "$OUT" \
  --ground-z-assumption -1.0 \
  --research-route
cat "$OUT/manifest.json"
```

`--ground-z-assumption -1.0` **只是来自 G4.1 的测试候选，不是正式地面标定值**。如果地面不符合该平面，G4.4 可能报告大量未知/障碍；这是正确的诊断结果。用于验收前，需确认实际地面与建图 `map` 的关系。

### B. 历史 Bag → SCAN 模式3 联调（只有完成隔离检查后）

```bash
cd /home/demo/lyy/hivecore-navigation
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to n01_offline_integration scan_planner
source install/setup.bash

export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export N01_HISTORICAL_BAG_ONLY=1 N01_G4_RESEARCH_ENABLE=1
# 与此环境并存的活跃机器人控制接口必须为零；不得桥接到实体 Hive2。

ros2 launch n01_offline_integration g4_offline_scan_reference.launch.py \
  bag_path:=/home/demo/lyy/slam_data/bags/bag_20261027_225100 \
  reference_file:="$OUT/g45_scan_reference.json"
```

验收时只接受原生 SCAN 的 `/planning/bspline` 实际消息和对应的离线观测报告，不接受仅生成的参考 JSON 作为“SCAN 已走通”。运行完成使用 Ctrl+C 结束。若 `WAITING_MATCHING_BAG_START_POSE`，则应重新选择**与真实历史 Bag 位姿一致**的研究起点，而不是将历史机器人的行走伪装为新路线。

所有报告明确标记 `safety_authorization=NEVER`、`actuator_output=DISABLED`。下一步仍需将 PCD 静态障碍的高可信分层信息正确融合进 SCAN 局部占据查询，并开展逐点静态碰撞验证。
