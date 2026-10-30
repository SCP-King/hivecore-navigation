# G4.7 — 局部 PCD + 历史 Bag → SCAN → G2/G3 离线闭环观测

日期：2026-10-30。**这次提交的是可复验的开发与自动诊断，不是用户 Ubuntu/ROS 2 已实测成功。** 本阶段全部执行在隔离历史数据模式，无底盘运动指令，任何研究路线均不获得实机安全许可。

## 已具备的串联入口

```text
PCD 原始 XYZ + G4.0 factory_map.pgm/json + G4.2 bag_ray.pgm/json
  → G4.3 对齐审计，冲突转 UNKNOWN，研究用 A* 路线
  → G4.5 未验证参考路径 JSON + G4.4 真实 PCD 采样高度/障碍审计
  → 已有 G3 全量历史 Bag 读取（白名单，不播 TF/控制）
  → SCAN navi_mode=3 订阅 /initial_path（必须匹配历史机器人起点）
  → SCAN 原生 /planning/bspline → 桥接 /nav/test/global_path
  → G2 历史点云候选跟踪 → G3 简化有限控制 MPC-CBF
  → G4.7 独立观测 + G3 旧有全量验收 → 打包日志与冲突地图
```

不能把 G4.3 研究路线当作已验证的全局安全路线；SCAN 当前的静态障碍仍只来自历史点云局部地图，完整 PCD 静态占据体尚未融合。G4.7 只能证明**相应话题在同一观察进程中按顺序出现**，不证明 G3 输出确实严格追踪该新参考路线（报告中 `g3_output_proven_to_follow_the_new_g4_route=false`）；更不能证明地面连续、足轮净空、碰撞安全、实时定位或动态避障成功。

## 使用前的真实数据前提

- Ubuntu 22.04 + ROS 2 Humble；**真实 Hive2 控制器、SDK、SLAM 在线控制链物理隔离**。单纯设置 ROS_DOMAIN_ID 不能替代物理隔离与审批。
- 现有源 PCD：`/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd`（100121 个点）；配套历史 Bag `/home/demo/lyy/slam_data/bags/bag_20261027_225100`。
- 已导出的 `factory_map.pgm/json` 与 `bag_ray_20261030_103843.pgm/json` 必须位于同一个 XY 栅格，默认目录为 `/home/demo/lyy/n01_runtime/pcd_goal`。
- 必须明确设置 **测试假设** `N01_G4_GROUND_Z_MAP_M`。例如 `-1.0` 是以前的 G4.1 研究候选，**不是经实测确认的地面高度**。没有人工确认不要把它写成真实数值。
- 如需要从历史 Bag 机器人实际起点规划，必须用有效 `map` 坐标显式提供四项 `N01_G4_START_X`、`N01_G4_START_Y`、`N01_G4_GOAL_X`、`N01_G4_GOAL_Y`。全部不填时选择「自动最远端点研究」；自动端点**很可能与历史 Bag 位姿不一致**，届时安全拒绝是正常结果。

## 集中运行入口（只限隔离离线研究，非当前已验收命令）

```bash
cd /home/demo/lyy/hivecore-navigation
git -c http.version=HTTP/1.1 pull --ff-only origin main
# 已审查的研究地面 Z；-1.0 仅是示例，不代表实测或安全通过
N01_G4_GROUND_Z_MAP_M=-1.0 bash scripts/run_n01_g4_full_chain.sh
```

脚本首先输出 `g43_evidence.json/svg`、`g44_geometry_audit.json`、`g45_scan_reference.json`、`manifest.json`，然后复用原有 `run_n01_bag_functional.sh` 的 Python 单测、ROS 构建、图隔离、命令发布者检查、全量历史 Bag 读取、G3 逐帧审计及进程清理。

新增 `g4_chain_evidence.json` 独立统计：

- 必须严格看到**与参考 JSON 一致**的 `/initial_path` 恰好一次及发布者状态恰好一次；
- **只有在收到该路径后出现的原生 SCAN 有效 B 样条**才可记入 G4；
- 之后还应观测桥接路径、G2 消息、G3 求解报告和 G3 候选路径；
- 出现 `/cmd_vel` 发布者或检查 ROS 图失败则拒绝；
- 若历史位姿和参考起点 XY/Z 不吻合，发布者返回 `WAITING_MATCHING_BAG_START_POSE`，不能为了绕过阻断随意改高度/阈值或伪造位姿；
- 任一段数据缺失时输出 `G4_CHAIN_INCOMPLETE`（不是把旧版 G1–G3 功能成功当成 G4 成功）。

运行生成的 G4 冲突图、几何审计、参考路线及 G4 链路报告都会并入最终 `n01_bag_functional_*.tar.gz`；每次由原有 process guard 按唯一 token 清理本轮子进程。如果存在真实机器人运动连接或 ROS 图不干净，**不得运行**。

## 未解决的技术约束（后续不应该隐瞒）

1. 部分录制帧缺历史 TF（以前全量 Bag：928 读取、571 转换、357 拒绝）；P0 真实 TF 唯一发布者、SLAM tracking 尚未验收。
2. G4.0 PCD 的 z 截面可能把立体结构误当成地面障碍；G4.3 Bag FREE 仅表示激光高度上的可见区域，不代表机器人可通行。
3. G4.4 对 PCD 只是采样表面审计：未见点不等于空间已知为空；全部静态 PCD 障碍还没有实际进入 SCAN 碰撞查询。
4. 当前历史 Bag 的扫描时序、机器人真实运动轨迹与新 A→B 路线不一定匹配；轨迹级动态风险可能只能 POST-HOC 研究比较。
5. G3 是简化点机器人模型上的离线约束优化，不是完整 Hive2 动力学 MPC-CBF、全身碰撞认证、实时跟踪控制器。
6. 只有在实际 Ubuntu ROS 2 运行上述测试并获得独立 `g4_chain_evidence.json` 以及 G3 原始报告后，才能讨论 G4 **话题数据链路**是否成功；即便通过也不代表安全导航已完成。

## 回归测试

```bash
python3 -m unittest discover -s src/n01_offline_integration/test \
  -p 'test_g4_chain_evidence.py' -v
python3 -m unittest discover -s src/n01_offline_integration/test \
  -p 'test_g4_scan_reference.py' -v
python3 -m unittest discover -s src/n01_offline_integration/test \
  -p 'test_pcd_bag_evidence_audit.py' -v
bash -n scripts/run_n01_g4_full_chain.sh
bash -n scripts/run_n01_bag_functional.sh
```

本次未在用户 Ubuntu 机器、真实 ROS 图及原始 PCD 上运行以上脚本；“GitHub 已提交”不等于“验收通过”。
