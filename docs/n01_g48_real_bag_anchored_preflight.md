# G4.8：基于真实 Bag 起点的局部 PCD+Bag 离线前置检查

日期：2026-10-30。**状态：源码已提交，未在用户的 Ubuntu ROS 2 上完成这一新版前置检查的运行验收。**

## 来自用户的已确认前提

1. PCD 与 Bag 是同一次 SLAM 建图、同一 map 坐标系（**用户确认**；不是算法从 PCD 文件头自动证明了坐标配准）。
2. map 坐标系下地面 Z 值**未知**。**不得**把先前 `-1.0` 测试切片、PCD Z 分位数、`base_footprint` 位姿高度，或 Hive2 的 z=0.90m 安装高度冒充已验证地面。
3. 研究起点是**历史 Bag 第一帧有效扫描所对应的时间戳有效定位**，目标是在此起点实际所在的连续自由观测分量中选取。
4. 测试电脑已与 Hive2 运动控制链隔离，代码也持续拒绝发布底盘运动指令。

## G4.8 新增的自动计算

`scripts/n01_g4_bag_start_preflight.py` 只读历史 Bag，通过与现有历史回放**相同的 TF 选取条件**（`map→base_footprint`、原始 Livox 消息采集时刻、历史 TF 最大年龄 1s、frame 必须 `lidar_link`、PointCloud2 格式有效），寻找**第一帧可接收的历史扫描**；不将任一不合格 TF 充作定位，也不会启动 SLAM。

随后用 Bag 中真实起点 XY 投影到与 G4.3 同一栅格，检查起点是不是 Bag 射线观测的 FREE 且没有 PCD 切片占据冲突。如果起点在 UNKNOWN / OCC / 地图外，**立即失败，不自动把真实机器狗位置挪到最近空闲格**。

如果起点有效，使用 4 连通遍历，只在起点所属的可观测连通分量中选择远端研究目标，调用原有 G4.3 A* 生成 XY 研究路径。起终点和路径均不是假造的 Bag 轨迹；依旧**不代表真实可通行性**。

同步读取原始 PCD 在真实 Bag 起点 XY 周围 1m 内的 XYZ 采样，输出表面 Z 分位数作为**候选地面调查证据**，不自动选取单一地面 Z。该步骤无 ROS 节点、无 DDS 连接、无底盘控制。

## 当前应执行的命令：未知地面，仅作只读前置检查

```bash
cd /home/demo/lyy/hivecore-navigation
git -c http.version=HTTP/1.1 pull --ff-only origin main
env -u N01_G4_GROUND_Z_MAP_M -u N01_G4_FLOOR_Z_VERIFIED \
  bash scripts/run_n01_g4_full_chain.sh
```

此时脚本执行 **G4.8 只读检查**，不允许进入 SCAN ROS 启动。成功时输出：

```text
G4.8: BAG_ANCHORED_XY_RESEARCH_ROUTE_READY_Z_UNKNOWN
G4.8 evidence: .../run_*/bag_start_preflight/g48_preflight.json
G4.8 conflict map: .../run_*/bag_start_preflight/g43_evidence.svg
G4.8_XY_READY_Z_UNKNOWN: stopped before SCAN (correct; ground not yet verified).
```

如果真正记录的第一帧扫描起点在当前 G4.3 的未知格（历史数据稀疏时有可能），返回例如 `BAG_START_NOT_OBSERVED_FREE`。这说明现有 PCD/Bag-derived **观测自由空间不能支持从真实起点直接生成研究路线**，不代表 Bag 无法读取，也不能通过“自动挪起点”制造成功。

输出目录：

`/home/demo/lyy/n01_runtime/runs/n01_g4_research/run_*/bag_start_preflight/`

包含 `g48_preflight.json`、`g43_evidence.json`、`g43_evidence.svg`（只在起终点匹配成功时生成 G4.3 研究路线）。请优先提供 `g48_preflight.json` 给开发端分析。

## 如果地面 Z 真正经过确认

只有在取得经审核的地图地面高度，并解决 SCAN 的 `grid_map.body_height=0.4` 与该历史 Bag 采用的 Hive2 `base_link` z=0.90m 的语义/参考系差异后，才考虑以 `N01_G4_GROUND_Z_MAP_M` 和 `N01_G4_FLOOR_Z_VERIFIED=1` 解锁后续**隔离研究模式**。这一分支仍用真实历史起点和连通目标，绝不回退到任意最远点。届时只能验证 ROS 离线话题计算链路，不代表地面支撑、整机净空、真实实时定位和实机运动安全。

先前 `N01_G4_GROUND_Z_MAP_M=-1.0` 的实验命令在不知道地面 Z 时**不应再使用**。

## 不变的安全约束

- PCD+Bag 的座标同源由用户确认；不代表已有完整几何配准数值报告
- FREE 仅表示与 PCD 片段无明显冲突的激光射线可见格；未知区不当作可通行
- 输入数值、地图时戳、历史 TF 不合法即阻断
- 3D 高度未知即不生成 `g45_scan_reference.json`，更不进入 SCAN
- `safety_authorization=NEVER`、`actuator_output=DISABLED`

## 静态回归测试

```bash
python3 -m unittest discover -s src/n01_offline_integration/test \
  -p 'test_bag_anchored_goal.py' -v
bash -n scripts/run_n01_g4_full_chain.sh
```

测试结果要以操作端实际输出为准，不能凭 GitHub 提交记录宣告通过。
