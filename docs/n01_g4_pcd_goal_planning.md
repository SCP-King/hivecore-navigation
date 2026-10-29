# G4：整图 PCD 栅格与指定目标路线——当前实现、SCAN 接入边界

更新：2026-10-30。输入是用户已有药厂地图
`/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd`，
PCD v0.7 / binary / 100121 XYZ 点；XYZ 范围约
X=[-16.85,40.01]、Y=[-13.18,33.05]、Z=[-1.50,3.08] m。
文件是同一次建图过程的 SLAM 导出图，但还没有凭固定环境特征
独立确认与 Bag `map` 完全对齐。**所有示例只在离线环境中运行。**

## 已验证的与尚未验证的能力

- G1-G3 之前的真实 Bag 离线测试（2026-10-29）曾验证 SCAN 4/4 有效 B 样条、G3 61 次数值求解。它使用的是逐帧 Livox map 点云和约 1.5m 近前方测试目标；绝不是整张地图的 A→B 路线。
- G3.2 PCD 显示模块已从用户 2026-10-30 输出读取并发布全部 100121 点，写出 `pcd_static_preview.json`。它只供 RViz 显示，不构建 SCAN 输入地图。
- 2026-10-30 的最新演示报错是 G3.1 `pathlib.Path` 和 ROS `nav_msgs.msg.Path` 命名冲突造成 G3 节点崩溃。代码已更正为 `FilePath` 并加入回归测试。用户 Ubuntu 尚未复验。
- **整图全局导航尚未验证。** 不得宣称已经跑通完整药厂路径。

## G4.0 新增：独立地图投影与全局参考路线预览

纯 Python 模块 `pcd_goal_grid.py` + CLI `scripts/n01_pcd_goal_preview.py`。
输入整张 PCD，以显式 `--floor-z`、高差区间和假定膨胀半径将
3D 点投影成 2D 占据/自由/未知栅格；默认拒绝穿越未知区域。
可以指定 `--start x y --goal x y`，使用**独立 A*** 搜索生成研究用
全图参考折线，输出 `.pgm`、`.yaml`、`.svg` 和 `.json`。

**这不是 SCAN 算法计算的路径。** 它只为后续 SCAN 原生参考路径接口
提供能审计的候选输入；不能拿预览曲线替代 SCAN 验收。
如果地面点缺失或未知区域较多，保守模式可能返回无路径。
可显式用 `--unknown-policy optimistic --i-accept-unverified-unknown`
生成仅供视觉检查的乐观假设路线；这种路线穿越未知区域，
禁止解释为可行、安全路线，更不能下发机器人。

### 操作步骤

1. 先只读统计 PCD 高程分布，确认楼地面所在高度（不能仅用最小 Z 判断）：

```bash
cd /home/demo/lyy/hivecore-navigation
python3 scripts/n01_pcd_goal_preview.py \
  --pcd /home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd
```

2. 经实测/原 SLAM 图定位确认地面高程后生成实验地图，下面 `0.0` 只是**需替换的示例**：

```bash
python3 scripts/n01_pcd_goal_preview.py \
  --pcd /home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd \
  --floor-z 0.0 \
  --output-prefix /home/demo/lyy/n01_runtime/pcd_goal/factory_preview
```

3. 打开 `factory_preview.svg`，根据真实 XY 范围挑选地图上的可达起终点，
然后带参数重新生成：

```bash
python3 scripts/n01_pcd_goal_preview.py \
  --pcd /home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd \
  --floor-z 0.0 \
  --output-prefix /home/demo/lyy/n01_runtime/pcd_goal/route_example \
  --start START_X START_Y --goal GOAL_X GOAL_Y
```

上例的 `START_X` 等是占位符，不是有效数值，不能直接执行。
`--clearance 0.5` 只是研究用示例，并非完成 Hive2 腿/轮/机身碰撞包络校准。
每次运行的 `json` 会明确包含
`global_reference_algorithm=A_STAR_ON_PROJECTED_GRID_NOT_SCAN`、
`scan_local_optimizer_connected=false`、
`actuator_output=DISABLED` 和 `safety_authorization=NEVER`。

## 下一步的 SCAN 原生接入

当前 SCAN 源码 `scan_replan_fsm.cpp` 已支持 `fsm.navi_mode=3`，
从 `initial_path` (`nav_msgs/Path`) 接收参考路线；
局部轨迹优化由 SCAN 自身的 A* Rebound 搜索、B 样条求解输出
`/planning/bspline`，现有桥接器能变成
`/nav/test/global_path` 供 RViz 查看。

接入前必须完成：

1. 核对 PCD 地面/障碍/未知分层、机器人起点与终点均为同一 `map`，
   不允许只靠同名 frame_id 或文件时间相近证明。
2. SCAN 当前 `grid_map.sliding_map_size_x/y=10m` 是滑动局部地图；
   整图 A* 仅负责参考路线，不能声称 SCAN 内部已经做了全局避障。
3. 不能把整张 PCD 当一次虚拟激光扫描直接灌入 `/cloud`：
   当前 `GridMap::cloudCallback` 会按当前 `sensor_pose` 射线更新占据概率，
   这不代表 SLAM 融合地图的测量原点，会制造错误的自由空间。
   需要独立的静态占据地图接入/融合机制，并对规划结果进行静态碰撞复核。
4. `pathCallback` 当前会执行 `wp.z = incoming_z + grid_map.body_height`，
   需要分清地面高程、参考路径高程和机体中心高度，
   不可凭表观正常就做数值复用。
5. 要有真实/明确的起点、目标点、不可通行区和安全包络，
   才能报告“真实整图 SCAN 规划已完成”；目前没有相关验收结果。

**任何输出仍是离线可视化/算法研究，绝不能连接底盘。**
