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


## 2026-10-30 G4.1：真实 PCD 地图质量不达标及诊断工具

用户提供的 `factory_map.json/.pgm/.svg`（PCD 原始文件未包含在上传压缩包中）显示：
`width=230`、`height=187`、`resolution=0.25`，
共 43,010 格；当 `floor_z=0.0`、`floor_tolerance=0.12`、
`clearance=0.5` 时，`floor_hits=5005`、`obstacle_hits=31127`，
`free_cells=18`、`inflated_occupied_cells=6810`、`unknown_cells=36182`。
根据上传 PGM 四邻接计算，自由格仅有 13 个互不连通的区域，最大连续块 3 格。
**因此该地图尚不具备可审计的 A→B 路径规划输入，不应直接随便指定坐标或将未知区放行。**
这只能说明采用当前地面切片与膨胀模型的结果失败，不能证明实际厂区无可通行走廊。

新增 `scripts/n01_pcd_floor_diagnose.py`，使用源 PCD 逐高度比较
`free_cells`、`largest_free_component_cells`、`unknown_percent`，
完整输出 `n01.offline.pcd_floor_sweep.v1` JSON。
它**不会自动选地面**，因为机顶、设备和天花板可能产生较大的假自由连通片。
不要使用结果排名直接当作地面认证。命令：

```bash
cd /home/demo/lyy/hivecore-navigation
python3 -m unittest discover -s src/n01_offline_integration/test -p 'test_pcd_floor_diagnostics.py' -v
python3 scripts/n01_pcd_floor_diagnose.py \
  --pcd /home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd \
  --output /home/demo/lyy/n01_runtime/pcd_goal/floor_sweep_$(date +%Y%m%d_%H%M%S).json
```

之前 G4 生成器也会新增最大的四邻接候选自由区统计；
当最大连通片少于 25 栅格时显式输出地图质量警告。这个阈值只用于
**提示高度碎片化**，不是可通行性或安全认证阈值。

**下一开发门槛：** 最可靠是检查 SLAM 是否另存占据地图 / TSDF / ESDF，
或以 Bag 中有效历史位姿和原始激光观测进行有几何一致性的自由空间射线
重建，再与 PCD 静态表面融合。单张 PCD 通常不能证明“没有点=没有障碍”。
建图视图中的已观测区、未知区、地面、可行驶区及 Hive2 多姿态包络
必须分开建模，才能对真实路线做静态碰撞与未知区审计。

## G4.2：仅有 PCD + 同次建图 Bag 时的下一步（2026-10-30）

用户明确说明没有其他 PGM/OctoMap；不再寻找不存在的外部栅格。G4.1 的真实高度扫描已在 Ubuntu 通过 5/5 单元测试，按 0.25m 高度切片：
- 假设 z=0.00m：候选自由 18 格，最大连通 3 格，未知 84.12%，不能规划；
- 假设 z=-1.00m：候选自由 585 格，最大连通 538 格，未知 83.33%，**仅为待检查地面高度候选**；
- 假设 z=2.25～2.75m：局部连续点也较多，但是高处可能是设备顶部/天花板，绝不能由最大连通分数自动选为地面。
这些数据不能证明整厂地图的自由空间，因此不能自动给出已验证的 A→B 路线。

**新增 G4.2 初始离线工具：** `scripts/n01_bag_ray_visibility.py`。
它使用 ROS 2 的 rosbag2_py 只读打开 Bag，先从历史 `/tf` 获取带时间戳的
`map→base_footprint`，再以已选定的**仅用于回放研究**的挂载关系复原
`map←lidar_link`，逐帧对 `/livox/lidar/geo_filtered` 的原始时间戳配对；
只考虑距离有限、近水平的激光射线，用经过栅格的射线统计已观测可见空间，
以射线终点统计命中障碍。默认未知栅格始终未知，命中点优先于自由票数，
输出一份独立 `PGM/YAML/SVG/JSON` 可见性证据图，不启动 ROS 图或底盘。

执行（先小样本，避免一次处理过多射线）：

```bash
cd /home/demo/lyy/hivecore-navigation
git -c http.version=HTTP/1.1 pull --ff-only origin main
python3 -m unittest discover -s src/n01_offline_integration/test -p 'test_bag_ray_visibility.py' -v
set +u
source /opt/ros/humble/setup.bash
python3 scripts/n01_bag_ray_visibility.py \
  --pcd /home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd \
  --bag /home/demo/lyy/slam_data/bags/bag_20261027_225100 \
  --max-frames 180 --point-stride 32 \
  --output-prefix /home/demo/lyy/n01_runtime/pcd_goal/bag_ray_$(date +%Y%m%d_%H%M%S)
```

输出汇报 `cloud_frames_processed`、`rays_accepted`、自由/障碍/未知格、每帧
历史 TF 失败率。**第一版未显式选择 PCD 静态障碍高度时，PCD 只提供 XY
边界，不会擅自把整张 PCD 的高架点/屋顶也视作地面障碍。**
可选参数 `--pcd-obstacle-z-min/--pcd-obstacle-z-max` 必须成对给定，并须
根据地面高度与实际障碍高度解释选值。禁止自动猜测或按最大 free 区选高度。

**特别警告：** 该输出的 `FREE` 只是“某雷达高度水平射线在这个网格通过”的
视觉证据，不能保证地面可行走、矮障碍不存在、机器人全高净空、碰撞/腿轮安全。
还需要按雷达仰俯角与楼地面几何重建 3D 自由体素、与静态 PCD 面融合，
继而建立多高度机器人通行检查；当前绝不能直接送入 SCAN 当成权威局部
占据地图，也不能自动生成实机导航指令。当前该工具是代码提交阶段，尚未在用户
Ubuntu 的真实 Bag 跑过，测试结果要由用户输出验证。

## G4.3：真实 PCD+Bag 已上传栅格上的对齐审计 + A* 研究路线（2026-10-30）

用户上传 `pcd_goal1.zip`，其中有真实 `bag_ray_20261030_103843.pgm/.json`、
`factory_map.pgm/.json` 和 floor sweep。已从上传的两份栅格计算：

- G4.2 Bag 180 帧中，99 帧匹配 TF，81 帧没有有效 TF；38,437 抽样激光点，
  7,877 有效近水平射线；
- Bag 射线自由格 336，障碍 1,314，未知 41,360（96.16%）；
- 旧 PCD z=0 切片自由格 18，膨胀障碍 6,810；
- 两图以相同 XY origin、0.25m 分辨率、230×187 大小可进行栅格比较，
  但同格并不等于验证了 3D 刚性配准；
- Bag 自由、旧 PCD 切片障碍冲突 103 格；Bag 命中、旧 PCD 切片自由冲突 2 格；
- Bag 自由与旧 PCD 自由仅 3 格一致；
- **冲突自由格先标为 UNKNOWN**，不会把争议区域当作通行空间，剩余
  233 个仅雷达高度可见的自由格；最大 4-连通区 230 格；
- 使用最大候选区两端点，独立 A* 可得到约 8.286m 的**研究路线**，
  起点约 (0.02938,-0.30187)，终点约 (5.02938,-2.05187)。
  这是从实际数据自动选的端点，**不是任务目标**。

新增纯 Python 的 `pcd_bag_evidence_audit.py`、
`scripts/n01_pcd_bag_evidence_audit.py`，只读取已有地图 PGM+JSON，
不重读 Bag、不启动 ROS，输出冲突统计 JSON 和紫色冲突格 + 青色研究路线 SVG。
`--research-route` 才生成自动候选 A*；可用数字 `--start X Y --goal X Y`
代替，但会拒绝不在观测/非冲突候选自由格的端点。不产出
`/initial_path`、`/planning/bspline` 或任何底盘命令。

在用户 Ubuntu 拉取代码后用现有导出文件即可运行，不需要重算 180 帧 Bag：

```bash
cd /home/demo/lyy/hivecore-navigation
git -c http.version=HTTP/1.1 pull --ff-only origin main
python3 -m unittest discover -s src/n01_offline_integration/test \
  -p 'test_pcd_bag_evidence_audit.py' -v

python3 scripts/n01_pcd_bag_evidence_audit.py \
  --ray-prefix /home/demo/lyy/n01_runtime/pcd_goal/bag_ray_20261030_103843 \
  --pcd-prefix /home/demo/lyy/n01_runtime/pcd_goal/factory_map \
  --output-prefix /home/demo/lyy/n01_runtime/pcd_goal/g43_audit_$(date +%Y%m%d_%H%M%S) \
  --research-route
```

**目前不能称为机器人可通行或 SCAN 已接入：** Bag Ray FREE 代表激光高度附近穿越，
PCD 高度切片又未验证地面高度；多高度净空、地面、机器人碰撞包络、动态障碍
及离线重定位尚未验收。下一步 G4.4 应在原始 Bag 中加入多高度 ray 证据和
地面连续性检查；再做静态局部地图的 SCAN 查询接口与 
`fsm.navi_mode=3` 的 `/initial_path` 接入。
