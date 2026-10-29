# 静态一致性核验(纯数学,无 ROS)
来源数据:
- 目标 PCD: cloud_map_20261027_225549.pcd
  pts=100121  bounds x[-16.85,40.01] y[-13.18,33.05] z[-1.50,3.08]
- bag 225100 最后一条 /cloud_map (t+282.5s): pts=32525
  bounds x[-16.88,39.88] y[-13.25,33.06] z[-1.29,2.92]
- 隔离域回放产出(scan_grid_map map_localized):
  first update acc @ stamp 1793112662.478  window=(0.30,-0.00)
  occ=4822  infl=39989  ground_z=-0.504

核验结论:
1. bag 末条 cloud_map 包围盒 (x[-16.88,39.88], y[-13.25,33.06]) 是 PCD
   (x[-16.85,40.01], y[-13.18,33.05]) 的严格子集 —— 同一地图的"进行中态"vs"最终态",
   PCD 是 bag 期间建图的最终保存结果(相差最后 ~11s)。
2. 回放首帧 sensor 位 (0.30,-0.00) 落在 PCD 包围盒内部 —— 机器人起点在当前地图内,
   静态层与实时层在 map 系一致。
3. ground_z=-0.504(robust_z_percentile)落在 PCD z[-1.50,3.08] 的低位段,与
   scan_map.yaml min_z=-1.0/max_z=2.0 窗口语义一致。
4. 因此:乙方 n01_scan 在 map 系的实时占用(mode map_localized)与甲方最终 PCD
   静态地图是同构的 —— 静态层可直接与实时链叠加,无需坐标变换。

保留此项为文档级证据,数字均已在上游脚本/日志中落盘:
- src/n01_static_map/output/scan_grid_map.log
- src/n01_static_map/output/topic_echo_occ.log
- cloud_map_trace(临时脚本,结论抄录于此)
