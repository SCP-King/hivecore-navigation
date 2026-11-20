# PCD 历史自车拖影清理

## 1. 录制建图轨迹

建图期间运行：

```bash
python3 apps/n01_slam_gui/tools/record_robot_pose_csv.py \
  --topic /robot_pose \
  --output /home/demo/lyy/slam_data/maps/mapping_traj.csv
```

CSV 格式：

```text
timestamp x y z qx qy qz qw
```

## 1.1 没有轨迹时从 bag 提取

建图没有自动录 CSV 时，bag 里通常有 `/robot_pose`：

```bash
python3 apps/n01_slam_gui/tools/bag_to_pose_csv.py \
  --bag /home/demo/lyy/slam_data/bags/bag_20261119_205530 \
  --topic /robot_pose \
  --output /home/demo/lyy/slam_data/maps/bag_20261119_205530_robot_pose.csv
```

bag 与 PCD 必须是同一次建图。

## 2. 清理 PCD

```bash
python3 apps/n01_slam_gui/tools/filter_pcd_ego_trail.py \
  --pcd /home/demo/lyy/slam_data/maps/raw.pcd \
  --trajectory /home/demo/lyy/slam_data/maps/mapping_traj.csv \
  --output /home/demo/lyy/slam_data/maps/clean.pcd \
  --length 0.58 --width 0.34 \
  --margin 0.06 --resolution 0.10 \
  --z-down 0.80 --z-up 0.90
```

有 URDF 足迹 JSON 时：

```bash
--footprint-json footprint.json
```

JSON 格式：`[[x,y], ...]`（机身坐标系，可选 `{"vertices_m":[...]}`）。

## 3. 规则

- 只删除历史足迹 + 余量内的点；
- 不按固定半径裁图；
- 没有轨迹时工具拒绝运行；
- 真实障碍若曾进入机器人足迹也会被删除，报告中注明；
- 输出新文件，不覆盖输入。

## 4. 推荐流程

```text
建图 + 录制 /robot_pose
→ 保存原始 PCD
→ filter_pcd_ego_trail.py
→ 用 clean.pcd 做定位/导航
→ 检查起点包络、窄通道、墙体保留
```
