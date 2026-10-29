# n01_static_map — 只读静态 PCD 地图供给(乙方新增桥节点)

把甲方 SLAM 保存的静态 PCD(纯 xyz,**binary**)加载为 map 帧静态占用 PointCloud2,
恒定发布到 `/grid_map/static_occupancy`,供 N-01 全局规划/可视化层作静态先验。

- 只读:源 PCD 以外部绝对路径参数传入,绝不写入、移动、覆盖;中间全在内存,不落盘。
- 数据源铁律:本节点**只声明** `frame_id="map"`(由甲方 SLAM 保存语义决定),
  不做任何 TF 假设、不发布 TF、不改甲方 SLAM 任意文件。
- 动态险情(车辆/行人)不属本层,需 `/livox/lidar/geo_filtered` + `/local_pcd`
  实时链路(`n01_perception` + `n01_scan`)。本节点不替代实时感知。

## 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `pcd_path` | `""` | 甲方 SLAM 静态 PCD 绝对路径(未设置则节点空转) |
| `resolution` | `0.05` | 体素边长 m(与 n01_scan `scan_map.yaml` 一致) |
| `publish_rate` | `2.0` | 恒定发布速率 Hz |
| `frame_id` | `"map"` | 输出 frame(只声明,不做 TF) |
| `min_z` / `max_z` | `-1.5` / `3.0` | 参与体素化的 z 窗口(世界系) |
| `seed` | `0` | 每体素格代表点选取种子(确定性可复现) |

## 运行

```bash
source /opt/ros/humble/setup.bash
ros2 launch n01_static_map static_map.launch.py \
  pcd_path:=/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd
```

> 可安全合并进 `n01_bringup/map_visualization.launch.py`,或独立启动作为静态层。
> 注意:launch 里 `LogInfo` 已显式提醒——**该 PCD 的坐标系与甲方 map 帧一致性
> 需甲方定位确认,乙方不臆造 TF**。

## 离线校验

```bash
# 纯只读:校验目标 PCD 与对应 bag 的结构化对应性(TF 链、/cloud_map 演变、主题清单)
source /opt/ros/humble/setup.bash
python3 src/n01_static_map/scripts/offline_verify.py \
  --pcd  /home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd \
  --bag  /home/demo/lyy/slam_data/bags/bag_20261027_225100
```

## 接口冒烟(隔离域,不碰真机)

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
python3 src/n01_static_map/scripts/interface_smoke.py \
  --pcd /home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd
```

冒烟在进程内起节点 + 自订阅者,验证"加载→体素化→发布"全链路,
不向真机/网络发送任何指令,结束后 shutdown 无残留。

## 2026-10-28 已核验结论(target PCD = bag_20261027_225100)

- PCD: `cloud_map_20261027_225549.pcd` 100121 点全部 finite,
  bounds x[-16.85,40.01] y[-13.18,33.05] z[-1.50,3.08] (0.05m 体素化=98966 独立格)。
- bag span 283.9s; `/cloud_map` 147 条,末条(282.5s)32525 点
  bounds x[-16.88,39.88] y[-13.25,33.06] —— **全程从 3652 点只长到 32525**,
  即 bag 内不含最终版 100121 点地图;最终 PCD 在 bag 结束后 ~11s 才保存(slam3d 日志)。
- TF 链完整: `map→base_footprint`(233)→`base_link`(866)→`lidar_link`(866),
  静态边 `lidar_link→imu_link→livox_frame` 也在。
- 结论:本 bag 验证地图覆盖 / TF 链 / 话题稳定,**不能验证 100121 点最终版**
  (它不在 bag 内)。N-01 接入的总前提(甲方 map TF 可用)在本 bag 成立。