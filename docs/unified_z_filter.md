# 统一 z 过滤方案

## 1. 唯一高度带

所有障碍进入 2D 规划前，统一使用：

```text
z_min = ground(x,y) + 0.06
z_max = ground(x,y) + 1.60
```

其中：

```text
ground(x,y) = ground_z + slope_x*x + slope_y*y
```

## 2. 语义

```text
低于 ground+6cm：地面/地面噪声，忽略
高于 ground+1.60m：天花板/高处结构，不挡 2D 路径
中间：障碍候选
```

机器人机身高度约 0.90m；1.60m 作为统一上包络，覆盖机身和腿摆动。

## 3. 必须使用同一规则的层

- PCD 静态地图预处理；
- `global_grid`；
- `GridMap` static overlay；
- `getLiveObstaclePoints()`；
- `fused_2d_`；
- `planning_grid_`；
- `search_grid`；
- `publishProjectedGrids()`；
- GUI 低耗/细节点云/诊断。

任何一层都不能单独使用 1.20/1.57/1.60 另一套。

## 4. 风险

- 1.60m 以上的真实低悬障碍会被忽略；
- 地面坡度误差超过 6cm 时地面会进障碍；
- 2D 投影仍无法表达悬空物；
- 统一上包络会增加少量高处障碍，需现场确认。
