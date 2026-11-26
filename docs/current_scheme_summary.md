# N-01 当前方案与项目总览（2026-11-21）

## 1. 项目分层

```text
甲方 SLAM
  localization_lite / mapping_lite
  /loc_status /n01/vendor...pose_raw /localization_map /cloud_map
        ↓
乙方适配层
  n01_localization_adapter（pose/status/TF）
  n01_live_scan_bridge（/body_pose /cloud）
  n01_ground_reference（动态地面）
        ↓
导航
  n01_static_map → GridMap 局部3D + 持久全局2D
  → 全局 A* → 局部 SCAN → 轨迹/监视
        ↓
GUI / 安全监督 / 底盘（执行默认关闭）
```

### 离线导航的地图寿命

- 全局栅格从加载的 PCD 建立，在本次离线会话冻结。规划和实时观测不修改这份栅格，也不写回 PCD。
- 局部实测障碍参与碰撞和全局规划的临时搜索。同一三维体素有多帧、新鲜的穿越证据时，临时参考层可抑制该体素的静态障碍。证据过期或离开窗口后回退到长期参考。二维投影的空闲不覆盖三维障碍。
- 全局 A* 和选点预检共用临时搜索输入。预检在发送目标前定期发布。GUI 白色需要同时具备空闲证据与全局连通性。PCD 不包含历史空闲栅格，下次会话从当次实时扫描重新积累空闲证据。

## 2. 当前 frame 方案

当前采用 N-01 动态地面方案，不是 A 方案固定 frame。

```text
map
└── base_link                定位/adapter
    ├── base_footprint       动态地面投影（ground_reference）
    └── lidar_link           RSP: 0.295, 0, 0.146
        └── imu_link         甲方 static TF
            └── livox_frame  identity
```

### 静态机械量

| 项 | 值 | 来源 |
|---|---:|---|
| `base_link→lidar_link` | 0.295, 0, 0.146 | RSP/URDF 实机 |
| `lidar_link→imu_link` | -0.838236 / 10.326584 deg | N-01 5586 帧静置标定 |
| `imu_link→livox_frame` | identity | 甲方配置 |
| `base_link→base_footprint` | 动态，约 -0.768~-0.775 | ground_reference，不是固定值 |
| `body_height` | 0.90 m | N-01 配置 |
| `body_to_ground` | 0.775 m | legacy fallback，仅诊断 |

### 动态量

| 量 | 来源 |
|---|---|
| `map→base_link` | 定位位姿 |
| `base_link→base_footprint` | 地面投影 |
| `base_link→lidar_link` | RSP，值静态但运行期发布 |
| `lidar_link→imu_link` | 启动 seed + IMU 自动校准 |
| body pose / 轮组 / 关节 | 定位、底盘反馈 |
| 地面平面 | `estimate_ground` 动态拟合 |

## 3. 静态配置参数

### SLAM / 传感器

| 参数 | 当前 |
|---|---|
| tracking frame | `lidar_link` |
| mola vehicle frame | `lidar_link` |
| scan frame | `lidar_link` |
| `vehicle_to_scan` | `[0,0,0]` |
| `enable_imu_fusion` | false |
| `enable_extrinsic_tf_override` | true |
| reloc hunt | false |
| 雷达 crop | ±20 m |
| 雷达桥接 max range | 12 m（定位）/ 40 m（导航） |
| 雷达型号 | Livox Mid-360 |

### 地图 / 规划

| 参数 | 当前 |
|---|---|
| 静态 PCD 分辨率 | 0.10 m |
| 静态 z 窗口 | −1.2~1.2 m |
| 静态地面裁切 | ground + 0.06 |
| 地面过滤 | **0.06 m** |
| 局部地图 | 30 m |
| 局部前瞻 | 自动 14 m（半宽 −1） |
| max ray | 15 m |
| URDF 足迹 + environment margin | **6 cm** |
| 垂直膨胀 | up 0.90 / down 0.25 |
| 全局层 free/occupied/unknown | 动态构建 |
| 局部 unknown 回退全局 2D | true |
| 静态短期空闲证据 | true |

### 控制/验收

| 参数 | 当前 |
|---|---|
| 速度 | 0.4 m/s |
| 加速度 | 0.25 m/s² |
| 重规划阈值 | 0.1 m / 0.5 s |
| 执行器 | 默认不启动 |

## 4. 动态获取的数据

- `/loc_status`：定位会话、fitness、degraded；
- `/n01/vendor...pose_raw`：甲方位姿；
- `/n01/localization/pose`：adapter 输出 base_link；
- `/body_pose`：桥接后导航位姿；
- `/cloud`：导航实时点云；
- `/cloud_map`：建图累积；
- `/grid_map/reference_grid` / `live_grid` / `global_grid`；
- `n01_ground_reference` 的 plane/pose；
- 规划状态、路径、执行反馈。

## 5. 当前地面链路

```text
PCD/实时点云 + body pose
→ estimate_ground
→ plane a,b,c
→ base_link→base_footprint 动态投影
→ 导航静态/局部/全局地面过滤
```

`body_to_ground=0.775` 只在没有动态地面时兜底。

## 6. 当前已知不一致

| 项 | 状态 |
|---|---|
| 建图累积地面 vs live 地面 | 实测差约 3.7 cm |
| base_footprint 动态地面 vs 导航静态图 | 仍需持续验证 |
| relocalization | one-shot；hunt 默认关闭 |
| RViz/TF | 当前缺少/重复发布时需检查 |
| 地图版本 | 仍需 map_id/hash 统一 |
| 性能线程化 | 未完成 |

## 7. 绝不能随便改的参数

- 甲方 vendor static TF；
- RSP 的 `base_link→lidar_link`；
- N-01 5586 帧 IMU 标定；
- URDF 机械尺寸/足印；
- 参数必须注明来源，不能“试出来”。
