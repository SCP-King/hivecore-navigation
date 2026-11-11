# N-01 导航项目 · AI 交接 README

> **写给接手的 AI**：这个文件包含你接手全部所需的信息。请先完整读完再动手。
> 最后更新：2026-11-10 22:20

---

## 0. 一句话现状

用户（缪豪楠）要在 **HiveCore 蜂核轮足机器人**上跑通「**FAST-LIO2 定位 + SCAN-Planner 规划 + 自研控制器驱动底盘**」的纯导航链路。
链路已能跑通（车实际动过 3.7 m），但**雷达安装倾角 10.54° 导致远处地面被当成障碍**，以及**若干节点会静默卡死**，这两个问题待解决。

---

## 1. 工作区位置（重要）

### 主副本在**设备（下位机）**上 —— 接手先看这里

```
/home/demo/lyy/hivecore-navigation/AI_HANDOVER.md      ← 主副本（仓库根目录，与代码同放）
/home/demo/lyy/AI_HANDOVER.md                          ← 同一份，放在 lyy 目录便于找
```

> ⚠️ **不要覆盖 `hivecore-navigation/README.md`**（363 行，是 N-01 官方工程文档）。本交接文档特意用了 `AI_HANDOVER.md` 这个名字。

代码、数据、工具**全部在设备上**，本地只是文档备份。接手时先 SSH 进设备读这份文档。

### 设备目录结构

```
/home/demo/lyy/
├── AI_HANDOVER.md                  ← 本文档
├── hivecore-navigation/            ← 乙方仓库（唯一允许改）
│   ├── README.md                   ← N-01 官方文档，勿覆盖
│   ├── AI_HANDOVER.md              ← 本文档副本
│   ├── src/
│   │   ├── n01_traj_tracker/       ← 我加的控制器（C++，已编译）
│   │   ├── n01_scan/               ← 含 ground_filter.hpp，未构建
│   │   └── scan_planner/           ← 上游 SCAN-Planner
│   ├── apps/n01_operator_console/  ← Web 操作台 + 采图器
│   ├── scripts/n01_cold_start.sh   ← 七步冷启动
│   └── install/                    ← **ros2 launch 实际读这里！**
├── n01_runtime/slam/               ← 所有 PCD 地图
└── docs/                           ← 厂商 PDF 文档
```

### 本地备份（macOS，仅文档）

```
/Users/miao/WorkBuddy/2026-11-10-18-53-59/workbuddy work/
├── README.md                       ← 本文档备份
└── SCAN-Planner_FAST-LIO2_部署方案.md  ← 早期方案（结论已部分被推翻，见 §11）
```

### 设备（Ubuntu 工控机，真正跑代码的地方）

| 项 | 值 |
|---|---|
| 地址 | `10.113.118.145`（校园网）；历史热点网段 `10.72.141.145` |
| 账号 / 密码 | `demo` / `123456` |
| sudo 密码 | **`123456`**（调用方式有坑，见 §6.1） |
| 主机名 | `demo-NuoDaJia-PC`（诺达佳工控机） |
| 硬件 | 8 核 i7-10700 @2.9GHz，15G 内存，**无 GPU**，kernel 6.1.12-rt7（PREEMPT_RT） |
| ROS | `/opt/ros/humble`，**`ROS_DOMAIN_ID=55`** |

### 连接方式（本机无 sshpass，用 expect）

已备好脚本（如果失效，按下面重建）：

```
/tmp/wb_ssh.exp   10.113.118.145 demo 123456 '<远端命令>'
/tmp/wb_scp.exp   <本地文件> demo@10.113.118.145:/tmp/ 123456
```

`wb_ssh.exp` 内容：
```expect
#!/usr/bin/expect -f
set timeout 300
set host [lindex $argv 0]
set user [lindex $argv 1]
set pass [lindex $argv 2]
set cmd  [lindex $argv 3]
spawn ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
  -o PreferredAuthentications=password -o PubkeyAuthentication=no \
  -o LogLevel=ERROR -o ConnectTimeout=15 $user@$host $cmd
expect {
  -re "(?i)password:" { send "$pass\r"; exp_continue }
  eof
}
catch wait result
exit [lindex $result 3]
```

**注意**：expect 的 pty 会让部分命令输出丢失（尤其含 `&` 后台任务、`pkill`）。**每条远端命令尽量短**，复杂的写成脚本上传再执行。

---

## 2. 三方代码边界（铁律，别越界）

| 归属 | 路径 | 权限 |
|---|---|---|
| **甲方（厂商）** | `/opt/hivecore/…`、`/opt/slam3d_release/…`、`/home/demo/hivecore_slam3d_release/…` | **禁止修改** |
| **乙方（我方）** | `/home/demo/lyy/hivecore-navigation` | 唯一允许改的仓库 |
| 文档 | `/home/demo/lyy/docs/1.如何使用 Hivecore Cloud Preprocessor.pdf` | 只读参考 |

乙方仓库：git 分支 `dev/n01-current`，远端 `github.com/SCP-King/hivecore-navigation`，**有大量未提交改动，不要 `git stash` / `checkout` / `commit`**。

---

## 3. 已构建的新包（我加的）

### 3.1 `n01_traj_tracker` — 自研轨迹跟踪控制器

位置 `/home/demo/lyy/hivecore-navigation/src/n01_traj_tracker/`（C++ ROS2，已编译）

**作用**：把 `/planning/bspline` 转成 `/cmd_vel` 驱动底盘。移植自上游 SCAN-Planner 的 `closed_loop_controller.cpp`（ROS1）。

关键设计：
- 订阅 `/planning/bspline`（**QoS 必须 RELIABLE = `rclcpp::QoS(10)`**）+ `/body_pose`（**必须 `rclcpp::SensorDataQoS()`**）
- 发布 `/cmd_vel` + `/n01/traj_tracker/status`(JSON)
- **默认 `enable=false`**，必须发 `/n01/traj_tracker/enable` (Bool) 才输出速度
- 限速 `max_vx/max_vy=0.30`、`max_vyaw=0.40`、20 Hz
- Hive2 是 4WIS 全向底盘，`linear.y` 有效，**不做投影**

### 3.2 操作台 Web（发 goal / 看状态）

```
/home/demo/lyy/hivecore-navigation/apps/n01_operator_console/
  n01_operator_console.py       # 主程序，纯 stdlib + rclpy
  n01_cloud_pcd_recorder.py     # 点云录制成 PCD
```

启动后访问 `http://10.113.118.145:8901/`。功能：点云俯视（车中心 12 m 视野）、点击发 goal、执行使能/急停、门控状态。

**它不碰 `/cmd_vel`**，只发 goal 和 enable 两个开关。

### 3.3 冷启动脚本

```
/home/demo/lyy/hivecore-navigation/scripts/n01_cold_start.sh
```

七步冷启动：清场 → 驱动 → FAST-LIO → 静态 TF → 位姿适配 → N-01 栈 → 控制器 → 前端。每步带等待与校验。

**必须用 `setsid bash scripts/n01_cold_start.sh` 启动**（理由见 §6.3）。

---

## 4. 当前地图与数据

| 文件 | 说明 |
|---|---|
| `~/lyy/n01_runtime/slam/fresh.pcd` | 最新原始点云（102032 点 / 986 帧），**含倾斜地面** |
| `~/lyy/n01_runtime/slam/fresh_bin.pcd` | 上一版可用地图（二进制，35563 点，未转正） |
| `~/lyy/n01_runtime/slam/leveled.pcd` | **已用 RANSAC 转正 10.54°**（含地面，50113 点） |
| `~/lyy/n01_runtime/slam/map_keepfloor.pcd` | 转正 + 保留地面 + 裁自车（48788 点，**但位姿未转正，会错位**） |
| `~/lyy/n01_runtime/slam/final_bin.pcd` | 转正 + 裁掉地面（34987 点，前方 2m 内 0 点，**但 overlay not ready**） |

---

## 5. 系统架构（当前实际链路）

```
Livox MID-360 (192.168.1.196, UDP)
    ↓ /livox/lidar (CustomMsg) + /livox/imu (200Hz)
livox_ros_driver2_node
    ↓
fast_lio (fastlio_mapping)  ──→ /Odometry (10Hz) + /cloud_registered_body
    ↓
n01_fast_lio_pose_adapter    ──→ /robot_pose (map→base_footprint) + /loc_status
    ↓
n01_localization_adapter     ──→ /n01/localization/{pose,status}
    ↓
n01_live_scan_bridge         ──→ /cloud (map 系) + /body_pose
    ↓
scan_planner_node (SCAN-Planner) ──→ /planning/bspline
    ↓
n01_traj_tracker             ──→ /cmd_vel
    ↓
steer_drive_node (厂商运控，接收 /cmd_vel)
```

**甲乙边界**：`/opt/hivecore/hive2-motion` 是运控（**绝对不能动**），它占 CPU0/CPU1 两个核。

---

## 6. 踩坑清单（最重要，请逐条阅读）

### 6.1 sudo 必须带 `-k`

```bash
# ✅ 可用
echo 123456 | sudo -S -k whoami        # → root
# ❌ 不可用（输出空）
echo 123456 | sudo -S whoami
```
**时间戳缓存无效，每条 sudo 命令都要单独带密码。** 且一条命令里只做一次 sudo（用 `sudo bash -c "..."` 包多条）。

### 6.2 `ros2 launch` 读的是 `install/` 里的拷贝！

改 `src/.../launch/xxx.launch.py` **不会生效**，必须：
- 改 `install/<pkg>/share/<pkg>/launch/xxx.launch.py`（Python 文件，改完即生效），或
- `colcon build --packages-select <pkg>`

**这是最容易浪费时间的坑，我在此浪费了 1 小时。**

### 6.3 后台进程必须 `setsid`

只用 `nohup` 时，SSH 会话一断，进程组就被带走（日志表现为 `ExternalShutdownException`、全部节点同时死）。

```bash
setsid nohup <cmd> > log 2>&1 < /dev/null &
```

### 6.4 N-01 的参数有硬编码上限，超限会拒绝启动

`live_scan_bridge.py` 第 112–139 行有 fail-closed 校验，超限直接 `ValueError` 并杀掉节点（表现为"只起了 4 个节点"，极易误判为资源不足）：

| 参数 | 合法范围 |
|---|---|
| `max_status_age_s` | 0.1–3.0 |
| `max_pose_stamp_age_s` | 0.1–2.0 |
| `max_pose_anchor_delta_s` | 0.1–1.5 |
| `max_cloud_age_s` | 0.1–2.0 |
| `max_live_body_pose_anchor_delta_s` | **≤ `max_pose_stamp_age_s`** |

**当前生效值**（`install/.../live_scan.launch.py`）：`max_cloud_age_s 1.6`、`max_pose_stamp_age_s 2.0`、`max_pose_anchor_delta_s 1.5`、`max_live_body_pose_anchor_delta_s 2.0`。

### 6.5 `static_map_node` 只认 **binary** PCD

ASCII 格式会被解成 1 个点（`pcd_bounds=[0.03,0.03]`，A* 必然失败）。用 `pcd_to_binary.py` 转换。

### 6.6 自车点必须按**车位置**裁，不能按原点裁

FAST-LIO 的 map 原点 ≠ 车的位置。按原点裁 0.85 m 等于没裁，车周围 0.51 m 处的车体点会让 `start_occ=1`。

用 `--ego-x/--ego-y` 传入**真实车位置**（从 `/robot_pose` 取）。

### 6.7 裁光地面会导致 `static_pcd_overlay_not_ready_in_current_window`

窗口内一个点都没有时，planner 认为"地图无信息"而拒绝。**地面要留着**，靠 `ground_z_map_m` 让 planner 自己过滤，不要物理删除。

### 6.8 延迟真凶是 `cube_side_length: 1000.0`

1 km³ 的局部地图让 ikd-Tree 随运行不断膨胀，处理越来越慢。**改成 `100.0`** 后 pipeline 从 1.5 s 降到 0.01 s。

**不要靠降载（减点/减迭代）解决延迟**——用户明确指出过"CPU 有大冗余"。实际 CPU 只用 6.7%，是在**等数据**。

### 6.9 `time_sync_en: true` 是错的

会触发 `lidar loop back, clear buffer` 然后卡死。保持 `false`，用 `time_offset_lidar_to_imu` 补偿（实测 `-1.149` 有效，但重启后会漂，不稳健）。

### 6.10 `/livox/lidar` 实际只有 6.3–7.5 Hz（标称 10 Hz）

受 CPU 竞争限制。**降 `publish_freq` 到 5.0 反而更差（实测 2.9 Hz）**，不要试。

### 6.11 已做的 CPU 隔离（重启后失效，需重做）

```
CPU0: wheel_controller / rt_ros2_control（运控，不动）
CPU2: fastlio_mapping
CPU3: livox_driver
CPU4-7: enp8s0 网卡中断（irq 151/152/153）
```

```bash
# 中断
echo 123456 | sudo -S -k bash -c "systemctl stop irqbalance; for i in 151 152 153; do echo 4-7 > /proc/irq/\$i/smp_affinity_list; done"
# 绑核（taskset 改自己进程不需要 root）
taskset -cp 2,3 <fastlio_pid>
```

### 6.12 节点会静默卡死（今晚各 4 次）

- `n01_fast_lio_pose_adapter`：进程活着但不发 `/robot_pose`，日志停在最后一条后无输出
- `static_transform_publisher`（`lidar_link→imu_link`）：会掉，掉了 adapter 就不出位姿
- `n01_localization_adapter`：`scripts/n01 navigation stop` 会**故意保留**它，再起 launch 会被 guard 拒（`Refusing to start a second N-01 live navigation stack`），必须先 `kill`

**判据**：进程存在但话题无数据 = 卡死，必须重启该节点。

### 6.13 UDP 中断全压在 CPU3

```
IRQ 153 (enp8s0-TxRx-2) → CPU3: 290,895,023 次
```
`irqbalance` 虽 active 但没生效。这是驱动只有 6.3 Hz 的深层原因。

---

## 7. 雷达倾角问题（当前主攻方向）

### 7.1 已定量确认

```
RANSAC 拟合地面：法向量 (-0.1782, 0.0415, 0.9831)
倾角 = 10.54°（pitch -10.27°, roll -2.42°）
地面点占 26%
```

**URDF/TF 里 RPL 全是 0**，即 `base_link→lidar_link` 报零旋转，但实际装了 10.54°。

**危害**：远处地面被抬高（2 m 抬 0.3 m）进入障碍带 → A* 把前方地面的延伸当障碍 → 绕路或规划失败。

### 7.2 试过但失败的路（不要重复）

| 方法 | 结果 |
|---|---|
| 把 RANSAC 旋转写进 FAST-LIO `extrinsic_R` | 报 `No Effective Points!`（方向反） |
| 用转置 `R^T` | 10.54° → 4.45°（改善但不彻底） |
| 再叠加一次补偿 | 恶化到 8.69° |
| **结论** | **外参补偿不收敛**——FAST-LIO 位姿估计会随外参重新收敛，二者强耦合 |
| 转正点云但不转位姿 | 车落到地图空白区，`overlay not ready` |

**重要认知**：**位姿本身没问题**，它和点云是自洽的。是我"只转点云"破坏了自洽性。别再怀疑位姿。

### 7.3 正确方向（待实施）

**方案 A：在点云进入栅格地图前，按倾斜平面剔掉地面点**（坐标不动、位姿不动）
- 改点：`grid_map.cpp:1115` `GridMap::cloudCallback`
- 地面方程：`z_ground = z0 + 0.186*x - 0.043*y`（0.186=tan(10.54°)，以车当前位置为参考）

**方案 A'（更省事，推荐）**：直接构建 N-01 里**已有**的 `n01_scan` 包
```
src/n01_scan/src/ground_filter.hpp   ← 法向量自适应判地面 + RANSAC，为此场景写的
```
该包**从未构建**（install 里没有）。`colcon build --packages-select n01_scan` 后用它的 `ground_filter`。

---

## 8. 工具箱（设备上 `/home/demo/lyy/tools/`）

```bash
# 用法示例（都要先 source 环境 + export ROS_DOMAIN_ID=55）
cd /home/demo/lyy/tools
python3 latency_probe2.py
python3 floor_tilt.py <某.pcd> <x> <y>
```

| 脚本 | 作用 |
|---|---|
| `latency_probe2.py` | **拆解延迟**：同时订阅 `/livox/lidar` 与 `/Odometry`，输出 `lidar_age/odom_age/pipeline` |
| `floor_tilt.py PCD x y` | RANSAC 量地面倾角 |
| `front_analysis.py PCD x y yaw [r]` | 按**车头朝向**看前方扇区的 z 分布（判断是地面还是真障碍） |
| `sector_clearance.py PCD x y [r] [yaw]` | 四方位净空分析 |
| `inspect_ego_area.py PCD x y [r]` | 看车周围分层点数 |
| `pcd_to_binary.py SRC DST [--min-z][--max-z][--ego-x][--ego-y][--ego-radius]` | ASCII→binary + 裁剪 |
| `pcd_level.py SRC DST [...]` | RANSAC 自动转正 + 裁剪 |
| `rd_probe.py TOPIC` | 订阅任意 String 话题打印完整 JSON（**别用 `ros2 topic echo`，长消息会被截断**） |
| `find_free_goals.py PCD` | 找空旷可达的候选目标点 |
| `n01_cloud_pcd_recorder.py --out X.pcd` | 点云录制成 PCD（FAST-LIO 自带 `pcd_save` 会段错误，必须用这个） |
| `n01_operator_console.py --port 8901` | Web 操作台 |
| `n01_cold_start.sh` | 七步冷启动 |

> **采图技巧**：录制器用 `kill -TERM` 才保存（`SIGINT` 被 rclpy 拦截不会保存），且常需 kill 两次。看到日志出现 `SAVED xxx points` 才算成功。

---

## 9. 常用命令

```bash
# 环境前缀（每条非交互命令必带）
source /opt/ros/humble/setup.bash
source /opt/hivecore/env/hive2_motion_current.sh   # livox 驱动
source /opt/slam3d_release/install/setup.bash      # LocStatus 消息
source /home/demo/lyy/hivecore-navigation/install/setup.bash
export ROS_DOMAIN_ID=55

# 一键冷启动（推荐）
cd /home/demo/lyy/hivecore-navigation
setsid bash scripts/n01_cold_start.sh > /tmp/boot.log 2>&1 < /dev/null &

# 起 N-01 栈（手动）
ros2 launch n01_live_navigation live_scan.launch.py \
  pcd_path:=/home/demo/lyy/n01_runtime/slam/<地图>.pcd \
  pcd_map_frame_asserted:=true ground_z_map_m:=-0.81 body_height_m:=0.90 \
  rviz:=false start_localization_adapter:=true

# 停 N-01
bash scripts/n01 navigation stop
# 注意：它会保留独立 localization adapter，再起 launch 前要 kill

# 急停
ros2 topic pub --once /chassis_dec_stop std_msgs/msg/Empty "{}"
ros2 topic pub --once /n01/traj_tracker/enable std_msgs/msg/Bool "{data: false}"
```

---

## 10. 下一步（按优先级）

1. **解决地面倾角** → 方案 A'（构建 `n01_scan` 用 `ground_filter`）或方案 A（改 `grid_map.cpp`）
2. **给 `pose_adapter` 和 `static_transform_publisher` 加看门狗**（`/robot_pose` 超 2 s 无更新就自动重启）
3. 重建一张干净地图（延迟已优化，现在采的图比之前准）
4. 若追求更好的实时性：让厂商 `hivecore_cloud_preprocessor` 接管预处理（文档在 `~/lyy/docs/`）

---

## 11. 给接手者的提醒

- **不要相信旧文档**：同目录的 `SCAN-Planner_FAST-LIO2_部署方案.md` 里"CPU 算力不足""位姿漂移"等结论**已被实测推翻**，以本文件为准。
- **先量再改**：任何"延迟/漂移/障碍"的结论，先用 §8 的探针拿数据。
- **改完必须验证**：这个系统里每个环节都有静默失效的坑，改完一定要看实际话题数据。
- **运控不能碰**：`/opt/hivecore` 下的任何东西、以及 CPU0/CPU1 上的运控进程，动了可能让车失控。
