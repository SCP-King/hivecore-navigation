# 现场自测 SOP —— 建图 · 定位 · 导航测试

> 面向现场实操:怎么把现场"扫成离线地图 → 存图 → 加载定位 → 在上面跑导航",以及三类测试
> (静态导航 / 动态避障 / 阻断恢复)怎么具体操作。
> 平台:HiveCore(Hive2 全向轮底盘 + HiveCore 3D SLAM)+ 乙方 DynaCBF 导航层。ROS 2 Humble。

---

## 0. 核心概念(先读这 3 行)

- **建图(mapping)= 一次性**,把现场扫成离线 `.pcd`;东西不变就不重扫。
- **定位(localization)= 每次测试都跑**:加载那张图,把自己定进去,出 `map` 定位。
- **导航 = 在定位之上跑 DynaCBF**:全局用离线图规划,局部用实时点云栅格避障。
  > ⚠️ 跑导航必须用 **localization(定位)**,不是 mapping(建图)。两者别混。

---

## 1. 前置环境准备

每次开机都要确保:

| 项 | 做法 |
|---|---|
| ROS 环境 | `source /opt/ros/humble/setup.bash` |
| 运行域(与甲方一致) | `export ROS_DOMAIN_ID=55`、`export FASTDDS_BUILTIN_TRANSPORTS=UDPv4`(桌面启动脚本已自动设为 55) |
| 乙方 overlay | `cd <乙方工作空间> && source install/setup.bash`(SRAM 侧不要 source `install/setup.bash`,用 `setup_runtime.sh`) |

速查命令:
```bash
ros2 node list            # 看有哪些节点
ros2 topic list | sort    # 看话题
ros2 topic hz /odom       # 看频率(底盘应 ~50Hz)
```

---

## 2. 一键启动流程(顺序固定)

> 桌面三件套: **Hive2底盘启动 → GUI-SLAM启动 → (以后)乙方导航启动**。

### 2.1 先起底盘
双击 `Desktop/Hive2底盘启动`(等价: `/opt/hivecore/hive2-motion/1.0.0/scripts/start_hive2_motion.sh`)
- 开 3 个 Tab:**[1/3] EtherCAT OP / [2/3] driver / [3/3] hive2_controller**
- 等 **[1/3] 全部从站显示 OP** 后,driver 和 controller 才会正常
- 确认:`ros2 topic hz /odom`、`ros2 topic echo -n1 /chassis_state`
- Web 调试/遥控:浏览器开 `http://127.0.0.1:8791/`(默认 mode=0 Web 模控)

### 2.2 再起 SLAM(GUI)
双击 `Desktop/GUI-SLAM启动`(等价: `source /home/demo/release_slam_3d/setup_runtime.sh && ros2 launch hivecore_console_slam3d gui_slam_console.launch.py use_sim_time:=false`)
- 等 `/livox/lidar` 有数据:`ros2 topic hz /livox/lidar`
- 起完会弹 7 标签页窗口: `preprocess / mapping / localization / octomap / bag / viz / debug`

### 2.3 以后:起乙方导航(本 SOP 的"跑导航"环节)
- 在定位跑起来后,再启动 DynaCBF 导航节点(见 §5,属乙方待开发/待对接部分)。

---

## 3. 模式 A —— 建图(一次性,离线)

> 现场第一次跑、或换场地时做;之后直接跳过本节。

1. GUI-SLAM 切到 **mapping** 页
2. 点 **Start**(首次会先 `stop_localization`;若之前加载过 overlay 会问"擦除 vs 重定位续建",选你想要的)
3. 用手柄或 Web(`http://127.0.0.1:8791`)把车**慢慢开遍现场**,把要导航的区域扫全
4. 扫完点 **Save**(或调 `~/save_cloud_map_once`),存成 `.pcd` 到你要的路径
5. 记住这个 `.pcd` 路径(定位时要用)

等效命令行(不依赖 GUI 的方式):
```bash
# 启动建图 / 停止建图(服务名按你 node list 里的实际节点名)
ros2 service call /lidar_3d_mapping_node/start_mapping std_srvs/srv/Trigger
# 保存地图
ros2 service call /lidar_3d_mapping_node/save_cloud_map_once hivecore_slam3d/srv/SaveCloudMap "{file_path: '/home/demo/resources/cloud_map.pcd'}"
```
> 注意:私有服务 `~/…` 默认解析到节点名下(如 `/lidar_3d_mapping_node/…`);请以 `ros2 node list` + `ros2 service list` 实际名称为准。

**本机已有现成图,环境未变可直接复用**,无需重扫:
- `/home/demo/resources/cloud_map.pcd`
- `/root/resources_storage/cloud_map_pgo.pcd`(即 SLAM 默认 `map_path`)

---

## 4. 模式 B —— 定位 + 导航测试(每次跑)

### 4.1 加载离线图 + 启动定位
GUI-SLAM → **localization** 页:
1. **Load**:选你存的 `.pcd`(或直接用默认 `/root/resources_storage/cloud_map_pgo.pcd`)
2. **Start**:开始定位
   - 若处于"已加载图但没进跟踪/没对好位",先调 **Relocalization / enter_relocalization**,或 RViz 点 `/initialpose` 给定粗略初始位
3. 确认 `map` 定位正常:
   ```bash
   ros2 run tf2_ros tf2_echo map base_link     # 位姿应连续、不跳变
   ros2 topic echo -n1 /localization_status    # 看 reloc_ok / 模式
   ```

### 4.2 跑导航(乙方 DynaCBF)
> 你现在 DynaCBF 还是仿真闭环;此处是**目标形态**的对接位(第 3 步待建)。
1. source 乙方 overlay,启动导航 bringup(之后提供)
2. 导航应订阅 `/robot_pose` 或 `map` TF 拿定位,加载同一张图做全局规划
3. 下发目标(工位点)开始导航;观察:全局路径 / 局部轨迹 / `/cmd_vel`

**真机上 /cmd_vel 的路径**:乙方导航 → `/cmd_vel` → 甲方 `steer_drive_node` 底盘。
确认指令被底盘吸收:
```bash
ros2 topic echo /cmd_vel        # 导航发的
ros2 topic hz /odom             # 车应在动
```

---

## 5. 三类测试具体怎么做

| 场景 | 对应需求 | 操作 |
|---|---|---|
| **静态导航** | 成功率 ≥95%、≤500ms | 加载定位后,依次下多个定点目标,记录成功率/耗时 |
| **动态避障** | 行人横穿 / 行人对向 / AGV 对向 | 让车走,人在前方横穿/对向走过;验收车是"提前平滑绕行"而非急刹 |
| **阻断恢复** | 局部绕行→重规划→等待/急停 | 规划好路径后在路中放临时障碍/木箱,验证:绕行→不行则重规划→再不行安全停车 |

性能/指标验收(记录到测试报告):
```bash
ros2 topic hz /global_path        # 规划频率(目标 500ms 内出一次)
timed_perf                       # 单次规划耗时统计(乙方自建工具)
```

---

## 6. 日常座舱命令速查

| 想看什么 | 命令 |
|---|---|
| 有哪些节点 | `ros2 node list` |
| 话题频率 | `ros2 topic hz /odom /livox/lidar /cmd_vel` |
| 底盘状态 | `ros2 topic echo /chassis_state` |
| 定位状态 | `ros2 topic echo /localization_status` |
| TF 是否通 | `ros2 run tf2_ros tf2_echo map base_link`、`tf2_echo odom base_link` |
| 建图状态 | `ros2 topic echo /mapping_status` |
| 机器人位姿 | `ros2 topic echo /robot_pose` |
| 底盘紧急停止 | `ros2 topic pub /chassis_dec_stop std_msgs/msg/Empty '{}'`(或 `/estop`) |

---

## 7. 排障速查(按出现顺序查)

| 现象 | 原因与处理 |
|---|---|
| `/odom` 没数据 | 底盘 Tab 3 没起来/从站非 OP;等 EtherCAT **全部 OP** 再启动运动层 |
| `/livox/lidar` 空 | SLAM GUI 没起 / livox 没开;`ros2 topic hz /livox/lidar` 确认 |
| RViz 点云空但有 echo | PointCloud2 设 **Best Effort** |
| `map` TF 查不到 | 定位没 Start,或 SLAM 后端未发布 map→tracking;先确认 `/localization_status` |
| 定位跳变/对不上 | 初始位没给准;用 `/initialpose` 再重定位一次 |
| 底盘报超速/超时停车 | `/cmd_vel` 停发超 0.2s → 自动停车(正常保护);检查是否有别的节点在抢 `/cmd_vel` |
| 手柄和 Web/导航抢指令 | 三者互斥;用 `/chassis_control_mode` 确认当前活跃模式再操作 |

---

## 8. 对接注意点(乙方落地时)

1. **先验证":M1 链路"** —— 先不起 SLAM,靠底盘 `/odom` TF 做"相对起点直行"测试,确认 **底盘收 `/cmd_vel`、感知吃 livox 点云、TF 链通**;再上 SLAM 定位测地图内导航。
2. **速度**:乙方导航限速 + 甲方底盘 `max_linear_speed` 都要对齐到 ≤1.0 m/s(甲方 Hive2 默认 2.0,改 `robot_motion.yaml` 或导航层)。
3. **两张图要分开用**:全局规划读 `.pcd` 静态图,局部避障靠实时点云栅格——别把动态障碍建进离线图。
4. **DynaCBF 运动模型是差速,甲方是全向轮(带 `vy`)**,对接时运动模型需扩成 3-DOF(见融入方案 §4)。