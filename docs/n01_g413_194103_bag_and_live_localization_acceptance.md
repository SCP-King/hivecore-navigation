# G4.13 19:41 双证据验收：离线导航话题通；在线 SLAM 定位未启动/未观察（2026-10-30）

本报告来自用户上传的 `n01_bag_functional_20261030_194103_1618696.tar.gz` 和用户粘贴的 15s 在线只读定位 JSON；两者分别是在**隔离历史 Bag 环境**和**机器人真实 ROS 图**中获取，不能混成一次实时导航测试。离线报告生成于 Git 提交 `81f7bc6`，2026-10-30 19:41:03 +08:00。当前源码对报告反映的小问题已有修复；下面的证据值以**原上传压缩包**为准，而非重跑后的宣称。

## A. 历史 Bag G4→SCAN→G2→G3：首次成功观察话题顺序

原始 `summary.txt`：
- `result=BAG_FUNCTIONAL_PATH_SMOKE_PASS`、exit_code=0；
- `owned_process_cleanup=CLEAN`、`post_cleanup_ros_graph=CLEAN`；
- 隔离 `ROS_DOMAIN_ID=88`、`ROS_LOCALHOST_ONLY=1`，且图中 /cmd_vel publishers=0。

`recorded_bag_report.json`：原 Bag 928 个 filtered 点云，571 个成功转换，357 因采集时间戳缺 map→base_footprint TF 被拒绝；回放真实使用 `historical_mount_profile=MAPPING_SLAM_DYNAMIC_0P30`、`height_used_m=0.3`，未重发原冲突 TF；历史 TF 中存在 .3/.9 的矛盾。G4 研究路径 `path_geometry_length_m=8.036`，29 点，明确 `map reference z=-0.55` 是候选；`tmp/launch_params_0py2fs6z` 证实原生 SCAN runtime mode3、`grid_map.ground_height=-0.55`、`grid_map.body_height=.85`；`tmp/launch_params_33zxpbok` 证实历史回放 .3 档被安装参数真正采用。

`g4_chain_evidence.json`：严格消息顺序 `/initial_path → /planning/bspline → /nav/test/global_path → G2 → G3` 观察到：
- 1 条与 G4 路线匹配的初始路径；发布前有效原生 B 样条 0，之后有效 B 样条 1；
- 之后 SCAN bridge path 1；后续 dynamic track 报告 567、G3 solver 报告 744、G3 candidate Path 104。这些是话题消息计数，不是成功规划/安全移动次数；
- `strict_callback_observation_order=true`、`issues=[]`，报告 `G4_TO_G3_TOPIC_FLOW_OBSERVED_NOT_SAFETY_CERTIFIED`；
- **同时明确** `g3_output_proven_to_follow_the_new_g4_route=false`、`static_full_pcd_integrated_into_scan_grid=false`、`terrain_and_collision_verified=false`，只能作为**端到端软件话题联通**里程碑。

`g3_evidence_summary.json`：108 次求解尝试 = 68 次真实回放先后顺序 + 40 次事后回溯；104 个有候选的筛查都满足 Hive2 / SCAN 的平移约束（使用原定离散四位小数误差容差），4 次不可行；动态移动假设出现 18 次，CBF 检查 72,324 次，**CBF candidate pruning=0**，不能宣称动态避障干预成功。 `g44_geometry_audit.json` 29 点中有 **9 个 PCD 表面障碍冲突路点**、20 个研究层无采样障碍候选路点；地面支撑/全身外廓均未获真实验收，不能把研究路径视为可驾驶路线。

### 已发现并修复的小问题

`g4_prepared/manifest.json` 在这份上传档案中最后有字面量 `\\n`，使严格 JSON parser 提示 `Extra data`，尽管其 JSON 正文内容可恢复且原始 G4 运行成功。问题在 `scripts/run_n01_g4_full_chain.sh` 研究档尾部换行编码；**已改为实际换行**，添加相应回归测试。无需因此重录 Bag，也不影响原始 `g4_chain_evidence.json` 和 `g3_evidence_summary.json` 的有效性。

## B. 机器人在线只读 SLAM 15s 观测：没有进入 tracking

用户真实在线 JSON：
- `/tf_static` 观察到仅 1 种 mount +0.90m；没有观察到 +0.30m。两个 ROS 图 TF 发布端均归属节点 `/robot_state_publisher`。**这只是观测窗口和端点信息，不能从这里精确归因每一条 TF 的作者或断言以后绝无冲突**。
- `map_to_base_footprint_seen=false`；
- `last_localization_status=null`，但 `status_subscription_error=null`（消息类型导入未失败，**是观测期间没收到状态**）；
- `last_filtered_cloud=null`（观测期间未收到 `/livox/lidar/geo_filtered`）；
- 可见 `/cmd_vel` 发布端 `/cmd_wheel_preprocessor_node`、`/person_follow_controller_node`。有发布端 ≠ 有运动命令，但说明实时图含潜在控制链。**现阶段不允许向该在线环境注入乙方导航输出**。
- 保护状态 `BLOCKED_POSE_OR_SENSOR_NOT_OBSERVED` 正确；扩展后的脚本还会在 `blocking_conditions` 一次性显示位姿、过滤点云、fresh tracking 和 cmd_vel publisher 独立问题。

这份在线报告更像是**运动 RSP 已在场，甲方 SLAM localization 及 Livox 的 filtered 转发未形成数据**。无法仅凭这 15s 判定甲方 SLAM 程序未启动、运行在不同 ROS_DOMAIN_ID、还在 mapping 而非 localization，或前端尚未收到 LiDAR；需要读实时节点、话题和终端启动记录，不能猜设备异常。

## C. 接下来最短路径：两部分不混合

**不需再次重复历史 Bag 基线。** G4→SCAN→G2→G3 话题集成可以作为完成的离线软件里程碑。后续需要单独核验新 G4 路线到 G3 的几何依赖和原生 SCAN 使用完整静态 PCD 的接口；这两个缺口不阻碍先研究机器人**静止在线定位**。

在线只读步骤（甲方操作员在机器人静止、无导航指令的原 ROS 环境）：
```bash
ros2 node list
ros2 topic list -t | grep -E 'localization_status|mapping_status|geo_filtered|/tf|/cmd_vel'
ros2 topic info /localization_status --verbose
ros2 topic info /livox/lidar/geo_filtered --verbose
ros2 topic info /cmd_vel --verbose
```
目的仅为确认甲方的 Tracking Localization 和 Livox / geo_filtered 链是否正常存在、与观察终端在同一 DDS 域及是否有未授权运动发布者；这组命令均为读操作。由甲方工程师按**甲方文档**加载旧 PCD 的定位模式并确保仅 Hive2 0.90m RSP；**不要**让本项目调用甲方启动服务或驱动车轮。重新运行 `scripts/n01_live_localization_readonly_audit.py` 仅在状态/点云确实出现之后，输出不应再缺 `map→base_footprint`、tracking / filtered cloud。再用记录到的原地图 PCD 与在线 LiDAR 做只读配准校验。

在前述条件和实时唯一 TF、静态 PCD 融合、四轮转向、急停/刹车、安全监护验收前，**禁止低速自动行驶试验**。不应该仅通过关掉 `slam_rsp_disabled_confirmed` 保护或设置 `N01_G4_FLOOR_Z_VERIFIED=1` 伪造可运行状态。
