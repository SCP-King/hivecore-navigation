# N01 G1-G3 真实历史 Bag 动态演示（待 Ubuntu 验收）

## 目标
让现有已验证的药厂 Bag → map 点云 → SCAN → G2 动态目标 → G3 离线数值轨迹贯穿 RViz；从真实 RViz 窗口录制 MP4，并由视频导出 GIF 和 PNG。只读、零底盘指令。

## 可显示的图层

| 图层 | ROS 话题 | 说明 |
| --- | --- | --- |
| 历史 Map 点云 | /cloud | 现有 Bag + 有效历史 TF；拒绝缺 TF 数据 |
| SCAN 体素障碍 | /grid_map/occupancy_inflate | 橙色膨胀障碍 |
| SCAN B 样条参考 | /nav/test/global_path | 浅蓝原始参考路径 |
| G3 名义轨迹 | /n01/offline/demo_nominal_path | 黄色 |
| G3 约束候选轨迹 | /n01/offline/demo_candidate_path | 绿色、仅数值研究候选，不能执行 |
| 机器人与运动假设 | /n01/offline/demo_markers | 机器人、运动候选框、速度箭头、未来 1.5s 预测线与标签 |

G3 图层只接受已经执行求解的历史帧；不把心跳当新轨迹。运动障碍图层只从精确一致的录制扫描 stamp 读取对应的目标记录，不把无关时刻混在一起。POST-HOC 晚到 Path 结果明确显示“POST-HOC HISTORICAL COMPARISON”，不是录制时的在线规划，更不是实际机器人执行避障。

## 一键演示（Ubuntu 22.04 / ROS 2 Humble）

在断开 Hive2 运动控制链、具有 X11 / XWayland 图形桌面的开发机中运行。需要 rviz2、colcon、ffmpeg、ffprobe、xdotool、xdpyinfo；默认 Bag 目录是 /home/demo/lyy/slam_data/bags/bag_20261027_225100。

```bash
cd /home/demo/lyy/hivecore-navigation
git -c http.version=HTTP/1.1 pull --ff-only origin main
bash scripts/run_n01_demo_replay.sh
```

如果缺少录制依赖，可在获得管理员许可后安装：

```bash
sudo apt install ffmpeg xdotool x11-utils ros-humble-rviz2
```

如果是纯 SSH 无图形会话，录屏会明确报错，不退化成全桌面录制。目标窗口必须是本次自动启动的 RViz 进程对应的 X11 window ID。启动前要求域 88 的 ROS 图为空，输出限定只读 MarkerArray、Path、原有 SCAN/G3 诊断。

## VS Code Remote-SSH 中出现 x11_display_unavailable

这属于图形会话前置检查失败，并不代表 PCD 加载、Bag 数据读取或规划算法失败。Remote-SSH 终端常没有 Ubuntu 图形桌面的 `DISPLAY` 和 X11 授权，即使安装了 RViz2 也不能直接录制窗口。

在 Ubuntu 实际 GNOME/X11 或 XWayland 图形桌面打开“终端”执行：

```bash
echo "DISPLAY=${DISPLAY:-unset} XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-unset}"
xdpyinfo -display "$DISPLAY" >/dev/null && echo X11_OK
cd /home/demo/lyy/hivecore-navigation
export N01_STATIC_PCD_PATH="/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd"
# 只有确认这份整图与录制 Bag 的 map 坐标基准相同才允许此声明
export N01_PCD_MAP_FRAME_ASSERTED=1
bash scripts/run_n01_demo_replay.sh
```

如果没有物理桌面，也可在合法授权的远程图形桌面会话中打开终端；本脚本目前不支持纯无头 SSH 录制。**不要**盲目设置 `DISPLAY=:0`、`xhost +` 或复制他人 `XAUTHORITY`；这些操作不能保证会话有效且可能削弱访问控制。修复后的 `summary.txt` 即使在早期检查失败也会如实显示 `static_pcd_requested=YES`，与是否成功发布 PCD 底图 `static_pcd_report` 分开。

## 自动生成的交付文件

保存到 /home/demo/lyy/n01_runtime/demo_exports/YYYYMMDD_HHMMSS_PID/ ：

- n01_g1_g2_g3_rviz.mp4：真实 RViz 窗口录屏，不是生成式视频。
- n01_g1_g2_g3_preview.gif：从真实 MP4 抽帧的预览。
- n01_g1_g2_g3_preview.png：从真实 MP4 抽帧的截图。
- recorded_bag_report.json：既有全量 Bag 数据链真实报告。
- demo_summary.json：真实有效点云、SCAN 与 G3 求解结果摘要。
- summary.txt 及 logs/：启动、记录、退出和 ROS 图清理证据。

手工已打开 RViz 窗口时可单独运行：

```bash
bash scripts/record_n01_demo.sh /tmp/n01_demo.mp4
# Ctrl+C 结束录屏，再导出
python3 scripts/export_n01_demo_gif.py --video /tmp/n01_demo.mp4 --gif /tmp/n01_demo.gif --still /tmp/n01_demo.png --start 5 --seconds 10
```

## G3.1 逐帧数值证据（新增，尚待用户 Ubuntu 复测）

回放脚本的 G3 节点会在本轮独立输出目录创建 `g3_frame_evidence.jsonl`。**仅**对 `solver_executed=true` 的真实历史 Bag 数值尝试逐行写入，心跳和未求解的等待结果不写入。每行包含：

- 原始历史扫描 `recorded_scan_stamp_ns`、参考 Path 序号及 `path_origin`，区分 `POST_HOC_HISTORICAL_COMPARISON` 与按照回放顺序使用参考几何的求解。两者都不代表历史时刻真正在线控制。
- `nominal_xy_m`、`candidate_xy_m`、`trajectory_changed`、`minimum_nominal_h_m2`、`minimum_candidate_h_m2`。没有动态运动候选约束时，h 为 null，**不能解读成安全距离足够**。
- `moving_constraint_tracks`、`cbf_constraints_checked`、`cbf_constraints_rejected`、`status`，以及明确 `safety_authorization=NEVER` 和 `actuator_output=DISABLED`。

脚本退出时按 JSONL 逐行核验，拒绝空文件、重复 `(scan_stamp, path_reference_seq, posthoc)`、不完整行及越权状态，同时核对至少包含 observer 报告的求解次数。通过才创建 `g3_evidence_summary.json`；若 observer 计数与本地求解节点日志计数不一致，摘要仅如实记录、不伪称完全一致。**验收时必须查看 `summary.txt` 与 `logs/g3_evidence_check.log`**。

新增数据仍然是有限时域平面点模型 + 未经真值标注的运动假设，不能用 `minimum_candidate_h_m2 >= 0` 推断实机碰撞安全。此前已生成的 MP4 不包含 JSONL；要取得 G3.1 证据需在图形桌面重新运行脚本，旧视频本身不代表新阶段通过。

## G3.2 可选整图 PCD 背景与短时轨迹辨识（代码已提交，待 Ubuntu 验收）

**重要区别：现有 `/cloud` 来自历史 Bag 的每帧 Livox 扫描，`/grid_map/occupancy_inflate` 来自 SCAN 局部障碍；两者都不是已保存的整张 PCD 地图。** 现增加只读 `/n01/offline/static_pcd_map` 灰色底图和更明显的黄色/绿色短时 Path 图层。PCD **只作背景展示**，不会被接入 SCAN、G3 或机器人运动控制，因此不代表全图全局规划已经实现。

1. 在 Ubuntu 主机查找真实整图：`find /home/demo/lyy -type f -iname '*.pcd' -printf '%s %p\n' | sort -nr | head -20`。
2. 查看目标 PCD 的头部直到 `DATA ...` 一行：`awk '{print} /^DATA[[:space:]]/{exit}' /真实路径/地图.pcd`，确认是否为 `ascii` 或 `binary`。当前**不支持** `binary_compressed`；若遇到此格式，只对拷贝进行格式转换，不修改原始文件。
3. **必须核实静态 PCD 坐标基准与旧 Bag 的 `map` 一致**：依据建图导出来源、SLAM 地图坐标原点与真实固定特征对齐证据；仅有 PCD 头部、相同文件名或设置 RViz `Fixed Frame=map` 均不足以证明这一点。当前代码仅能记录人为确认标志，不会自动验证地图配准。
4. 经确认后，再在 Ubuntu 图形桌面显式设置：
   ```bash
   cd /home/demo/lyy/hivecore-navigation
   export N01_STATIC_PCD_PATH="/真实路径/药厂整体地图.pcd"
   export N01_PCD_MAP_FRAME_ASSERTED=1
   bash scripts/run_n01_demo_replay.sh
   ```
   不设置 `N01_STATIC_PCD_PATH` 时保留旧演示，日志明确提示未加载整图；设置路径但未确认坐标系则主动中止。
5. 如启用成功，目录会增加 `pcd_static_preview.json`，包含读取格式、原始点数、抽样点数、XYZ 边界及“只读、未经 Bag 配准验证”的告警。退出后 `summary.txt` 显示 `static_pcd_requested=YES` 和 `static_pcd_report=YES`。灰色图层为 PCD 底图，白蓝色是帧点云，橙色是局部膨胀障碍；浅蓝色是 SCAN 参考，黄色是 G3 名义短时轨迹，绿色是 G3 候选短时轨迹。

当前 G3 搜索时域约 1.5s，目标由历史位姿约 1.5m 前方生成，**不是厂区 A→B 的全局路径规划**。黄绿两条线可能重合（没有合格动态候选约束时尤为常见），不可凭看不到偏移断言算法未运行。要做真正整厂全局路线，后续还需要确认 PCD 如何转换为可通行性/障碍地图、目标终点、机器人包络和长距离路径规划，这与本次“背景可视化”是不同阶段。

## 安全边界与验收

1. 同现有离线功能链相同，强制 ROS_DOMAIN_ID=88、ROS_LOCALHOST_ONLY=1、N01_HISTORICAL_BAG_ONLY=1。禁止底盘命令、真实 TF 广播和运动授权。
2. 启动前检测隔离 ROS graph 和 /cmd_vel publisher；非空则中止，不干扰外部节点。只清理这次带令牌的本轮子进程。
3. 必须存在真实 Bag SCAN/G3 求解报告，并验证 ffprobe 确认 MP4、真实视频抽帧 GIF/PNG、进程及图清理，否则不标记演示 PASS。
4. 画面中的 MOVING_HYPOTHESIS_UNVERIFIED 未经真值验证，绿色 G3 轨迹基于简化平面点模型，不能声称碰撞安全、实机行走或真实动态避障已经通过。
5. 这套可视化代码当前只完成仓库开发，尚未在用户图形桌面执行，不宣称用户已拿到可播放视频。

用户上一轮 G3 全量 Bag 软件测试已经 PASS（928 条已读取、571 map 帧、61 数值求解帧、10 帧含未验证动态候选约束），本演示在此基础上添加只读展示，而不是重做 G2、G3。
