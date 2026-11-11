# SLAM3D Lite GUI 故障排查

| 现象 | 可能原因 | 建议检查 |
|---|---|---|
| Web 页面打不开 | GUI 未运行或端口冲突 | 启动终端输出；`ss -ltnp \| grep 8123`；尝试 `./run_slam_lite_gui.sh 8090` |
| ROS 未连接 | 环境未 source、消息类型不可用或日志目录不可写 | 用随包 `run_slam_lite_gui.sh`；检查 Lite package prefix；检查 `log/ros` 权限 |
| ROS 未连接，顶部显示“启动失败：…” | ROS worker 初始化异常，或仍在运行旧版 GUI 进程 | 看具体异常；用 `run_slam_lite_gui.sh` 完全重启 GUI；刷新浏览器无效 |
| 设置 map_path 失败：`Node not found` | `/localization_lite_node` 未运行，或定位 desk 正在重启 | 先点“启动定位”，等节点上线并进入 Relocalization，再“设置并加载”；GUI 会等待并重试 |
| 定位地图和导航栅格不一致 | 定位 `map_path` 与导航 `N01_NAV_PCD` 不是同一个 PCD | 导航模块“同步定位地图”，重启导航；看“地图一致性” |
| 导航 RViz 没起来 / `qt.qpa.xcb: could not connect to display` | 当前环境没有可连接的 X/Wayland，或 `DISPLAY`/`XAUTHORITY` 不对 | 用 Foxglove；或在桌面终端设置 `DISPLAY=:0`、`XAUTHORITY` 后启动 GUI/导航 RViz；GUI 现在用 `xdpyinfo` 探测，连不上不会启动死进程；导航启动默认 `N01_NAV_RVIZ=false` |
| Lite 消息类型不可用 | source 了错误安装树 | `source /opt/slam3d_release/setup_runtime.sh`；检查 `ros2 pkg prefix hivecore_slam3d_lite_localization` |
| 节点存在但话题无数据 | 上游雷达/IMU未就绪或节点会话 Idle | 检查 `/livox/lidar/geo_filtered`、定位/建图会话状态和节点日志 |
| 服务不可用 | 对应 Lite 节点未启动或 ROS Domain 不一致 | `ros2 service list`、`ros2 node list`、核对 `ROS_DOMAIN_ID` |
| 初始位姿无效果 | 不在 Relocalization，或初值/坐标系不正确 | 确认 `session=relocalization`，检查 map 坐标系下 x/y/yaw |
| reloc_ok=false | 重定位未通过质量判定 | 检查地图、初值、点云、外参及实际阈值；不要绕过 `reloc_ok` |
| 地图/点云视图空白 | 对应 PointCloud2 无数据 | 检查 `/localization_map`、`/cloud_map`、上游点云及会话状态 |
| 定位与建图节点同时存在 | 两个互斥工作流被同时启动 | 停止其中一个；不可同时运行 |
| 结束建图后 save_map 失败 | deactivate_mapping 已清空内存地图 | 检查 deactivate_mapping 自动保存结果和 save_dir；不要在 Idle 再调用 save_map |
| RViz 无窗口 | 无 X11/桌面会话 | 使用本机图形终端或 X11 转发；否则用网页 2D/Foxglove |
| Foxglove 连接失败 | Bridge 未启动、8765 不通或转发地址错误 | GUI 启动 Bridge；连接 `ws://<host>:8765`；远程使用 SSH 端口转发 |
| Python/rclpy context 异常 | 同一解释器重复启动或旧 worker 未退出 | 彻底退出旧 GUI/Python 进程，使用普通持久终端重新启动 |
