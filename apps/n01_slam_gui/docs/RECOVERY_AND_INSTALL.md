# 统一生命周期 GUI：修复与替换

## 1. 先退出旧 GUI

在旧 GUI 对应的终端按 `Ctrl+C`。如果终端不可用，按现场 PID 精确发送信号：

```bash
kill -INT <GUI_PID>
sleep 3
ps -p <GUI_PID> -o pid,ppid,stat,args
```

只有确认该 PID 仍是 `slam_lite_gui.py` 后，才对该 PID 单独发送 `SIGTERM`。不要使用 `killall`、`pkill -f ros` 或全局 ROS 清理。

## 2. 更新仓库内 GUI

```bash
cd ~/lyy/hivecore-navigation/apps/n01_slam_gui
chmod +x run_slam_lite_gui.sh tools/*.sh tools/*.py tests/*.py
```

GUI 现在随 `hivecore-navigation` 一起版本管理，不再用独立 ZIP 覆盖安装。更新前先检查仓库工作树，避免覆盖未提交修改。

## 3. 离线检查

```bash
bash apps/n01_slam_gui/tools/verify_lifecycle.sh
```

若需要运行 ROS 图集成测试，再在机器人实际 ROS 环境执行：

```bash
python3 apps/n01_slam_gui/tests/test_lifecycle.py
```

## 4. 启动

不要先 source 旧的 `~/latest` 或 `~/FROZEN` 工作空间。启动脚本会自行创建干净环境，再 source：

- `/opt/ros/humble/setup.bash`
- `/opt/slam3d_release/setup_runtime.sh`

执行：

```bash
./run_slam_lite_gui.sh
```

默认地址：`http://机器人IP:8123`

启动器不会继承旧的 `AMENT_PREFIX_PATH`、`CMAKE_PREFIX_PATH`、`PYTHONPATH`、
`LD_LIBRARY_PATH`、`COLCON_PREFIX_PATH` 和 `PKG_CONFIG_PATH`，并会验证 Lite
localization package prefix 是否位于所选 release 的 `install/` 下。

## 5. 启动前两阶段检查

1. 刷新本机 ROS CLI daemon，普查 ROS 节点。
2. 仅当发现 N-01 导航节点时，调用导航仓库自己的受管 stop 入口，并重新普查。

定位/建图 Lite 节点、定位适配器、Livox、IMU、底盘、腿控及控制器属于保护对象。
无法证明进程归属的节点只报告，不会自动杀死。

## 6. 若仍出现 `/home/demo/FROZEN/.../local_setup.bash`

干净环境已经排除了从旧 Shell 继承的 ROS 前缀。如果仍然出现这个路径，说明
`/opt/slam3d_release/setup_runtime.sh` 本身解析到了旧安装位置，或 `/opt` 路径
是指向旧目录的符号链接。先只读检查：

```bash
readlink -f /opt/slam3d_release
readlink -f /opt/slam3d_release/setup_runtime.sh
ls -ld /opt/slam3d_release /opt/slam3d_release/install
sed -n '1,70p' /opt/slam3d_release/setup_runtime.sh
```

不要直接删除 `/home/demo/FROZEN` 或重建 `/opt` 链接。先确认甲方当前 Lite release 的真实目录，再修正部署路径。

## 7. 生命周期安全边界

- GUI 只管理自己启动的 SLAM desk / RViz / Foxglove 进程组。
- N-01 导航由 `hivecore-navigation/scripts/n01_navigation.sh` 管理。
- GUI 正常退出时，只停止本 GUI 启动的导航。
- 用户显式点击“停止全部 GUI 托管模块”时，才调用导航受管 stop。
- 不会全局 kill ROS、传感器、底盘或控制器进程。
