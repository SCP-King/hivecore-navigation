#!/usr/bin/env bash
# N-01 冷启动脚本（FAST-LIO2 作唯一定位源 + SCAN-Planner + 轨迹跟踪器 + 操作台）
#
# 为什么要"冷启动"：这套链路里每个环节都缓存上游状态，热重启单个节点会留下
#   过期缓存/DDS 幽灵，导致 system_health 长期 BLOCKED。
#   必须按下面的顺序、带上等待时间，一次跑完。
#
# 依赖：/opt/hivecore（运控，不要动）、/opt/slam3d_release（只取 LocStatus 消息）
set +u

WS="$HOME/lyy/hivecore-navigation"
PCD="${PCD:-/home/demo/lyy/n01_runtime/slam/fast_lio_map_bin.pcd}"
DOMAIN="${ROS_DOMAIN_ID:-55}"
LOG_DIR=/tmp/n01_boot
mkdir -p "$LOG_DIR"

say() { printf '\n=== %s ===\n' "$*"; }

wait_topic() {
  local topic="$1" need="${2:-1}" tries="${3:-20}"
  for _ in $(seq 1 "$tries"); do
    if timeout 3 ros2 topic hz "$topic" 2>/dev/null | grep -q "average rate"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
source /opt/hivecore/env/hive2_motion_current.sh     # 带 livox 驱动
source /opt/slam3d_release/install/setup.bash        # LocStatus 消息类型
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="$DOMAIN"

say "0. 清场：停 N-01 残留（不动运控 / livox 以外的东西）"
timeout 60 bash scripts/n01 navigation stop >"$LOG_DIR/stop.log" 2>&1 || true
for p in $(pgrep -f "lib/n01_live_navigation|lib/n01_static_map|scan_planner_node|n01_traj_tracker"); do
  kill -9 "$p" 2>/dev/null
done
for p in $(pgrep -f "n01_fast_lio_pose_adapter|fastlio_mapping|static_transform_publisher"); do
  kill -9 "$p" 2>/dev/null
done
sleep 3
ros2 daemon stop >/dev/null 2>&1; sleep 2; ros2 daemon start >/dev/null 2>&1

say "1. Livox 驱动（必须先起，等 IMU 稳定）"
if ! pgrep -f "livox_ros_driver2_node" >/dev/null; then
  setsid nohup ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
    -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 \
    -p output_data_type:=0 -p frame_id:=livox_frame \
    -p user_config_path:=/opt/hivecore/hive2-motion/1.0.0/install/share/livox_ros_driver2/config/MID360_config.json \
    >"$LOG_DIR/livox.log" 2>&1 &
  sleep 10
fi
wait_topic /livox/imu || { echo "FAIL: /livox/imu 无数据"; exit 1; }
echo "OK: IMU 200Hz"

say "2. FAST-LIO2（驱动的 IMU 稳定后再起，否则卡在 Node init finished）"
setsid nohup ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file src/n01_fast_lio/config/hive2_mid360.yaml \
  >"$LOG_DIR/fastlio.log" 2>&1 &
sleep 15
wait_topic /Odometry || { echo "FAIL: /Odometry 无数据，看 $LOG_DIR/fastlio.log"; exit 1; }
echo "OK: /Odometry 10Hz"

say "3. 补 imu_link 静态 TF（URDF 里没有这个帧，缺了适配器就不出位姿）"
setsid nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 lidar_link imu_link \
  >"$LOG_DIR/tf.log" 2>&1 &
sleep 5

say "4. 位姿适配器：FAST-LIO 的 map 系位姿 -> /robot_pose + /loc_status"
setsid nohup ros2 run n01_fast_lio n01_fast_lio_pose_adapter --ros-args \
  -p require_alignment:=false -p input_topic:=/Odometry -p publish_tf:=true \
  >"$LOG_DIR/adapter.log" 2>&1 &
sleep 12
wait_topic /robot_pose || { echo "FAIL: /robot_pose 无数据，看 $LOG_DIR/adapter.log"; exit 1; }
echo "OK: /robot_pose 10Hz"

say "5. N-01 规划栈（用 FAST-LIO 自建地图；bridge 已改为订阅 /cloud_registered_body）"
setsid nohup env PYTHONUNBUFFERED=1 ros2 launch n01_live_navigation live_scan.launch.py \
  pcd_path:="$PCD" pcd_map_frame_asserted:=true \
  ground_z_map_m:=-0.81 body_height_m:=0.90 \
  rviz:=false start_localization_adapter:=true \
  >"$LOG_DIR/n01_live.log" 2>&1 &
sleep 25
echo "点云: $(timeout 5 ros2 topic hz /cloud 2>/dev/null | head -1)"
echo "就绪: $(timeout 6 ros2 topic echo --once /n01/live_navigation/navigation_ready \
  std_msgs/msg/Bool --qos-reliability best_effort 2>/dev/null | head -1)"

say "6. 轨迹跟踪控制器（默认 enable=false，绝不自动发速度）"
setsid nohup env PYTHONUNBUFFERED=1 ros2 launch n01_traj_tracker traj_tracker.launch.py \
  enable:=false >"$LOG_DIR/tracker.log" 2>&1 &
sleep 6

say "7. 操作台 Web（http://<本机IP>:8901/）"
if ! curl -s --max-time 2 http://127.0.0.1:8901/api/state >/dev/null 2>&1; then
  setsid nohup env PYTHONUNBUFFERED=1 python3 apps/n01_operator_console/n01_operator_console.py \
    --port 8901 >"$LOG_DIR/console.log" 2>&1 &
  sleep 4
fi

cat <<EOF

冷启动完成。日志在 $LOG_DIR/
  前端      : http://$(hostname -I | awk '{print $1}'):8901/
  手动使能  : ros2 topic pub --once /n01/traj_tracker/enable std_msgs/msg/Bool "{data: true}"
  急停      : ros2 topic pub --once /n01/traj_tracker/enable std_msgs/msg/Bool "{data: false}"
               ros2 topic pub --once /chassis_dec_stop std_msgs/msg/Empty "{}"

若就绪仍为 false，看 $LOG_DIR/n01_live.log 里的 blockers 第一条。
EOF
