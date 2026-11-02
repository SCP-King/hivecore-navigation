#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-}"
if [[ -z "$BAG" || ! -d "$BAG" ]]; then
  echo "Usage: bash scripts/run_n01_fast_lio2_bag.sh /path/to/rosbag2 [output.pcd]" >&2
  exit 2
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
MAP_OUT="${2:-/home/demo/lyy/n01_runtime/slam/fast_lio2_${STAMP}.pcd}"
RUN_DIR="/home/demo/lyy/n01_runtime/runs/fast_lio2_bag/${STAMP}"
mkdir -p "$RUN_DIR" "$(dirname "$MAP_OUT")"

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -n "${LIVOX_SETUP:-}" ]]; then
  source "$LIVOX_SETUP"
fi
source "$ROOT/install/setup.bash"

export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
export ROS_HOME="/home/demo/lyy/n01_runtime/ros_home/fast_lio2_bag"
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"

LIDAR_TOPIC="${N01_FAST_LIO_LIDAR_TOPIC:-/livox/lidar}"
IMU_TOPIC="${N01_FAST_LIO_IMU_TOPIC:-/livox/imu}"

echo "[N01-FAST-LIO2] Read-only Bag metadata preflight"
python3 "$ROOT/scripts/n01_fast_lio_bag_preflight.py" \
  --bag "$BAG" --lidar-topic "$LIDAR_TOPIC" --imu-topic "$IMU_TOPIC" \
  | tee "$RUN_DIR/input_preflight.json"

FAST_PID=""
BAG_PID=""
cleanup() {
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "$FAST_PID" ]] && kill -0 "$FAST_PID" 2>/dev/null; then
    kill -INT "$FAST_PID" 2>/dev/null || true
    wait "$FAST_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[N01-FAST-LIO2] Starting isolated mapping backend; no vendor SLAM or chassis nodes are launched."
ros2 launch n01_fast_lio fast_lio_mapping.launch.py \
  use_sim_time:=true map_file_path:="$MAP_OUT" \
  >"$RUN_DIR/fast_lio_launch.log" 2>&1 &
FAST_PID=$!

READY=0
for _ in $(seq 1 60); do
  if ros2 service list 2>/dev/null | grep -Fxq "/n01/fast_lio/map_save"; then
    READY=1
    break
  fi
  if ! kill -0 "$FAST_PID" 2>/dev/null; then
    echo "[N01-FAST-LIO2] FAST-LIO launch exited before service startup." >&2
    tail -n 80 "$RUN_DIR/fast_lio_launch.log" >&2 || true
    exit 3
  fi
  sleep 1
done
if [[ "$READY" != "1" ]]; then
  echo "[N01-FAST-LIO2] Timed out waiting for /n01/fast_lio/map_save." >&2
  tail -n 80 "$RUN_DIR/fast_lio_launch.log" >&2 || true
  exit 3
fi

# Replay only the two raw sensor inputs. Vendor TF/status and every control
# topic in the original Bag are deliberately excluded from this experiment.
echo "[N01-FAST-LIO2] Replaying only $LIDAR_TOPIC and $IMU_TOPIC in ROS domain 88."
ros2 bag play "$BAG" --clock --topics "$LIDAR_TOPIC" "$IMU_TOPIC" \
  >"$RUN_DIR/bag_play.log" 2>&1 &
BAG_PID=$!
if ! wait "$BAG_PID"; then
  BAG_PID=""
  echo "[N01-FAST-LIO2] Bag playback failed." >&2
  tail -n 80 "$RUN_DIR/bag_play.log" >&2 || true
  exit 4
fi
BAG_PID=""

if ! kill -0 "$FAST_PID" 2>/dev/null; then
  echo "[N01-FAST-LIO2] FAST-LIO process exited during replay." >&2
  tail -n 100 "$RUN_DIR/fast_lio_launch.log" >&2 || true
  exit 5
fi

sleep 2 # allow the final queued scan and map callback to drain

echo "[N01-FAST-LIO2] Saving accumulated FAST-LIO map."
SAVE_OUTPUT="$(timeout 30s ros2 service call /n01/fast_lio/map_save std_srvs/srv/Trigger '{}' 2>&1)"
printf '%s\n' "$SAVE_OUTPUT" | tee "$RUN_DIR/map_save.txt"
if ! grep -q "success: true" <<<"$SAVE_OUTPUT"; then
  echo "[N01-FAST-LIO2] Map save service did not report success." >&2
  exit 6
fi

kill -INT "$FAST_PID" 2>/dev/null || true
wait "$FAST_PID" 2>/dev/null || true
FAST_PID=""

if [[ ! -s "$MAP_OUT" ]]; then
  echo "[N01-FAST-LIO2] Map file missing or empty: $MAP_OUT" >&2
  exit 7
fi

cat >"$RUN_DIR/summary.txt" <<EOF
result=FAST_LIO2_BAG_MAPPING_OUTPUT_CREATED
backend=FAST_LIO2
bag=$BAG
lidar_topic=$LIDAR_TOPIC
imu_topic=$IMU_TOPIC
map_file=$MAP_OUT
vendor_slam_modified=NO
vendor_topics_republished=NO
fast_lio_tf_broadcaster=DISABLED
cmd_vel_published=NO
robot_motion_commands=NO
localization_against_saved_map=NOT_IMPLEMENTED
EOF

echo "=============================================="
cat "$RUN_DIR/summary.txt"
echo "LOGS: $RUN_DIR"
echo "=============================================="
