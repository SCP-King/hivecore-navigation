"""Live Hive2 localization + LiDAR adapter for native SCAN.

Consumes the vendor's LocalizationStatus, TF and filtered PointCloud2.
Publishes only SCAN inputs (/cloud, /sensor_pose, /body_pose) and diagnostics.
It has no publisher for cmd_vel, wheel commands, chassis APIs or action controls.

Clouds are held in a small bounded queue while scan-time TF catches up. A cloud
is only published with transforms queried at its original header.stamp, and is
dropped if either its age or TF wait exceeds the configured safety bound.
"""
import json
import math
import struct
import time
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformListener, TransformException

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from .core import localization_is_tracking, rotate_translate, estimate_velocity


class LiveScanBridge(Node):
    def __init__(self):
        super().__init__("n01_live_scan_bridge")
        self.declare_parameter("status_topic", "/localization_status")
        self.declare_parameter("status_type", "hivecore_slam3d/msg/LocalizationStatus")
        self.declare_parameter("cloud_topic", "/livox/lidar/geo_filtered")
        self.declare_parameter("expected_cloud_frame", "lidar_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("max_status_age_s", 1.0)
        self.declare_parameter("max_cloud_age_s", 0.6)
        self.declare_parameter("max_tf_wait_s", 0.45)
        self.declare_parameter("tf_retry_period_s", 0.02)
        self.declare_parameter("max_pending_clouds", 8)
        self.declare_parameter("max_points", 100000)
        self.declare_parameter("max_point_range_m", 12.0)

        self.status_topic = str(self.get_parameter("status_topic").value)
        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.cloud_frame = str(self.get_parameter("expected_cloud_frame").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.body_frame = str(self.get_parameter("body_frame").value)
        self.max_status_age = float(self.get_parameter("max_status_age_s").value)
        self.max_cloud_age = float(self.get_parameter("max_cloud_age_s").value)
        self.max_tf_wait = float(self.get_parameter("max_tf_wait_s").value)
        self.tf_retry_period = float(self.get_parameter("tf_retry_period_s").value)
        self.max_pending_clouds = int(self.get_parameter("max_pending_clouds").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.max_range = float(self.get_parameter("max_point_range_m").value)

        if not (
            0.1 <= self.max_status_age <= 3.0
            and 0.1 <= self.max_cloud_age <= 2.0
            and 0.01 <= self.max_tf_wait <= 1.0
            and 0.005 <= self.tf_retry_period <= 0.1
            and 1 <= self.max_pending_clouds <= 32
            and 100 <= self.max_points <= 300000
            and 1.0 <= self.max_range <= 50.0
        ):
            raise ValueError("live bridge parameters outside bounded limits")

        status_type = str(self.get_parameter("status_type").value)
        self.status_class = get_message(status_type)
        self.status_ok = False
        self.status_reason = "waiting_localization_status"
        self.status_rx_monotonic = 0.0
        self.status_imu_sync = False
        self.status_fitness = float("nan")
        self.last_body = None
        self.last_velocity_valid = False

        # FIFO queue entries are (PointCloud2, enqueue_monotonic, wait_counted).
        self.pending_clouds = deque()
        self.last_cloud_out_monotonic = 0.0
        self.last_cloud_age_s = None
        self.last_tf_wait_s = 0.0
        self.max_tf_wait_observed_s = 0.0

        self.counts = {
            "cloud_in": 0,
            "cloud_queued": 0,
            "cloud_out": 0,
            "cloud_tf_wait_started": 0,
            "cloud_tf_wait_completed": 0,
            "drop_status": 0,
            "drop_frame": 0,
            "drop_stamp": 0,
            "drop_tf": 0,
            "drop_tf_timeout": 0,
            "drop_stale_after_wait": 0,
            "drop_queue_overflow": 0,
            "drop_cloud": 0,
        }

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.status_sub = self.create_subscription(
            self.status_class, self.status_topic, self.on_status, qos_profile_sensor_data)
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self.on_cloud, qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, "/cloud", qos_profile_sensor_data)
        self.sensor_pose_pub = self.create_publisher(Odometry, "/sensor_pose", qos_profile_sensor_data)
        self.body_pose_pub = self.create_publisher(Odometry, "/body_pose", qos_profile_sensor_data)
        self.diag_pub = self.create_publisher(String, "/n01/live_navigation/readiness", 10)

        self.create_timer(self.tf_retry_period, self.process_pending_clouds)
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().warning(
            "READ-ONLY LIVE SCAN BRIDGE: bounded scan-time TF queue enabled; "
            "no actuator publishers; IMU sync remains diagnostic")

    def on_status(self, msg):
        ok, reason = localization_is_tracking(msg)
        self.status_ok = ok
        self.status_reason = reason
        self.status_rx_monotonic = time.monotonic()
        self.status_imu_sync = bool(getattr(msg, "fusion_path_imu_sync_ok", False))
        self.status_fitness = float(getattr(msg, "registration_output_fitness_mse", float("nan")))

    def _status_is_fresh(self):
        return (
            self.status_ok
            and self.status_rx_monotonic > 0.0
            and time.monotonic() - self.status_rx_monotonic <= self.max_status_age
        )

    @staticmethod
    def _transform_from_msg(msg):
        t = msg.transform.translation
        q = msg.transform.rotation
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

    def _read_xyz(self, msg):
        fields = {f.name: f for f in msg.fields}
        if not all(k in fields for k in ("x", "y", "z")):
            raise ValueError("cloud_missing_xyz")
        fmt_by_type = {
            PointField.FLOAT32: ("f", 4),
            PointField.FLOAT64: ("d", 8),
        }
        specs = []
        for name in ("x", "y", "z"):
            f = fields[name]
            if f.datatype not in fmt_by_type or f.count != 1:
                raise ValueError("cloud_xyz_must_be_float")
            fmt, size = fmt_by_type[f.datatype]
            if f.offset + size > msg.point_step:
                raise ValueError("cloud_xyz_offset_out_of_record")
            specs.append((f.offset, fmt))
        total = int(msg.width) * int(msg.height)
        if total <= 0 or total > self.max_points or msg.point_step <= 0:
            raise ValueError("cloud_point_count_out_of_bounds")
        if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.row_step * msg.height:
            raise ValueError("cloud_data_layout_invalid")
        packed = bytearray(total * 12)
        out_count = 0
        endian = ">" if msg.is_bigendian else "<"
        for row in range(msg.height):
            row_base = row * msg.row_step
            for col in range(msg.width):
                base = row_base + col * msg.point_step
                try:
                    x, y, z = (struct.unpack_from(endian + fmt, msg.data, base + off)[0]
                               for off, fmt in specs)
                except struct.error as exc:
                    raise ValueError("cloud_record_truncated") from exc
                if not all(math.isfinite(v) for v in (x, y, z)):
                    continue
                if math.sqrt(x*x + y*y + z*z) > self.max_range:
                    continue
                struct.pack_into("<fff", packed, out_count * 12, x, y, z)
                out_count += 1
        if out_count == 0:
            raise ValueError("cloud_has_no_usable_points")
        return bytes(packed[:out_count * 12]), out_count

    def _xyz_map_cloud(self, msg, translation, quaternion):
        payload, count = self._read_xyz(msg)
        out = bytearray(count * 12)
        for i in range(count):
            point = struct.unpack_from("<fff", payload, i * 12)
            transformed = rotate_translate(point, translation, quaternion)
            struct.pack_into("<fff", out, i * 12, *transformed)
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.header.frame_id = self.map_frame
        cloud.height = 1
        cloud.width = count
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = count * 12
        cloud.is_dense = True
        cloud.data = bytes(out)
        return cloud

    @staticmethod
    def _odom(stamp, parent, child, transform, velocity=(0.0, 0.0, 0.0)):
        translation, quaternion = LiveScanBridge._transform_from_msg(transform)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = translation
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = quaternion
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = velocity
        return msg

    def on_cloud(self, msg):
        self.counts["cloud_in"] += 1
        if not self._status_is_fresh():
            self.counts["drop_status"] += 1
            return
        if msg.header.frame_id.lstrip("/") != self.cloud_frame:
            self.counts["drop_frame"] += 1
            return

        stamp_ns = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        age_s = (now_ns - stamp_ns) / 1e9
        if stamp_ns <= 0 or age_s > self.max_cloud_age or age_s < -0.2:
            self.counts["drop_stamp"] += 1
            return

        if len(self.pending_clouds) >= self.max_pending_clouds:
            self.pending_clouds.popleft()
            self.counts["drop_queue_overflow"] += 1
            self.get_logger().warning(
                "drop oldest pending cloud: scan-time TF queue reached its bounded capacity")

        self.pending_clouds.append((msg, time.monotonic(), False))
        self.counts["cloud_queued"] += 1

    def process_pending_clouds(self):
        # Preserve scan order: a newer cloud must not overtake an older pending scan.
        while self.pending_clouds:
            msg, enqueued_monotonic, wait_counted = self.pending_clouds[0]
            stamp_ns = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
            age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            waited_s = time.monotonic() - enqueued_monotonic

            if age_s > self.max_cloud_age:
                self.pending_clouds.popleft()
                self.counts["drop_stale_after_wait"] += 1
                self.get_logger().warning(
                    "drop pending cloud: age %.3fs exceeded max_cloud_age_s %.3fs"
                    % (age_s, self.max_cloud_age))
                continue

            if not self._status_is_fresh():
                self.pending_clouds.popleft()
                self.counts["drop_status"] += 1
                continue

            try:
                scan_time = Time.from_msg(msg.header.stamp)
                # Zero-timeout lookups are retried by this timer. This avoids
                # blocking the cloud callback while the localization TF catches up.
                tf_lidar = self.tf_buffer.lookup_transform(
                    self.map_frame, self.cloud_frame, scan_time,
                    timeout=Duration(seconds=0.0))
                tf_body = self.tf_buffer.lookup_transform(
                    self.map_frame, self.body_frame, scan_time,
                    timeout=Duration(seconds=0.0))
            except TransformException as exc:
                if not wait_counted:
                    self.counts["cloud_tf_wait_started"] += 1
                    self.pending_clouds[0] = (msg, enqueued_monotonic, True)
                if waited_s >= self.max_tf_wait:
                    self.pending_clouds.popleft()
                    self.counts["drop_tf"] += 1
                    self.counts["drop_tf_timeout"] += 1
                    self.last_tf_wait_s = waited_s
                    self.max_tf_wait_observed_s = max(
                        self.max_tf_wait_observed_s, waited_s)
                    self.get_logger().warning(
                        "drop cloud: scan-time TF unavailable after %.3fs wait "
                        "(cloud age %.3fs, max wait %.3fs): %s"
                        % (waited_s, age_s, self.max_tf_wait, exc))
                    continue
                break

            try:
                translation, quaternion = self._transform_from_msg(tf_lidar)
                cloud_map = self._xyz_map_cloud(msg, translation, quaternion)
            except (ValueError, OverflowError) as exc:
                self.pending_clouds.popleft()
                self.counts["drop_cloud"] += 1
                self.get_logger().warning("drop cloud: invalid XYZ payload: %s" % exc)
                continue

            self.pending_clouds.popleft()
            body_xyz = (tf_body.transform.translation.x,
                        tf_body.transform.translation.y,
                        tf_body.transform.translation.z)
            stamp_s = stamp_ns / 1e9
            velocity, velocity_valid = estimate_velocity(
                self.last_body, (body_xyz, stamp_s))
            self.last_body = (body_xyz, stamp_s)

            body_odom = self._odom(
                msg.header.stamp, self.map_frame, self.body_frame,
                tf_body.transform, velocity)
            sensor_odom = self._odom(
                msg.header.stamp, self.map_frame, self.cloud_frame,
                tf_lidar.transform)

            # All three SCAN inputs retain exactly the original scan timestamp.
            self.sensor_pose_pub.publish(sensor_odom)
            self.body_pose_pub.publish(body_odom)
            self.cloud_pub.publish(cloud_map)

            self.last_velocity_valid = velocity_valid
            self.last_cloud_out_monotonic = time.monotonic()
            self.last_cloud_age_s = age_s
            self.last_tf_wait_s = waited_s
            self.max_tf_wait_observed_s = max(
                self.max_tf_wait_observed_s, waited_s)
            if wait_counted:
                self.counts["cloud_tf_wait_completed"] += 1
            self.counts["cloud_out"] += 1

    def publish_diagnostics(self):
        now_monotonic = time.monotonic()
        status_fresh = self._status_is_fresh()
        cloud_output_fresh = (
            self.last_cloud_out_monotonic > 0.0
            and now_monotonic - self.last_cloud_out_monotonic <= self.max_cloud_age
        )
        oldest_pending_wait_s = (
            now_monotonic - self.pending_clouds[0][1]
            if self.pending_clouds else 0.0
        )
        data = {
            "schema": "n01.live_scan_readiness.v2",
            "status": (
                "LIVE_SCAN_INPUTS_READY"
                if status_fresh and cloud_output_fresh
                else "WAITING_OR_BLOCKED"
            ),
            "localization_status": self.status_reason,
            "localization_status_fresh": bool(status_fresh),
            "cloud_output_fresh": bool(cloud_output_fresh),
            "vendor_imu_sync_ok": bool(self.status_imu_sync),
            "fitness_mse": self.status_fitness if math.isfinite(self.status_fitness) else None,
            "velocity_estimate_valid": bool(self.last_velocity_valid),
            "input_cloud_topic": self.cloud_topic,
            "input_cloud_frame": self.cloud_frame,
            "output_cloud_frame": self.map_frame,
            "outputs": ["/cloud", "/sensor_pose", "/body_pose"],
            "pending_clouds": len(self.pending_clouds),
            "oldest_pending_wait_s": oldest_pending_wait_s,
            "last_cloud_age_s": self.last_cloud_age_s,
            "last_tf_wait_s": self.last_tf_wait_s,
            "max_tf_wait_observed_s": self.max_tf_wait_observed_s,
            "max_tf_wait_s": self.max_tf_wait,
            "max_cloud_age_s": self.max_cloud_age,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
            **self.counts,
        }
        self.diag_pub.publish(String(data=json.dumps(data, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = LiveScanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
