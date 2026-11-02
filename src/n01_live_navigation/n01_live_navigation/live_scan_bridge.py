"""Live Hive2 localization + LiDAR adapter for native SCAN.

The bridge never invents a transform or rewrites a sensor timestamp. A bounded
FIFO waits for scan-time TF; expired scans are dropped before they can enter SCAN.
"""
import json
import math
import time
from collections import deque

import numpy as np
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

from .core import (
    localization_is_tracking, estimate_velocity,
    decode_pointcloud_xyz, transform_points,
)


class LiveScanBridge(Node):
    def __init__(self):
        super().__init__("n01_live_scan_bridge")
        defaults = {
            "status_topic": "/localization_status",
            "status_type": "hivecore_slam3d/msg/LocalizationStatus",
            "cloud_topic": "/livox/lidar/geo_filtered",
            "expected_cloud_frame": "lidar_link",
            "map_frame": "map",
            "body_frame": "base_link",
            "max_status_age_s": 1.0,
            "max_cloud_age_s": 0.6,
            "max_tf_wait_s": 0.45,
            "tf_retry_period_s": 0.02,
            "max_pending_clouds": 8,
            "max_points": 100000,
            "max_point_range_m": 12.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.status_topic = str(get("status_topic"))
        self.cloud_topic = str(get("cloud_topic"))
        self.cloud_frame = str(get("expected_cloud_frame")).lstrip("/")
        self.map_frame = str(get("map_frame")).lstrip("/")
        self.body_frame = str(get("body_frame")).lstrip("/")
        self.max_status_age = float(get("max_status_age_s"))
        self.max_cloud_age = float(get("max_cloud_age_s"))
        self.max_tf_wait = float(get("max_tf_wait_s"))
        self.tf_retry_period = float(get("tf_retry_period_s"))
        self.max_pending_clouds = int(get("max_pending_clouds"))
        self.max_points = int(get("max_points"))
        self.max_range = float(get("max_point_range_m"))
        if not (
            0.1 <= self.max_status_age <= 3.0
            and 0.1 <= self.max_cloud_age <= 2.0
            and 0.01 <= self.max_tf_wait <= 1.0
            and 0.005 <= self.tf_retry_period <= 0.1
            and 1 <= self.max_pending_clouds <= 32
            and 100 <= self.max_points <= 300000
            and 1.0 <= self.max_range <= 50.0
            and self.map_frame and self.body_frame and self.cloud_frame
        ):
            raise ValueError("live bridge parameters outside bounded limits")

        self.status_class = get_message(str(get("status_type")))
        self.status_ok = False
        self.status_reason = "waiting_localization_status"
        self.status_rx_monotonic = 0.0
        self.status_imu_sync = False
        self.status_fitness = float("nan")
        self.last_body = None
        self.last_velocity_valid = False

        self.pending_clouds = deque()
        self.last_cloud_out_monotonic = 0.0
        self.last_cloud_age_s = None
        self.last_tf_wait_s = 0.0
        self.max_tf_wait_observed_s = 0.0
        self.last_tf_error = ""

        self.counts = {
            "cloud_in": 0, "cloud_queued": 0, "cloud_out": 0,
            "cloud_tf_wait_started": 0, "cloud_tf_wait_completed": 0,
            "drop_status": 0, "drop_frame": 0, "drop_stamp": 0,
            "drop_tf": 0, "drop_tf_timeout": 0,
            "drop_stale_after_wait": 0, "drop_queue_overflow": 0,
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
        self.create_timer(0.5, self.publish_diagnostics)
        self.get_logger().warning(
            "READ-ONLY bridge: scan-time TF + exact-stamp SCAN inputs; no actuator publishers")

    def on_status(self, msg):
        ok, reason = localization_is_tracking(msg)
        self.status_ok = ok
        self.status_reason = reason
        self.status_rx_monotonic = time.monotonic()
        self.status_imu_sync = bool(getattr(msg, "fusion_path_imu_sync_ok", False))
        self.status_fitness = float(
            getattr(msg, "registration_output_fitness_mse", float("nan")))

    def _status_is_fresh(self):
        return (
            self.status_ok and self.status_rx_monotonic > 0.0
            and time.monotonic() - self.status_rx_monotonic <= self.max_status_age
        )

    @staticmethod
    def _transform_values(transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

    def _xyz_map_cloud(self, msg, transform):
        translation, quaternion = self._transform_values(transform)
        points = decode_pointcloud_xyz(msg, self.max_points, self.max_range)
        world = transform_points(points, translation, quaternion)
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.header.frame_id = self.map_frame
        cloud.height = 1
        cloud.width = int(world.shape[0])
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * cloud.width
        cloud.is_dense = True
        cloud.data = np.asarray(world, dtype="<f4").tobytes(order="C")
        return cloud

    @staticmethod
    def _odom(stamp, parent, child, transform, velocity=(0.0, 0.0, 0.0)):
        t = transform.transform.translation
        q = transform.transform.rotation
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = t.x, t.y, t.z
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = q.x, q.y, q.z, q.w
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
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns <= 0 or age_s > self.max_cloud_age or age_s < -0.2:
            self.counts["drop_stamp"] += 1
            return
        if len(self.pending_clouds) >= self.max_pending_clouds:
            self.pending_clouds.popleft()
            self.counts["drop_queue_overflow"] += 1
        self.pending_clouds.append((msg, time.monotonic(), False))
        self.counts["cloud_queued"] += 1

    def process_pending_clouds(self):
        # FIFO is intentional: a newer scan may not overtake an older pending scan.
        while self.pending_clouds:
            msg, enqueued, wait_counted = self.pending_clouds[0]
            stamp_ns = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
            age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            waited_s = time.monotonic() - enqueued
            if age_s > self.max_cloud_age:
                self.pending_clouds.popleft()
                self.counts["drop_stale_after_wait"] += 1
                continue
            if not self._status_is_fresh():
                self.pending_clouds.popleft()
                self.counts["drop_status"] += 1
                continue
            try:
                scan_time = Time.from_msg(msg.header.stamp)
                tf_lidar = self.tf_buffer.lookup_transform(
                    self.map_frame, self.cloud_frame, scan_time,
                    timeout=Duration(seconds=0.0))
                tf_body = self.tf_buffer.lookup_transform(
                    self.map_frame, self.body_frame, scan_time,
                    timeout=Duration(seconds=0.0))
            except TransformException as exc:
                self.last_tf_error = str(exc)
                if not wait_counted:
                    self.counts["cloud_tf_wait_started"] += 1
                    self.pending_clouds[0] = (msg, enqueued, True)
                if waited_s >= self.max_tf_wait:
                    self.pending_clouds.popleft()
                    self.counts["drop_tf"] += 1
                    self.counts["drop_tf_timeout"] += 1
                    self.last_tf_wait_s = waited_s
                    self.max_tf_wait_observed_s = max(self.max_tf_wait_observed_s, waited_s)
                    continue
                break

            try:
                cloud_map = self._xyz_map_cloud(msg, tf_lidar)
            except (ValueError, OverflowError) as exc:
                self.pending_clouds.popleft()
                self.counts["drop_cloud"] += 1
                self.get_logger().warning("drop cloud: invalid XYZ payload: %s" % exc)
                continue

            # Enforce freshness again after point decoding/transformation.
            output_age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if output_age_s > self.max_cloud_age:
                self.pending_clouds.popleft()
                self.counts["drop_stale_after_wait"] += 1
                continue

            body_xyz = (
                tf_body.transform.translation.x,
                tf_body.transform.translation.y,
                tf_body.transform.translation.z,
            )
            velocity, velocity_valid = estimate_velocity(
                self.last_body, (body_xyz, stamp_ns / 1e9))
            self.last_body = (body_xyz, stamp_ns / 1e9)
            body_odom = self._odom(
                msg.header.stamp, self.map_frame, self.body_frame, tf_body, velocity)
            sensor_odom = self._odom(
                msg.header.stamp, self.map_frame, self.cloud_frame, tf_lidar)

            # All three messages have the exact original scan timestamp.
            self.sensor_pose_pub.publish(sensor_odom)
            self.body_pose_pub.publish(body_odom)
            self.cloud_pub.publish(cloud_map)
            self.pending_clouds.popleft()

            self.last_velocity_valid = velocity_valid
            self.last_cloud_out_monotonic = time.monotonic()
            self.last_cloud_age_s = output_age_s
            self.last_tf_wait_s = waited_s
            self.max_tf_wait_observed_s = max(self.max_tf_wait_observed_s, waited_s)
            if wait_counted:
                self.counts["cloud_tf_wait_completed"] += 1
            self.counts["cloud_out"] += 1

    def publish_diagnostics(self):
        now_mono = time.monotonic()
        status_fresh = self._status_is_fresh()
        output_age = None
        if self.last_cloud_out_monotonic > 0.0 and self.last_cloud_age_s is not None:
            output_age = max(
                0.0, self.last_cloud_age_s + now_mono - self.last_cloud_out_monotonic)
        cloud_fresh = output_age is not None and output_age <= self.max_cloud_age
        pending_age = now_mono - self.pending_clouds[0][1] if self.pending_clouds else 0.0
        data = {
            "schema": "n01.live_scan_readiness.v3",
            "status": "LIVE_SCAN_INPUTS_READY" if status_fresh and cloud_fresh else "WAITING_OR_BLOCKED",
            "localization_status": self.status_reason,
            "localization_status_fresh": bool(status_fresh),
            "cloud_output_fresh": bool(cloud_fresh),
            "cloud_output_age_s": output_age,
            "vendor_imu_sync_ok": bool(self.status_imu_sync),
            "fitness_mse": self.status_fitness if math.isfinite(self.status_fitness) else None,
            "velocity_estimate_valid": bool(self.last_velocity_valid),
            "input_cloud_topic": self.cloud_topic,
            "input_cloud_frame": self.cloud_frame,
            "output_cloud_frame": self.map_frame,
            "outputs": ["/cloud", "/sensor_pose", "/body_pose"],
            "pending_clouds": len(self.pending_clouds),
            "oldest_pending_wait_s": pending_age,
            "last_cloud_age_s": self.last_cloud_age_s,
            "last_tf_wait_s": self.last_tf_wait_s,
            "max_tf_wait_observed_s": self.max_tf_wait_observed_s,
            "max_tf_wait_s": self.max_tf_wait,
            "max_cloud_age_s": self.max_cloud_age,
            "last_tf_error": self.last_tf_error,
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
