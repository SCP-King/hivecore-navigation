"""Backend-neutral localization + LiDAR bridge for native SCAN.

Global robot pose comes from a timestamped /robot_pose Odometry contract.
The bridge interpolates that pose at the original LiDAR stamp and composes it
with the robot-description sensor extrinsic (body_frame <- lidar_frame).
It never substitutes latest pose/TF or rewrites the scan timestamp.
"""
import json
import math
import time
from bisect import bisect_left
from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray

from .core import (
    localization_is_tracking,
    estimate_velocity,
    decode_pointcloud_xyz,
    transform_points,
    normalize_quaternion,
    compose_transform,
    interpolate_pose,
    motion_compensated_pose,
    scan_output_is_fresh,
    select_pose_anchor,
    select_latest_tf_stamp,
    pose_samples_equivalent,
)


class LiveScanBridge(Node):
    def __init__(self):
        super().__init__("n01_live_scan_bridge")
        defaults = {
            "status_topic": "/n01/localization/status",
            "cloud_topic": "/livox/lidar/geo_filtered",
            "pose_topic": "/n01/localization/pose",
            "odom_topic": "/odom",
            "odom_frame": "odom",
            "odom_child_frame": "base_link",
            "expected_cloud_frame": "lidar_link",
            "map_frame": "map",
            # Input /robot_pose contract stays map -> base_footprint (vendor SLAM).
            "body_frame": "base_footprint",
            # /body_pose output consumed by SCAN/tracker is the physical body,
            # base_footprint -> base_link +0.90 is composed once here.
            "planner_body_frame": "base_link",
            "max_status_age_s": 1.0,
            "max_pose_rx_age_s": 0.7,
            "max_pose_stamp_age_s": 1.5,
            "max_pose_anchor_delta_s": 1.25,
            "max_live_body_pose_anchor_delta_s": 1.4,
            "pose_cache_size": 64,
            "max_odom_rx_age_s": 0.2,
            "max_odom_stamp_age_s": 0.4,
            "odom_interpolation_max_gap_s": 0.08,
            "odom_cache_size": 512,
            "max_cloud_age_s": 0.9,
            "max_latest_scan_lag_s": 0.7,
            "body_pose_publish_period_s": 0.05,
            "max_live_body_pose_age_s": 0.25,
            "max_live_tf_lag_s": 0.15,
            "max_tf_wait_s": 0.55,
            "tf_retry_period_s": 0.02,
            "max_pending_clouds": 1,
            "max_points": 100000,
            "max_point_range_m": 12.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.status_topic = str(get("status_topic"))
        self.cloud_topic = str(get("cloud_topic"))
        self.pose_topic = str(get("pose_topic"))
        self.odom_topic = str(get("odom_topic"))
        self.odom_frame = str(get("odom_frame")).lstrip("/")
        self.odom_child_frame = str(get("odom_child_frame")).lstrip("/")
        self.cloud_frame = str(get("expected_cloud_frame")).lstrip("/")
        self.map_frame = str(get("map_frame")).lstrip("/")
        self.body_frame = str(get("body_frame")).lstrip("/")
        self.planner_body_frame = str(get("planner_body_frame")).lstrip("/")
        self.max_status_age = float(get("max_status_age_s"))
        self.max_pose_rx_age = float(get("max_pose_rx_age_s"))
        self.max_pose_stamp_age = float(get("max_pose_stamp_age_s"))
        self.max_pose_anchor_delta = float(get("max_pose_anchor_delta_s"))
        self.max_live_body_pose_anchor_delta = float(get("max_live_body_pose_anchor_delta_s"))
        self.pose_cache_size = int(get("pose_cache_size"))
        self.max_odom_rx_age = float(get("max_odom_rx_age_s"))
        self.max_odom_stamp_age = float(get("max_odom_stamp_age_s"))
        self.odom_interpolation_max_gap = float(get("odom_interpolation_max_gap_s"))
        self.odom_cache_size = int(get("odom_cache_size"))
        self.max_cloud_age = float(get("max_cloud_age_s"))
        self.max_latest_scan_lag = float(get("max_latest_scan_lag_s"))
        self.body_pose_publish_period = float(get("body_pose_publish_period_s"))
        self.max_live_body_pose_age = float(get("max_live_body_pose_age_s"))
        self.max_live_tf_lag = float(get("max_live_tf_lag_s"))
        self.max_tf_wait = float(get("max_tf_wait_s"))
        self.tf_retry_period = float(get("tf_retry_period_s"))
        self.max_pending_clouds = int(get("max_pending_clouds"))
        self.max_points = int(get("max_points"))
        self.max_range = float(get("max_point_range_m"))
        if not (
            0.1 <= self.max_status_age <= 3.0
            and 0.1 <= self.max_pose_rx_age <= 2.0
            and 0.1 <= self.max_pose_stamp_age <= 2.0
            and 0.1 <= self.max_pose_anchor_delta <= 1.5
            and 0.1 <= self.max_live_body_pose_anchor_delta <= self.max_pose_stamp_age
            and 2 <= self.pose_cache_size <= 256
            and 0.05 <= self.max_odom_rx_age <= 1.0
            and 0.1 <= self.max_odom_stamp_age <= 2.0
            and 0.02 <= self.odom_interpolation_max_gap <= 0.2
            and 16 <= self.odom_cache_size <= 2048
            and 0.1 <= self.max_cloud_age <= 2.0
            and 0.05 <= self.max_latest_scan_lag <= 1.0
            and 0.02 <= self.body_pose_publish_period <= 0.2
            and 0.1 <= self.max_live_body_pose_age <= 1.0
            and 0.02 <= self.max_live_tf_lag <= 0.5
            and 0.01 <= self.max_tf_wait <= 1.0
            and 0.005 <= self.tf_retry_period <= 0.1
            and 1 <= self.max_pending_clouds <= 32
            and 100 <= self.max_points <= 300000
            and 1.0 <= self.max_range <= 50.0
            and self.map_frame and self.body_frame and self.planner_body_frame
            and self.cloud_frame
            and self.odom_frame and self.odom_child_frame
        ):
            raise ValueError("live bridge parameters outside bounded limits")

        self.status_ok = False
        self.status_reason = "waiting_localization_status"
        self.status_rx_monotonic = 0.0
        self.status_stamp_ns = 0
        self.status_stamp_age_s = None
        self.status_imu_sync = False
        self.status_fitness = float("nan")

        # Samples are (stamp_ns, map_position_xyz, map_orientation_xyzw).
        # A bounded sorted cache permits interpolation without latest-pose reuse.
        self.pose_samples = []
        self.pose_rx_monotonic = 0.0
        self.pose_stamp_age_s = None
        self.pose_contract_error = "waiting_robot_pose"
        self.last_pose_error = ""
        self.last_pose_stamp_ns = None
        self.pose_conflict_stamps = set()
        self.pose_publisher_endpoints = []
        self.pose_publisher_query_error = ""
        self.pose_publisher_query_monotonic = 0.0
        self.last_body = None
        self.last_velocity_valid = False
        self.last_live_body = None
        self.last_live_body_pose_stamp_ns = 0
        self.last_live_body_pose_monotonic = 0.0
        self.last_live_body_pose_anchor_stamp_ns = None
        self.last_live_body_pose_anchor_delta_s = None
        self.last_live_body_tf_lag_s = None
        self.live_body_tf_latest_fallback_count = 0
        self.last_live_body_pose_error = "waiting_global_anchor"
        self.last_live_body_velocity_valid = False

        # High-rate local odometry cache; poses are odom<-base_link.
        # At query time TF converts base_link to the global-pose body frame.
        self.odom_samples = []
        self.odom_rx_monotonic = 0.0
        self.odom_stamp_age_s = None
        self.odom_contract_error = "waiting_odom"
        self.last_odom_stamp_ns = None
        self.last_odom_error = ""
        self.last_motion_compensation_error = "waiting_odom"
        self.last_motion_compensation = {}
        self.motion_compensation_ready = False

        self.pending_clouds = deque()
        self.last_cloud_out_monotonic = 0.0
        self.last_cloud_age_s = None
        self.last_cloud_input_stamp_ns = None
        self.last_cloud_input_rx_monotonic = 0.0
        self.last_processed_scan_stamp_ns = 0
        self.last_tf_wait_s = 0.0
        self.max_tf_wait_observed_s = 0.0
        self.last_tf_error = ""
        self.sensor_extrinsic_ready = False

        self.counts = {
            "cloud_in": 0, "cloud_queued": 0, "cloud_out": 0,
            "cloud_tf_wait_started": 0, "cloud_tf_wait_completed": 0,
            "pose_in": 0, "pose_accepted": 0, "pose_stamp_advance": 0,
            "pose_duplicate": 0, "pose_out_of_order": 0,
            "drop_pose_duplicate_conflict": 0,
            "odom_in": 0, "odom_accepted": 0, "odom_reordered": 0,
            "drop_odom_contract": 0, "drop_odom_timeout": 0, "drop_odom_tf": 0,
            "drop_pose_contract": 0, "drop_pose_timeout": 0,
            "drop_tf": 0, "drop_tf_timeout": 0,
            "drop_status": 0, "drop_frame": 0, "drop_stamp": 0,
            "drop_stale_after_wait": 0, "drop_queue_overflow": 0,
            "drop_superseded": 0, "drop_out_of_order": 0,
            "drop_cloud": 0, "body_pose_live_out": 0,
        }

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.status_sub = self.create_subscription(
            DiagnosticArray, self.status_topic, self.on_status, qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(
            Odometry, self.pose_topic, self.on_robot_pose, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.on_odometry, qos_profile_sensor_data)
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self.on_cloud, qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, "/cloud", qos_profile_sensor_data)
        self.sensor_pose_pub = self.create_publisher(
            Odometry, "/sensor_pose", qos_profile_sensor_data)
        # /body_pose is the high-rate current body pose used to slide SCAN's map.
        # The exact scan-time body pose is retained on a separate diagnostic topic.
        self.body_pose_pub = self.create_publisher(
            Odometry, "/body_pose", qos_profile_sensor_data)
        self.scan_body_pose_pub = self.create_publisher(
            Odometry, "/n01/live_navigation/scan_body_pose", qos_profile_sensor_data)
        self.diag_pub = self.create_publisher(
            String, "/n01/live_navigation/readiness", 10)

        self.create_timer(self.tf_retry_period, self.process_pending_clouds)
        self.create_timer(0.5, self.publish_diagnostics)
        self.get_logger().warning(
            "READ-ONLY bridge: normalized N-01 pose anchors + timestamped /odom relative "
            "motion compensation + body-to-LiDAR extrinsic; no actuator publishers")

    @staticmethod
    def _stamp_ns(stamp):
        return int(stamp.sec) * 1000000000 + int(stamp.nanosec)

    def on_status(self, msg):
        # Backend-neutral contract: DiagnosticArray status named
        # "localization" with stable N-01 key/value fields.
        normalized = {}
        for item in msg.status:
            if item.name == "localization":
                normalized = {entry.key: entry.value for entry in item.values}
                break
        ok, reason = localization_is_tracking(normalized)
        self.status_ok = ok
        self.status_reason = reason
        self.status_rx_monotonic = time.monotonic()
        self.status_stamp_ns = self._stamp_ns(msg.header.stamp)
        if self.status_stamp_ns > 0:
            self.status_stamp_age_s = (
                self.get_clock().now().nanoseconds - self.status_stamp_ns) / 1e9
        else:
            self.status_stamp_age_s = None
        self.status_imu_sync = normalized.get("imu_sync_ok", "false").lower() == "true"
        try:
            self.status_fitness = float(normalized.get("fitness_mse", "nan"))
        except (TypeError, ValueError):
            self.status_fitness = float("nan")

    def _status_is_fresh(self):
        if not (
            self.status_ok and self.status_rx_monotonic > 0.0
            and time.monotonic() - self.status_rx_monotonic <= self.max_status_age
            and self.status_stamp_ns > 0
        ):
            return False
        self.status_stamp_age_s = (
            self.get_clock().now().nanoseconds - self.status_stamp_ns) / 1e9
        return -0.2 <= self.status_stamp_age_s <= self.max_status_age

    def _pose_rx_is_fresh(self):
        if not (
            self.pose_rx_monotonic > 0.0
            and time.monotonic() - self.pose_rx_monotonic <= self.max_pose_rx_age
            and self.pose_samples
        ):
            return False
        latest_pose_stamp_ns = self.pose_samples[-1][0]
        self.pose_stamp_age_s = (
            self.get_clock().now().nanoseconds - latest_pose_stamp_ns) / 1e9
        return -0.2 <= self.pose_stamp_age_s <= self.max_pose_stamp_age

    def _odom_rx_is_fresh(self):
        if not (
            self.odom_rx_monotonic > 0.0
            and time.monotonic() - self.odom_rx_monotonic <= self.max_odom_rx_age
            and self.odom_samples
        ):
            return False
        latest_stamp_ns = self.odom_samples[-1][0]
        self.odom_stamp_age_s = (
            self.get_clock().now().nanoseconds - latest_stamp_ns) / 1e9
        return -0.2 <= self.odom_stamp_age_s <= self.max_odom_stamp_age

    def on_odometry(self, msg):
        self.counts["odom_in"] += 1
        parent = str(msg.header.frame_id).lstrip("/")
        child = str(msg.child_frame_id).lstrip("/")
        if parent != self.odom_frame or child != self.odom_child_frame:
            self.counts["drop_odom_contract"] += 1
            self.odom_contract_error = (
                "odom_frame_mismatch: expected %s<- %s, received %s<- %s"
                % (self.odom_frame, self.odom_child_frame, parent, child)
            )
            return

        stamp_ns = self._stamp_ns(msg.header.stamp)
        if stamp_ns <= 0:
            self.counts["drop_odom_contract"] += 1
            self.odom_contract_error = "odom_stamp_invalid"
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        position = (float(p.x), float(p.y), float(p.z))
        try:
            orientation = normalize_quaternion(
                (float(q.x), float(q.y), float(q.z), float(q.w)))
        except ValueError as exc:
            self.counts["drop_odom_contract"] += 1
            self.odom_contract_error = str(exc)
            return
        if not all(math.isfinite(v) for v in position):
            self.counts["drop_odom_contract"] += 1
            self.odom_contract_error = "odom_position_nonfinite"
            return

        stamps = [sample[0] for sample in self.odom_samples]
        index = bisect_left(stamps, stamp_ns)
        if self.odom_samples and stamp_ns < self.odom_samples[-1][0]:
            self.counts["odom_reordered"] += 1
        sample = (stamp_ns, position, orientation)
        if index < len(self.odom_samples) and self.odom_samples[index][0] == stamp_ns:
            self.odom_samples[index] = sample
        else:
            self.odom_samples.insert(index, sample)
        if len(self.odom_samples) > self.odom_cache_size:
            del self.odom_samples[:len(self.odom_samples) - self.odom_cache_size]
        self.odom_rx_monotonic = time.monotonic()
        self.odom_contract_error = ""
        self.last_odom_stamp_ns = self.odom_samples[-1][0]
        self.odom_stamp_age_s = (
            self.get_clock().now().nanoseconds - self.last_odom_stamp_ns) / 1e9
        self.counts["odom_accepted"] += 1
        self._publish_live_body_pose()

    def on_robot_pose(self, msg):
        self.counts["pose_in"] += 1
        parent = str(msg.header.frame_id).lstrip("/")
        child = str(msg.child_frame_id).lstrip("/")
        if parent != self.map_frame or child != self.body_frame:
            self.counts["drop_pose_contract"] += 1
            self.pose_contract_error = (
                "robot_pose_frame_mismatch: expected %s<- %s, received %s<- %s"
                % (self.map_frame, self.body_frame, parent, child)
            )
            return
        stamp_ns = (
            int(msg.header.stamp.sec) * 1000000000
            + int(msg.header.stamp.nanosec)
        )
        if stamp_ns <= 0:
            self.counts["drop_pose_contract"] += 1
            self.pose_contract_error = "robot_pose_stamp_invalid"
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        position = (float(p.x), float(p.y), float(p.z))
        try:
            orientation = normalize_quaternion(
                (float(q.x), float(q.y), float(q.z), float(q.w)))
        except ValueError as exc:
            self.counts["drop_pose_contract"] += 1
            self.pose_contract_error = str(exc)
            return
        if not all(math.isfinite(v) for v in position):
            self.counts["drop_pose_contract"] += 1
            self.pose_contract_error = "robot_pose_position_nonfinite"
            return

        if stamp_ns in self.pose_conflict_stamps:
            self.counts["drop_pose_duplicate_conflict"] += 1
            return

        stamps = [sample[0] for sample in self.pose_samples]
        index = bisect_left(stamps, stamp_ns)
        sample = (stamp_ns, position, orientation)
        if index < len(self.pose_samples) and self.pose_samples[index][0] == stamp_ns:
            existing = self.pose_samples[index]
            if pose_samples_equivalent(existing, position, orientation):
                # Duplicate endpoints can legitimately repeat an identical sample.
                # Do not treat a duplicate delivery as forward timestamp progress.
                self.counts["pose_duplicate"] += 1
                return
            # The same source timestamp carrying materially different poses is
            # ambiguous. Remove that anchor and reject further copies of it.
            conflict_was_latest = index == len(self.pose_samples) - 1
            self.pose_samples.pop(index)
            self.pose_conflict_stamps.add(stamp_ns)
            if len(self.pose_conflict_stamps) > 64:
                self.pose_conflict_stamps.remove(min(self.pose_conflict_stamps))
            self.counts["drop_pose_duplicate_conflict"] += 1
            self.pose_contract_error = "robot_pose_duplicate_stamp_conflict"
            self.last_pose_error = self.pose_contract_error
            self.last_pose_stamp_ns = (
                self.pose_samples[-1][0] if self.pose_samples else None)
            if conflict_was_latest:
                # A conflicting newest anchor invalidates pose freshness until
                # a strictly newer, unambiguous timestamp arrives.
                self.pose_rx_monotonic = 0.0
            return

        is_newest_stamp = not self.pose_samples or stamp_ns > self.pose_samples[-1][0]
        if not is_newest_stamp:
            self.counts["pose_out_of_order"] += 1
        self.pose_samples.insert(index, sample)
        if len(self.pose_samples) > self.pose_cache_size:
            del self.pose_samples[:len(self.pose_samples) - self.pose_cache_size]
        self.counts["pose_accepted"] += 1
        if is_newest_stamp:
            # Only a strictly advancing source timestamp refreshes pose freshness.
            # Re-delivery of an old sample must not mask a stalled SLAM pose stream.
            self.pose_rx_monotonic = time.monotonic()
            self.pose_contract_error = ""
            self.last_pose_stamp_ns = stamp_ns
            self.counts["pose_stamp_advance"] += 1

    @staticmethod
    def _transform_values(transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

    def _pose_anchor_at(self, stamp_ns, max_delta_s=None):
        if max_delta_s is None:
            max_delta_s = self.max_pose_anchor_delta
        return select_pose_anchor(self.pose_samples, stamp_ns, max_delta_s)

    def _odom_at(self, stamp_ns):
        if not self._odom_rx_is_fresh():
            self.last_odom_error = "odom_receive_or_stamp_stale"
            return None
        try:
            pose = interpolate_pose(
                self.odom_samples, stamp_ns, self.odom_interpolation_max_gap)
        except ValueError as exc:
            self.last_odom_error = str(exc)
            return None
        if pose is None:
            self.last_odom_error = "odom_not_bracketed_at_requested_stamp"
            return None
        self.last_odom_error = ""
        return pose

    def _odom_body_pose_at(self, stamp_ns):
        odom_child_pose = self._odom_at(stamp_ns)
        if odom_child_pose is None:
            return None
        if self.odom_child_frame == self.body_frame:
            return odom_child_pose
        try:
            child_from_body = self.tf_buffer.lookup_transform(
                self.odom_child_frame, self.body_frame,
                Time(nanoseconds=int(stamp_ns)),
                timeout=Duration(seconds=0.0))
            child_body_translation, child_body_quaternion = self._transform_values(
                child_from_body)
            return compose_transform(
                odom_child_pose[0], odom_child_pose[1],
                child_body_translation, child_body_quaternion)
        except (TransformException, ValueError) as exc:
            self.counts["drop_odom_tf"] += 1
            self.last_odom_error = "odom_body_frame_tf_unavailable: %s" % exc
            return None

    def _odom_body_pose_latest(self, requested_stamp_ns):
        """Resolve live body pose at latest available TF time, with bounded lag.

        This path is only for the continuously refreshed /body_pose output.
        Scan-time deskew remains strict and continues to query the exact scan
        timestamp in _odom_body_pose_at/_pose_at.
        """
        if self.odom_child_frame == self.body_frame:
            pose = self._odom_at(requested_stamp_ns)
            return (pose, int(requested_stamp_ns)) if pose is not None else None
        try:
            child_from_body = self.tf_buffer.lookup_transform(
                self.odom_child_frame, self.body_frame, Time(),
                timeout=Duration(seconds=0.0))
            tf_stamp_ns = self._stamp_ns(child_from_body.header.stamp)
            effective_stamp_ns = select_latest_tf_stamp(
                requested_stamp_ns, tf_stamp_ns, self.max_live_tf_lag)
            if effective_stamp_ns is None:
                lag = (
                    (int(requested_stamp_ns) - int(tf_stamp_ns)) / 1e9
                    if tf_stamp_ns > 0 else None
                )
                self.last_odom_error = (
                    "live_body_tf_outside_lag_budget: requested=%s tf=%s lag_s=%s max_s=%.3f"
                    % (requested_stamp_ns, tf_stamp_ns, lag, self.max_live_tf_lag)
                )
                return None
            # Crucial: interpolate odometry at the TF sample's real timestamp.
            # Never stamp a delayed transform as if it belonged to a newer time.
            odom_child_pose = self._odom_at(effective_stamp_ns)
            if odom_child_pose is None:
                return None
            child_body_translation, child_body_quaternion = self._transform_values(
                child_from_body)
            pose = compose_transform(
                odom_child_pose[0], odom_child_pose[1],
                child_body_translation, child_body_quaternion)
            lag_s = (
                (int(requested_stamp_ns) - int(tf_stamp_ns)) / 1e9
                if tf_stamp_ns > 0 else 0.0
            )
            self.last_live_body_tf_lag_s = max(0.0, lag_s)
            if tf_stamp_ns > 0 and effective_stamp_ns != int(requested_stamp_ns):
                self.live_body_tf_latest_fallback_count += 1
            self.last_odom_error = ""
            return pose, int(effective_stamp_ns)
        except (TransformException, ValueError) as exc:
            self.counts["drop_odom_tf"] += 1
            self.last_odom_error = "live_body_tf_latest_unavailable: %s" % exc
            return None

    def _publish_live_body_pose(self):
        """Publish current map<-body from a bounded SLAM anchor and timestamped odometry."""
        now_mono = time.monotonic()
        if now_mono - self.last_live_body_pose_monotonic < self.body_pose_publish_period:
            return
        if not self.pose_samples or not self.odom_samples:
            self.last_live_body_pose_error = "waiting_robot_pose_or_odom_samples"
            return
        if not self._pose_rx_is_fresh():
            self.last_live_body_pose_error = "robot_pose_receive_or_stamp_stale"
            return
        if not self._odom_rx_is_fresh():
            self.last_live_body_pose_error = "odom_receive_or_stamp_stale"
            return
        requested_stamp_ns = int(self.odom_samples[-1][0])
        latest_body = self._odom_body_pose_latest(requested_stamp_ns)
        if latest_body is None:
            self.last_live_body_pose_error = self.last_odom_error or "odom_current_unavailable"
            return
        odom_current, stamp_ns = latest_body
        if stamp_ns <= self.last_live_body_pose_stamp_ns:
            return
        # The live display pose follows the newest available TF sample, not a
        # future odometry stamp. A bounded 150 ms lag accommodates real TF
        # publication latency while preserving the transform's true timestamp.
        # Scan-time deskew remains exact and is not relaxed by this fallback.
        anchor = self._pose_anchor_at(
            stamp_ns, self.max_live_body_pose_anchor_delta)
        if anchor is None:
            self.last_live_body_pose_error = "robot_pose_anchor_outside_live_body_delta"
            return
        odom_anchor = self._odom_body_pose_at(anchor[0])
        if odom_anchor is None:
            self.last_live_body_pose_error = self.last_odom_error or "odom_anchor_unavailable"
            return
        try:
            map_body = motion_compensated_pose(
                (anchor[1], anchor[2]), odom_anchor, odom_current)
        except ValueError as exc:
            self.last_live_body_pose_error = "motion_compensation_invalid: %s" % exc
            return
        # The published planner body is the physical base_link, while the
        # /robot_pose input remains map<-base_footprint. Compose the fixed
        # base_footprint->base_link transform exactly once here.
        if self.planner_body_frame == self.body_frame:
            map_planner = map_body
        else:
            try:
                body_to_planner = self.tf_buffer.lookup_transform(
                    self.body_frame, self.planner_body_frame,
                    Time(nanoseconds=stamp_ns), timeout=Duration(seconds=0.05))
                body_to_planner_translation, body_to_planner_quaternion = \
                    self._transform_values(body_to_planner)
                map_planner = compose_transform(
                    map_body[0], map_body[1],
                    body_to_planner_translation, body_to_planner_quaternion)
            except (TransformException, ValueError) as exc:
                self.last_live_body_pose_error = (
                    "planner_body_tf_unavailable: %s" % exc)
                return
        velocity, velocity_valid = estimate_velocity(
            self.last_live_body, (map_planner[0], stamp_ns / 1e9))
        if not velocity_valid:
            velocity = (0.0, 0.0, 0.0)
        body_msg = self._odom(
            Time(nanoseconds=stamp_ns).to_msg(), self.map_frame,
            self.planner_body_frame,
            map_planner[0], map_planner[1], velocity)
        self.body_pose_pub.publish(body_msg)
        self.last_live_body = (map_planner[0], stamp_ns / 1e9)
        self.last_live_body_pose_stamp_ns = stamp_ns
        self.last_live_body_pose_monotonic = now_mono
        self.last_live_body_pose_anchor_stamp_ns = int(anchor[0])
        self.last_live_body_pose_anchor_delta_s = abs(stamp_ns - int(anchor[0])) / 1e9
        self.last_live_body_pose_error = ""
        self.last_live_body_velocity_valid = velocity_valid
        self.counts["body_pose_live_out"] += 1

    def _pose_at(self, stamp_ns):
        if not self._pose_rx_is_fresh():
            self.last_pose_error = "robot_pose_receive_or_stamp_stale"
            self.last_motion_compensation_error = self.last_pose_error
            self.motion_compensation_ready = False
            return None
        if not self._odom_rx_is_fresh():
            self.last_pose_error = "odom_receive_or_stamp_stale"
            self.last_motion_compensation_error = self.last_pose_error
            self.motion_compensation_ready = False
            return None

        anchor = self._pose_anchor_at(stamp_ns)
        if anchor is None:
            self.last_pose_error = "robot_pose_anchor_outside_allowed_time_delta"
            self.last_motion_compensation_error = self.last_pose_error
            self.motion_compensation_ready = False
            return None

        odom_anchor = self._odom_body_pose_at(anchor[0])
        if odom_anchor is None:
            self.last_pose_error = self.last_odom_error
            self.last_motion_compensation_error = self.last_pose_error
            self.motion_compensation_ready = False
            return None
        odom_scan = self._odom_body_pose_at(stamp_ns)
        if odom_scan is None:
            self.last_pose_error = self.last_odom_error
            self.last_motion_compensation_error = self.last_pose_error
            self.motion_compensation_ready = False
            return None

        try:
            pose = motion_compensated_pose(
                (anchor[1], anchor[2]), odom_anchor, odom_scan)
        except ValueError as exc:
            self.last_pose_error = "motion_compensation_invalid: %s" % exc
            self.last_motion_compensation_error = self.last_pose_error
            self.motion_compensation_ready = False
            return None

        self.last_pose_error = ""
        self.last_motion_compensation_error = ""
        self.last_motion_compensation = {
            "scan_stamp_ns": int(stamp_ns),
            "anchor_stamp_ns": int(anchor[0]),
            "anchor_delta_s": (int(stamp_ns) - int(anchor[0])) / 1e9,
            "anchor_abs_delta_s": abs(int(stamp_ns) - int(anchor[0])) / 1e9,
        }
        self.motion_compensation_ready = True
        return pose

    def _xyz_map_cloud(self, msg, translation, quaternion):
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
    def _odom(stamp, parent, child, translation, quaternion,
              velocity=(0.0, 0.0, 0.0)):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        (msg.pose.pose.position.x,
         msg.pose.pose.position.y,
         msg.pose.pose.position.z) = translation
        (msg.pose.pose.orientation.x,
         msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w) = quaternion
        (msg.twist.twist.linear.x,
         msg.twist.twist.linear.y,
         msg.twist.twist.linear.z) = velocity
        return msg

    def on_cloud(self, msg):
        self.counts["cloud_in"] += 1
        if msg.header.frame_id.lstrip("/") != self.cloud_frame:
            self.counts["drop_frame"] += 1
            return
        stamp_ns = self._stamp_ns(msg.header.stamp)
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns <= 0 or age_s > self.max_cloud_age or age_s < -0.2:
            self.counts["drop_stamp"] += 1
            return
        if stamp_ns <= self.last_processed_scan_stamp_ns:
            self.counts["drop_out_of_order"] += 1
            return
        if self.pending_clouds:
            pending_stamp_ns = self._stamp_ns(self.pending_clouds[-1][0].header.stamp)
            if stamp_ns <= pending_stamp_ns:
                self.counts["drop_out_of_order"] += 1
                return
        self.last_cloud_input_stamp_ns = stamp_ns
        self.last_cloud_input_rx_monotonic = time.monotonic()
        # Keep the last completed scan eligible only while its age and lag stay bounded.
        if not self._status_is_fresh():
            self.counts["drop_status"] += 1
            return
        if self.pending_clouds:
            # Do not let a blocked old scan hold up a newer scan. SCAN needs
            # the freshest obstacle observation, not a FIFO backlog.
            self.counts["drop_superseded"] += len(self.pending_clouds)
            self.pending_clouds.clear()
        self.pending_clouds.append((msg, time.monotonic(), False))
        self.counts["cloud_queued"] += 1

    def process_pending_clouds(self):
        # on_cloud maintains a latest-only queue; no stale FIFO backlog is retained.
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

            pose = self._pose_at(stamp_ns)
            if pose is None:
                if not wait_counted:
                    self.counts["cloud_tf_wait_started"] += 1
                    self.pending_clouds[0] = (msg, enqueued, True)
                if waited_s >= self.max_tf_wait:
                    self.pending_clouds.popleft()
                    if self.last_pose_error.startswith("odom_"):
                        self.counts["drop_odom_timeout"] += 1
                    else:
                        self.counts["drop_pose_timeout"] += 1
                    self.last_tf_wait_s = waited_s
                    self.max_tf_wait_observed_s = max(
                        self.max_tf_wait_observed_s, waited_s)
                    continue
                break

            try:
                scan_time = Time.from_msg(msg.header.stamp)
                # Only the rigid sensor mount is taken from TF. The global
                # map<-body pose is a SLAM anchor propagated by relative /odom motion.
                body_from_lidar = self.tf_buffer.lookup_transform(
                    self.body_frame, self.cloud_frame, scan_time,
                    timeout=Duration(seconds=0.0))
                body_lidar_translation, body_lidar_quaternion = self._transform_values(
                    body_from_lidar)
                map_body_translation, map_body_quaternion = pose
                map_lidar_translation, map_lidar_quaternion = compose_transform(
                    map_body_translation, map_body_quaternion,
                    body_lidar_translation, body_lidar_quaternion)
            except (TransformException, ValueError) as exc:
                self.last_tf_error = str(exc)
                self.sensor_extrinsic_ready = False
                if not wait_counted:
                    self.counts["cloud_tf_wait_started"] += 1
                    self.pending_clouds[0] = (msg, enqueued, True)
                if waited_s >= self.max_tf_wait:
                    self.pending_clouds.popleft()
                    self.counts["drop_tf"] += 1
                    self.counts["drop_tf_timeout"] += 1
                    self.last_tf_wait_s = waited_s
                    self.max_tf_wait_observed_s = max(
                        self.max_tf_wait_observed_s, waited_s)
                    continue
                break

            try:
                cloud_map = self._xyz_map_cloud(
                    msg, map_lidar_translation, map_lidar_quaternion)
            except (ValueError, OverflowError) as exc:
                self.pending_clouds.popleft()
                self.motion_compensation_ready = False
                self.last_motion_compensation_error = "cloud_conversion_failed: %s" % exc
                self.counts["drop_cloud"] += 1
                self.get_logger().warning("drop cloud: invalid XYZ payload: %s" % exc)
                continue

            output_age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if output_age_s > self.max_cloud_age:
                self.pending_clouds.popleft()
                self.motion_compensation_ready = False
                self.last_motion_compensation_error = "scan_stale_after_motion_compensation"
                self.counts["drop_stale_after_wait"] += 1
                continue

            body_xyz = map_body_translation
            velocity, velocity_valid = estimate_velocity(
                self.last_body, (body_xyz, stamp_ns / 1e9))
            self.last_body = (body_xyz, stamp_ns / 1e9)
            body_odom = self._odom(
                msg.header.stamp, self.map_frame, self.body_frame,
                map_body_translation, map_body_quaternion, velocity)
            sensor_odom = self._odom(
                msg.header.stamp, self.map_frame, self.cloud_frame,
                map_lidar_translation, map_lidar_quaternion)

            # All outputs retain the exact original LiDAR scan timestamp.
            self.sensor_pose_pub.publish(sensor_odom)
            self.scan_body_pose_pub.publish(body_odom)
            self.cloud_pub.publish(cloud_map)
            self.pending_clouds.popleft()

            self.sensor_extrinsic_ready = True
            self.last_tf_error = ""
            self.last_velocity_valid = velocity_valid
            self.last_cloud_out_monotonic = time.monotonic()
            self.last_cloud_age_s = output_age_s
            self.last_tf_wait_s = waited_s
            self.max_tf_wait_observed_s = max(
                self.max_tf_wait_observed_s, waited_s)
            if wait_counted:
                self.counts["cloud_tf_wait_completed"] += 1
            self.last_processed_scan_stamp_ns = stamp_ns
            self.counts["cloud_out"] += 1

    def _refresh_pose_publisher_endpoints(self, now_mono):
        if now_mono - self.pose_publisher_query_monotonic < 2.0:
            return
        self.pose_publisher_query_monotonic = now_mono
        try:
            endpoints = self.get_publishers_info_by_topic(self.pose_topic)
            self.pose_publisher_endpoints = []
            for endpoint in endpoints:
                try:
                    gid = bytes(endpoint.endpoint_gid).hex()
                except Exception:
                    gid = str(getattr(endpoint, "endpoint_gid", ""))
                namespace = str(endpoint.node_namespace).rstrip("/")
                node_name = str(endpoint.node_name)
                self.pose_publisher_endpoints.append({
                    "node": (namespace + "/" + node_name) if namespace else "/" + node_name,
                    "type": str(endpoint.topic_type),
                    "gid": gid,
                })
            self.pose_publisher_query_error = ""
        except Exception as exc:
            self.pose_publisher_query_error = str(exc)

    def publish_diagnostics(self):
        now_mono = time.monotonic()
        self._refresh_pose_publisher_endpoints(now_mono)
        now_ns = self.get_clock().now().nanoseconds
        status_fresh = self._status_is_fresh()
        pose_fresh = self._pose_rx_is_fresh()
        output_age = None
        if self.last_cloud_out_monotonic > 0.0 and self.last_cloud_age_s is not None:
            output_age = max(
                0.0, self.last_cloud_age_s + now_mono - self.last_cloud_out_monotonic)
        cloud_fresh = output_age is not None and output_age <= self.max_cloud_age
        pending_age = now_mono - self.pending_clouds[0][1] if self.pending_clouds else 0.0

        cloud_input_age = None
        if self.last_cloud_input_stamp_ns is not None:
            cloud_input_age = (now_ns - self.last_cloud_input_stamp_ns) / 1e9
        cloud_input_rx_age = None
        if self.last_cloud_input_rx_monotonic > 0.0:
            cloud_input_rx_age = now_mono - self.last_cloud_input_rx_monotonic

        latest_pose_stamp_ns = self.pose_samples[-1][0] if self.pose_samples else None
        oldest_pose_stamp_ns = self.pose_samples[0][0] if self.pose_samples else None
        pose_stamp_progress_age = (
            now_mono - self.pose_rx_monotonic
            if self.pose_rx_monotonic > 0.0 else None)
        latest_odom_stamp_ns = self.odom_samples[-1][0] if self.odom_samples else None
        oldest_odom_stamp_ns = self.odom_samples[0][0] if self.odom_samples else None
        odom_fresh = self._odom_rx_is_fresh()
        pose_scan_skew = None
        pose_scan_before = None
        pose_scan_after = None
        pose_scan_bracket_gap = None
        pose_scan_bracket_available = False
        if self.last_cloud_input_stamp_ns is not None and self.pose_samples:
            scan_ns = self.last_cloud_input_stamp_ns
            pose_scan_skew = (scan_ns - latest_pose_stamp_ns) / 1e9
            before = [sample[0] for sample in self.pose_samples if sample[0] <= scan_ns]
            after = [sample[0] for sample in self.pose_samples if sample[0] >= scan_ns]
            if before:
                pose_scan_before = (scan_ns - before[-1]) / 1e9
            if after:
                pose_scan_after = (after[0] - scan_ns) / 1e9
            if before and after:
                pose_scan_bracket_gap = (after[0] - before[-1]) / 1e9
                pose_scan_bracket_available = (
                    pose_scan_bracket_gap <= self.max_pose_anchor_delta)

        pose_scan_anchor_available = (
            self._pose_anchor_at(self.last_cloud_input_stamp_ns) is not None
            if self.last_cloud_input_stamp_ns is not None else False)

        odom_scan_skew = None
        odom_scan_bracket_gap = None
        odom_scan_bracket_available = False
        if self.last_cloud_input_stamp_ns is not None and self.odom_samples:
            scan_ns = self.last_cloud_input_stamp_ns
            odom_before = [sample[0] for sample in self.odom_samples if sample[0] <= scan_ns]
            odom_after = [sample[0] for sample in self.odom_samples if sample[0] >= scan_ns]
            odom_scan_skew = (scan_ns - latest_odom_stamp_ns) / 1e9
            if odom_before and odom_after:
                odom_scan_bracket_gap = (odom_after[0] - odom_before[-1]) / 1e9
                odom_scan_bracket_available = (
                    odom_scan_bracket_gap <= self.odom_interpolation_max_gap)

        compensation_for_processed_scan = (
            self.last_motion_compensation
            if self.last_motion_compensation.get("scan_stamp_ns") == self.last_processed_scan_stamp_ns
            else {})
        live_body_pose_age = (
            (now_ns - self.last_live_body_pose_stamp_ns) / 1e9
            if self.last_live_body_pose_stamp_ns > 0 else None)
        live_body_pose_rx_age = (
            now_mono - self.last_live_body_pose_monotonic
            if self.last_live_body_pose_monotonic > 0.0 else None)
        live_body_anchor_fresh = (
            self.last_live_body_pose_anchor_delta_s is not None
            and self.last_live_body_pose_anchor_delta_s <= self.max_live_body_pose_anchor_delta)
        live_body_pose_fresh = (
            live_body_pose_age is not None and live_body_pose_rx_age is not None
            and -0.2 <= live_body_pose_age <= self.max_live_body_pose_age
            and 0.0 <= live_body_pose_rx_age <= self.max_live_body_pose_age * 1.5
            and live_body_anchor_fresh)
        scan_output_current, latest_scan_lag_s = scan_output_is_fresh(
            self.last_processed_scan_stamp_ns or None,
            self.last_cloud_input_stamp_ns,
            output_age,
            cloud_input_age,
            self.max_cloud_age,
            self.max_latest_scan_lag,
        )
        if pose_scan_skew is None:
            pose_scan_alignment = "unknown"
        elif pose_scan_skew > 0.05:
            pose_scan_alignment = "pose_behind_scan"
        elif pose_scan_skew < -0.05:
            pose_scan_alignment = "pose_ahead_of_scan"
        else:
            pose_scan_alignment = "aligned"

        ready = (
            status_fresh and pose_fresh and odom_fresh and cloud_fresh
            and scan_output_current and live_body_pose_fresh
            and self.sensor_extrinsic_ready and self.motion_compensation_ready
            and bool(compensation_for_processed_scan)
        )
        data = {
            "schema": "n01.live_scan_readiness.v6",
            "status": "LIVE_SCAN_INPUTS_READY" if ready else "WAITING_OR_BLOCKED",
            "localization_status": self.status_reason,
            "localization_status_fresh": bool(status_fresh),
            "localization_status_stamp_age_s": self.status_stamp_age_s,
            "robot_pose_topic": self.pose_topic,
            "robot_pose_contract": self.pose_contract_error or "map_to_base_footprint",
            "robot_pose_receive_fresh": bool(pose_fresh),
            "robot_pose_receive_age_s": (
                now_mono - self.pose_rx_monotonic if self.pose_rx_monotonic > 0.0 else None),
            "robot_pose_stamp_progress_age_s": pose_stamp_progress_age,
            "robot_pose_publisher_count": len(self.pose_publisher_endpoints),
            "robot_pose_publishers": self.pose_publisher_endpoints,
            "robot_pose_multiple_publisher_endpoints": (
                len(self.pose_publisher_endpoints) > 1),
            "robot_pose_publisher_query_error": self.pose_publisher_query_error,
            "robot_pose_stamp_age_s": self.pose_stamp_age_s,
            "robot_pose_cache_samples": len(self.pose_samples),
            "robot_pose_oldest_stamp_ns": oldest_pose_stamp_ns,
            "robot_pose_newest_stamp_ns": latest_pose_stamp_ns,
            "robot_pose_last_stamp_ns": self.last_pose_stamp_ns,
            "robot_pose_max_stamp_age_s": self.max_pose_stamp_age,
            "robot_pose_anchor_max_delta_s": self.max_pose_anchor_delta,
            "robot_pose_live_anchor_max_delta_s": self.max_live_body_pose_anchor_delta,
            "robot_pose_last_error": self.last_pose_error,
            "odom_topic": self.odom_topic,
            "odom_contract": self.odom_contract_error or "%s<- %s" % (self.odom_frame, self.odom_child_frame),
            "odom_receive_fresh": bool(odom_fresh),
            "odom_receive_age_s": (
                now_mono - self.odom_rx_monotonic if self.odom_rx_monotonic > 0.0 else None),
            "odom_stamp_age_s": self.odom_stamp_age_s,
            "odom_cache_samples": len(self.odom_samples),
            "odom_oldest_stamp_ns": oldest_odom_stamp_ns,
            "odom_newest_stamp_ns": latest_odom_stamp_ns,
            "odom_last_stamp_ns": self.last_odom_stamp_ns,
            "odom_interpolation_max_gap_s": self.odom_interpolation_max_gap,
            "odom_last_error": self.last_odom_error,
            "odom_scan_skew_s": odom_scan_skew,
            "odom_scan_bracket_gap_s": odom_scan_bracket_gap,
            "odom_scan_bracket_available": odom_scan_bracket_available,
            "motion_compensation_method": "slam_anchor_plus_odom_relative_motion",
            "motion_compensation_ready": bool(
                self.motion_compensation_ready and bool(compensation_for_processed_scan)),
            "motion_compensation_error": self.last_motion_compensation_error or self.last_pose_error,
            "motion_compensation_anchor_stamp_ns": compensation_for_processed_scan.get("anchor_stamp_ns"),
            "motion_compensation_anchor_delta_s": compensation_for_processed_scan.get("anchor_delta_s"),
            "latest_scan_lag_s": latest_scan_lag_s,
            "latest_scan_within_lag": bool(scan_output_current),
            "max_latest_scan_lag_s": self.max_latest_scan_lag,
            "cloud_input_stamp_ns": self.last_cloud_input_stamp_ns,
            "cloud_input_age_s": cloud_input_age,
            "cloud_input_receive_age_s": cloud_input_rx_age,
            "pose_scan_skew_s": pose_scan_skew,
            "pose_scan_alignment": pose_scan_alignment,
            "pose_scan_before_delta_s": pose_scan_before,
            "pose_scan_after_delta_s": pose_scan_after,
            "pose_scan_bracket_gap_s": pose_scan_bracket_gap,
            "pose_scan_bracket_available": pose_scan_bracket_available,
            "pose_scan_anchor_available": pose_scan_anchor_available,
            "sensor_extrinsic_parent_frame": self.body_frame,
            "sensor_extrinsic_child_frame": self.cloud_frame,
            "sensor_extrinsic_ready": bool(self.sensor_extrinsic_ready),
            "body_pose_live_fresh": bool(live_body_pose_fresh),
            "body_pose_live_stamp_age_s": live_body_pose_age,
            "body_pose_live_receive_age_s": live_body_pose_rx_age,
            "body_pose_live_velocity_valid": bool(self.last_live_body_velocity_valid),
            "body_pose_live_anchor_stamp_ns": self.last_live_body_pose_anchor_stamp_ns,
            "body_pose_live_anchor_delta_s": self.last_live_body_pose_anchor_delta_s,
            "body_pose_live_anchor_fresh": bool(live_body_anchor_fresh),
            "body_pose_live_anchor_max_delta_s": self.max_live_body_pose_anchor_delta,
            "body_pose_live_tf_lag_s": self.last_live_body_tf_lag_s,
            "body_pose_live_tf_max_lag_s": self.max_live_tf_lag,
            "body_pose_live_tf_latest_fallback_count": self.live_body_tf_latest_fallback_count,
            "body_pose_live_error": self.last_live_body_pose_error,
            "body_pose_publish_period_s": self.body_pose_publish_period,
            "cloud_output_fresh": bool(cloud_fresh),
            "cloud_output_age_s": output_age,
            "cloud_output_stamp_ns": self.last_processed_scan_stamp_ns or None,
            "vendor_imu_sync_ok": bool(self.status_imu_sync),
            "fitness_mse": self.status_fitness if math.isfinite(self.status_fitness) else None,
            "velocity_estimate_valid": bool(self.last_velocity_valid),
            "input_cloud_topic": self.cloud_topic,
            "input_cloud_frame": self.cloud_frame,
            "output_cloud_frame": self.map_frame,
            "outputs": ["/cloud", "/sensor_pose", "/body_pose", "/n01/live_navigation/scan_body_pose"],
            "queue_policy": "latest_only_exact_scan_stamp",
            "pending_clouds": len(self.pending_clouds),
            "pending_cloud_limit": 1,
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
