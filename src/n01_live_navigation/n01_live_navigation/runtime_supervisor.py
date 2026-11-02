"""Read-only runtime supervisor for localization, static map and SCAN readiness."""
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3Stamped


class LiveRuntimeSupervisor(Node):
    def __init__(self):
        super().__init__("n01_live_runtime_supervisor")
        self.declare_parameter("max_body_pose_age_s", 0.6)
        self.declare_parameter("max_body_pose_rx_age_s", 0.8)
        self.declare_parameter("status_topic", "/n01/live_navigation/readiness")
        self.declare_parameter("static_ready_topic", "/grid_map/static_overlay_ready")
        self.declare_parameter("body_pose_topic", "/body_pose")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("timings_topic", "/mapping_processing_timings")
        self.max_body_age = float(self.get_parameter("max_body_pose_age_s").value)
        self.max_body_rx_age = float(self.get_parameter("max_body_pose_rx_age_s").value)
        if not (0.1 <= self.max_body_age <= 2.0 and 0.1 <= self.max_body_rx_age <= 3.0):
            raise ValueError("runtime supervisor age bounds invalid")

        self.bridge = {}
        self.static_ready = False
        self.body_stamp_ns = None
        self.body_rx_monotonic = None
        self.map_tf_stamp_ns = None
        self.tf_intervals = deque(maxlen=20)
        self.last_tf_stamp_ns = None
        self.timings = None
        self.timings_stamp_ns = None
        self.timings_rx_monotonic = None
        self.robot_feasibility = {}
        self.last_health = {}

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            String, self.get_parameter("status_topic").value, self.on_bridge, 10)
        self.create_subscription(
            Bool, self.get_parameter("static_ready_topic").value, self.on_static, latched)
        self.create_subscription(
            Odometry, self.get_parameter("body_pose_topic").value,
            self.on_body_pose, qos_profile_sensor_data)
        self.create_subscription(
            TFMessage, self.get_parameter("tf_topic").value, self.on_tf, tf_qos)
        self.create_subscription(
            Vector3Stamped, self.get_parameter("timings_topic").value,
            self.on_timings, 10)
        self.create_subscription(
            String, "/n01/live_navigation/hive2_feasibility", self.on_robot_feasibility, 10)
        self.ready_pub = self.create_publisher(
            Bool, "/n01/live_navigation/navigation_ready", 10)
        self.health_pub = self.create_publisher(
            String, "/n01/live_navigation/system_health", 10)
        self.create_timer(0.25, self.publish_health)

    def on_bridge(self, msg):
        try:
            self.bridge = json.loads(msg.data)
        except (ValueError, TypeError):
            self.bridge = {"status": "INVALID_BRIDGE_STATUS"}

    def on_static(self, msg):
        self.static_ready = bool(msg.data)

    def on_body_pose(self, msg):
        self.body_stamp_ns = (
            int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec))
        self.body_rx_monotonic = time.monotonic()

    def on_tf(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if parent == "map" and child == "base_footprint":
                stamp_ns = (
                    int(transform.header.stamp.sec) * 1000000000
                    + int(transform.header.stamp.nanosec))
                if self.last_tf_stamp_ns is not None and stamp_ns > self.last_tf_stamp_ns:
                    self.tf_intervals.append((stamp_ns - self.last_tf_stamp_ns) / 1e9)
                self.last_tf_stamp_ns = stamp_ns
                self.map_tf_stamp_ns = stamp_ns

    def on_timings(self, msg):
        self.timings = [float(msg.vector.x), float(msg.vector.y), float(msg.vector.z)]
        self.timings_stamp_ns = (
            int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec))
        self.timings_rx_monotonic = time.monotonic()

    def on_robot_feasibility(self, msg):
        try:
            self.robot_feasibility = json.loads(msg.data)
        except (ValueError, TypeError):
            self.robot_feasibility = {"status": "INVALID_ROBOT_FEASIBILITY"}

    def publish_health(self):
        now_ns = self.get_clock().now().nanoseconds
        now_mono = time.monotonic()
        body_stamp_age = (
            (now_ns - self.body_stamp_ns) / 1e9 if self.body_stamp_ns is not None else None)
        body_rx_age = (
            now_mono - self.body_rx_monotonic if self.body_rx_monotonic is not None else None)
        tf_age = (
            (now_ns - self.map_tf_stamp_ns) / 1e9 if self.map_tf_stamp_ns is not None else None)
        tf_rate = (
            1.0 / (sum(self.tf_intervals) / len(self.tf_intervals))
            if self.tf_intervals and sum(self.tf_intervals) > 0 else None)
        timings_age = (
            (now_ns - self.timings_stamp_ns) / 1e9 if self.timings_stamp_ns is not None else None)

        bridge_ready = (
            self.bridge.get("status") == "LIVE_SCAN_INPUTS_READY"
            and self.bridge.get("localization_status_fresh") is True
            and self.bridge.get("cloud_output_fresh") is True
        )
        body_fresh = (
            body_stamp_age is not None and body_rx_age is not None
            and -0.2 <= body_stamp_age <= self.max_body_age
            and 0.0 <= body_rx_age <= self.max_body_rx_age
        )
        tf_seen = self.map_tf_stamp_ns is not None
        blockers = []
        if not bridge_ready:
            blockers.append("live_localization_or_scan_bridge_not_ready")
        if not self.static_ready:
            blockers.append("static_pcd_overlay_not_ready_in_current_window")
        if not body_fresh:
            blockers.append("body_pose_missing_or_stale")
        if not tf_seen:
            blockers.append("map_to_base_footprint_tf_not_observed")

        navigation_ready = not blockers
        health = {
            "schema": "n01.live_system_health.v1",
            "status": "READY_FOR_GOAL_PLANNING" if navigation_ready else "BLOCKED",
            "navigation_ready": bool(navigation_ready),
            "blockers": blockers,
            "bridge": self.bridge,
            "static_overlay_ready": bool(self.static_ready),
            "body_pose_stamp_age_s": body_stamp_age,
            "body_pose_receive_age_s": body_rx_age,
            "map_base_tf_stamp_age_s": tf_age,
            "map_base_tf_rate_hz": tf_rate,
            "vendor_processing_timings_raw_xyz": self.timings,
            "vendor_processing_timings_age_s": timings_age,
            "vendor_processing_timings_semantics": (
                "vendor-defined Vector3Stamped; raw x/y/z only, no guessed units or labels"),
            "robot_feasibility": self.robot_feasibility,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
        self.ready_pub.publish(Bool(data=bool(navigation_ready)))
        self.health_pub.publish(String(data=json.dumps(health, sort_keys=True)))
        self.last_health = health


def main(args=None):
    rclpy.init(args=args)
    node = LiveRuntimeSupervisor()
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
