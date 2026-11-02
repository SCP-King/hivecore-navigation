"""Read-only adapter for the isolated FAST-LIO2 odometry stream.

It normalizes message delivery only. It does not relabel the FAST-LIO map as
the vendor map, publish TF, or claim saved-map localization/relocalization.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class FastLioOdometryAdapter(Node):
    def __init__(self):
        super().__init__("n01_fast_lio_odometry_adapter")
        self.declare_parameter("input_topic", "/n01/fast_lio/odometry")
        self.declare_parameter("output_topic", "/n01/slam/odometry")
        self.declare_parameter("diagnostics_topic", "/n01/slam/diagnostics")
        self.declare_parameter("expected_frame", "n01_fast_lio_map")
        self.declare_parameter("expected_child_frame", "imu_link")
        self.declare_parameter("max_age_s", 0.5)
        self.declare_parameter("max_position_abs_m", 100000.0)
        self.expected_frame = str(self.get_parameter("expected_frame").value)
        self.expected_child = str(self.get_parameter("expected_child_frame").value)
        self.max_age = float(self.get_parameter("max_age_s").value)
        self.max_position = float(self.get_parameter("max_position_abs_m").value)
        if not (0.05 <= self.max_age <= 5.0 and self.max_position >= 10.0):
            raise ValueError("invalid FAST-LIO adapter bounds")
        self.output = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 10)
        self.diag = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("diagnostics_topic").value), 10)
        self.sub = self.create_subscription(
            Odometry, str(self.get_parameter("input_topic").value),
            self.on_odometry, 10)
        self.accepted = 0
        self.rejected = 0
        self.get_logger().warning(
            "FAST-LIO adapter is odometry-only; private map frame is preserved; "
            "no TF, vendor topic, or actuator output is created")

    def publish_status(self, level, message, frame="", child="", age=float("nan")):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "n01_fast_lio/odometry_input"
        status.level = level
        status.message = message
        status.hardware_id = "FAST-LIO2"
        status.values = [
            KeyValue(key="accepted", value=str(self.accepted)),
            KeyValue(key="rejected", value=str(self.rejected)),
            KeyValue(key="frame_id", value=frame),
            KeyValue(key="child_frame_id", value=child),
            KeyValue(key="age_s", value=str(age)),
            KeyValue(key="map_alignment", value="UNVERIFIED_PRIVATE_FRAME"),
            KeyValue(key="saved_map_localization", value="NOT_IMPLEMENTED"),
        ]
        arr.status.append(status)
        self.diag.publish(arr)

    def on_odometry(self, msg):
        frame = msg.header.frame_id
        child = msg.child_frame_id
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        xyz = (p.x, p.y, p.z)
        quat = (q.x, q.y, q.z, q.w)
        valid = (frame == self.expected_frame and child == self.expected_child and
                 all(math.isfinite(v) for v in xyz + quat) and
                 max(abs(v) for v in xyz) <= self.max_position and
                 abs(sum(v*v for v in quat) - 1.0) < 0.05)
        if not valid:
            self.rejected += 1
            self.publish_status(DiagnosticStatus.ERROR,
                                "REJECTED_FRAME_OR_NONFINITE_POSE", frame, child)
            return
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        age = (now - stamp).nanoseconds / 1e9
        if age < -0.1 or age > self.max_age:
            self.rejected += 1
            self.publish_status(DiagnosticStatus.WARN,
                                "REJECTED_STALE_OR_FUTURE_ODOMETRY", frame, child, age)
            return
        self.output.publish(msg)
        self.accepted += 1
        self.publish_status(DiagnosticStatus.OK,
                            "FAST_LIO_ODOMETRY_STREAMING_PRIVATE_MAP_UNALIGNED",
                            frame, child, age)


def main(args=None):
    rclpy.init(args=args)
    node = FastLioOdometryAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
