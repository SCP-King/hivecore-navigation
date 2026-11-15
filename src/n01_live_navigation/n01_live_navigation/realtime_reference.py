"""Relay SLAM map snapshots as a retained global reference, never as live rays."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def reference_rejection(message, now_ns, last_stamp_ns, max_age_s=5.0, max_points=1000000):
    stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
    if message.header.frame_id != 'map':
        return 'reference_frame_mismatch'
    if stamp <= 0 or stamp <= last_stamp_ns:
        return 'reference_stamp_not_advancing'
    if not -0.2 <= (now_ns - stamp) / 1e9 <= max_age_s:
        return 'reference_stale'
    count = message.width * message.height
    if not 0 < count <= max_points:
        return 'reference_point_count_out_of_bounds'
    if not {'x', 'y', 'z'} <= {field.name for field in message.fields}:
        return 'reference_missing_xyz'
    if (message.point_step <= 0 or message.row_step < message.width * message.point_step
            or len(message.data) < message.row_step * message.height):
        return 'reference_data_truncated'
    return ''


class RealtimeReference(Node):
    def __init__(self):
        super().__init__('n01_realtime_reference')
        topic = self.declare_parameter('input_topic', '/cloud_map').value
        self.last_stamp_ns = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PointCloud2, '/grid_map/static_occupancy', qos)
        self.sub = self.create_subscription(PointCloud2, topic, self.on_map, qos_profile_sensor_data)

    def on_map(self, message):
        error = reference_rejection(message, self.get_clock().now().nanoseconds, self.last_stamp_ns)
        if error:
            self.get_logger().warning(error, throttle_duration_sec=5.0)
            return
        self.last_stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        # Retain original frame/time. A historical map must never be republished
        # onto /cloud or synchronized with the newest sensor pose for raycasting.
        self.pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeReference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
