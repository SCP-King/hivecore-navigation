"""G1 deterministic offline input feed for the REAL SCAN planner (not a planner).

Only publishes sensor pose, body pose, point cloud and RViz goal in domain 88.
No fake TF, no cmd_vel, no robot chassis communication.
"""
import math
import os
import struct
import time

if os.environ.get("ROS_DOMAIN_ID") != "88" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise RuntimeError("G1 feed must run in isolated ROS domain 88 with localhost-only")

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField


def test_obstacle_points():
    """Deterministic wall returns; rays toward it should clear space near straight goal."""
    points = []
    for iy in range(-24, 25):
        y = iy * 0.125
        for z in (0.2, 0.4, 0.6, 0.85, 1.1):
            points.append((3.3, y, z))
    # An offset obstacle probes local 3D occupancy without blocking the direct goal.
    for ix in range(7):
        for iy in range(7):
            for z in (0.35, 0.6, 0.9):
                points.append((1.1 + ix * 0.05, 1.2 + iy * 0.05, z))
    return points


def xyz_cloud(stamp, points):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = "world"
    msg.height, msg.width = 1, len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.data = b"".join(struct.pack("<fff", *point) for point in points)
    msg.is_dense = True
    return msg


class G1Feed(Node):
    def __init__(self):
        super().__init__("scan_g1_offline_feed")
        self.body_pub = self.create_publisher(Odometry, "/body_pose", qos_profile_sensor_data)
        self.sensor_pub = self.create_publisher(Odometry, "/sensor_pose", qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, "/cloud", qos_profile_sensor_data)
        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.points = test_obstacle_points()
        self.started = time.monotonic()
        self.count = 0
        self.goal_times = (4.0, 17.0, 32.0, 52.0)
        self.goal_attempts = 0
        self.create_timer(.05, self.tick)
        self.get_logger().warning("G1 TEST FEED; no fake TF or actuator output")

    def tick(self):
        self.count += 1
        stamp = self.get_clock().now().to_msg()
        for pub, z in ((self.body_pub, .4), (self.sensor_pub, .8)):
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = "world"
            msg.child_frame_id = "base_link" if z == .4 else "sensor"
            msg.pose.pose.position.z = z
            msg.pose.pose.orientation.w = 1.0
            pub.publish(msg)
        if self.count % 2 == 0:
            self.cloud_pub.publish(xyz_cloud(stamp, self.points))
        if (self.goal_attempts < len(self.goal_times)
                and time.monotonic() - self.started >= self.goal_times[self.goal_attempts]):
            goal = PoseStamped()
            goal.header.stamp = stamp
            goal.header.frame_id = "world"
            goal.pose.position.x = 2.0
            goal.pose.position.y = 0.0
            goal.pose.position.z = 0.4
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.goal_attempts += 1
            self.get_logger().info(
                "Published offline goal attempt %d/%d at (2.0, 0.0, 0.4)" %
                (self.goal_attempts, len(self.goal_times)))


def main(args=None):
    rclpy.init(args=args)
    node = G1Feed()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
