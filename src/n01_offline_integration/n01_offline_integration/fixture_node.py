"""Domain-88 synthetic SLAM+Livox fixture. No real robot or cmd topics.

Publishes a valid TF chain, fake test-only readiness Bool and lidar_link points.
Never use this fixture as proof that genuine LocalizationStatus is integrated.
"""
import math
import os
import struct
import time

if (os.environ.get("ROS_DOMAIN_ID") != "88" or
        os.environ.get("ROS_LOCALHOST_ONLY") != "1" or
        os.environ.get("N01_OFFLINE_FIXTURE") != "1"):
    raise RuntimeError("Fixture requires explicit isolated offline environment")

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def new_tf(stamp, parent, child, xyz):
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id, tf.child_frame_id = parent, child
    # ROS 2 Humble generated Vector3 setters reject Python int, even for zero.
    x, y, z = (float(value) for value in xyz)
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = z
    tf.transform.rotation.w = 1.0
    return tf


class Fixture(Node):
    def __init__(self):
        super().__init__("n01_offline_fixture_source")
        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.sensor_pub = self.create_publisher(
            PointCloud2, "/livox/lidar/geo_filtered", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(
            Bool, "/n01/offline/test/localization_ready", qos_profile_sensor_data)
        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.started = time.monotonic()
        self.count = 0
        self.goal_count = 0
        self.next_goal = (7.0, 23.0, 42.0)
        self.positions = self.points()
        self.create_timer(.1, self.tick)
        self.get_logger().warning(
            "SYNTHETIC TF fixture: SLAM map and Hive2 static z=0.90 are TEST INPUTS ONLY")

    def points(self):
        result = []
        # Free corridor from origin to goal (2.0,0); a wall lies BEYOND goal.
        for ystep in range(-20, 21):
            for z in (.20, .40, .60, .85, 1.10):
                result.append((3.20, ystep*.12, z))
        # Offset obstacle to trigger a nonempty inflated map without route block.
        for ix in range(8):
            for iy in range(8):
                for z in (.3, .6, .9):
                    result.append((1.1+ix*.05, 1.15+iy*.05, z))
        return result

    def tick(self):
        self.count += 1
        stamp = self.get_clock().now().to_msg()
        if self.count == 1:
            self.static_broadcaster.sendTransform([
                new_tf(stamp, "base_footprint", "base_link", (0,0,.9)),
                new_tf(stamp, "base_link", "lidar_link", (.295,0,0)),
            ])
        self.dynamic_broadcaster.sendTransform(
            new_tf(stamp, "map", "base_footprint", (0,0,0)))
        self.status_pub.publish(Bool(data=True))
        if self.count % 2 == 0:
            msg = PointCloud2()
            msg.header.stamp = stamp
            msg.header.frame_id = "lidar_link"
            msg.width, msg.height = len(self.positions), 1
            msg.fields = [
                PointField(name="x",offset=0,datatype=PointField.FLOAT32,count=1),
                PointField(name="y",offset=4,datatype=PointField.FLOAT32,count=1),
                PointField(name="z",offset=8,datatype=PointField.FLOAT32,count=1),
            ]
            msg.is_bigendian, msg.point_step = False, 12
            msg.row_step = msg.point_step * msg.width
            msg.data = b"".join(struct.pack("<fff", x-.295,y,z-.9)
                                 for x,y,z in self.positions)
            msg.is_dense = True
            self.sensor_pub.publish(msg)
        elapsed = time.monotonic() - self.started
        if self.goal_count < len(self.next_goal) and elapsed >= self.next_goal[self.goal_count]:
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = "map"
            msg.pose.position.x, msg.pose.position.z = 2.0, .9
            msg.pose.orientation.w = 1.0
            self.goal_pub.publish(msg)
            self.goal_count += 1
            self.get_logger().info("SYNTHETIC test goal #%d" % self.goal_count)


def main(args=None):
    rclpy.init(args=args)
    node = Fixture()
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
