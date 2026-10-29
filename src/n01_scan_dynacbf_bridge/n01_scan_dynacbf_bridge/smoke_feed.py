"""Deterministic synthetic trajectory + odometry. Isolation-only."""
import os
if os.environ.get('ROS_DOMAIN_ID') != '88' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('synthetic data prohibited outside isolated domain 88')
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from scan_planner_msgs.msg import Bspline


class Feed(Node):
    def __init__(self):
        super().__init__('bridge_synthetic_feed')
        self.odom_pub = self.create_publisher(Odometry, '/body_pose', 10)
        self.path_pub = self.create_publisher(Bspline, '/planning/bspline', 10)
        self.count = 0
        self.create_timer(.1, self.tick)

    def tick(self):
        self.count += 1
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = .3
        odom.pose.pose.position.z = .3
        odom.pose.pose.orientation.w = 1.
        self.odom_pub.publish(odom)
        if self.count % 10:
            return
        spline = Bspline()
        spline.order = 3
        spline.traj_id = 1
        spline.start_time = stamp
        spline.knots = [0., 0., 0., 0., 1., 1., 1., 1.]
        spline.pos_pts = [Point(x=float(i), y=0.0, z=.3) for i in range(4)]
        spline.yaw_pts = []
        spline.yaw_dt = 0.
        self.path_pub.publish(spline)


def main(args=None):
    rclpy.init(args=args)
    node = Feed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
