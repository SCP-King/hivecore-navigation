"""Read-only B-spline path sampler. No TF or robot commands."""
import math
import os
import time

if os.environ.get('ROS_DOMAIN_ID') != '88' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('bridge only runs in ROS_DOMAIN_ID=88, ROS_LOCALHOST_ONLY=1')

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from scan_planner_msgs.msg import Bspline
from .curve import BSplineCurve, CurveError


class SamplerNode(Node):
    def __init__(self):
        super().__init__('scan_bspline_bridge')
        for name, default in (
            ('frame_id', 'world'), ('bspline_topic', '/planning/bspline'),
            ('odom_topic', '/body_pose'), ('path_topic', '/nav/test/global_path'),
            ('path_points', 120), ('arc_step', 0.05), ('table_samples', 400),
            ('odom_timeout_s', 0.75), ('trajectory_timeout_s', 3.0)):
            self.declare_parameter(name, default)
        get = lambda n: self.get_parameter(n).value
        self.frame = get('frame_id')
        self.points, self.step, self.samples = get('path_points'), get('arc_step'), get('table_samples')
        self.odom_timeout, self.traj_timeout = get('odom_timeout_s'), get('trajectory_timeout_s')
        if not isinstance(self.frame, str) or not self.frame or self.frame.startswith('/'):
            raise ValueError('invalid frame_id')
        if (any(isinstance(x, bool) or not isinstance(x, int) for x in (self.points, self.samples))
            or not (2 <= self.points <= 3000 and 20 <= self.samples <= 20000)):
            raise ValueError('invalid count')
        if (not all(isinstance(x, (float, int)) and math.isfinite(x)
                    for x in (self.step, self.odom_timeout, self.traj_timeout))
            or not (.001 <= self.step <= 5 and .05 <= self.odom_timeout <= 60 and .05 <= self.traj_timeout <= 60)):
            raise ValueError('invalid timing/step')
        self.curve = self.traj_rx = self.odom_xy = self.odom_rx = self.last_anchor = None
        self.need_publish = False
        self.path_pub = self.create_publisher(Path, get('path_topic'), 10)
        self.valid_pub = self.create_publisher(Bool, '/nav/test/bridge_valid', 10)
        self.create_subscription(Bspline, get('bspline_topic'), self.on_trajectory, 10)
        self.create_subscription(Odometry, get('odom_topic'), self.on_odom,
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self.on_timer)

    def on_trajectory(self, msg):
        try:
            curve = BSplineCurve(int(msg.order), msg.knots,
                                 [(p.x, p.y, p.z) for p in msg.pos_pts], self.samples)
        except (CurveError, ValueError, TypeError, OverflowError) as exc:
            self.curve, self.traj_rx, self.last_anchor = None, None, None
            self.need_publish = False
            self.valid_pub.publish(Bool(data=False))
            self.get_logger().error('Rejected Bspline: %s' % exc)
            return
        self.curve, self.traj_rx = curve, time.monotonic()
        self.last_anchor, self.need_publish = None, True

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if msg.header.frame_id != self.frame or not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.odom_xy, self.odom_rx = None, None
            return
        self.odom_xy, self.odom_rx = (p.x, p.y), time.monotonic()

    def on_timer(self):
        now = time.monotonic()
        ready = (self.curve is not None and self.odom_xy is not None
                 and self.traj_rx is not None and self.odom_rx is not None
                 and 0 <= now - self.traj_rx <= self.traj_timeout
                 and 0 <= now - self.odom_rx <= self.odom_timeout)
        self.valid_pub.publish(Bool(data=bool(ready)))
        if not ready:
            return
        try:
            s, samples = self.curve.sample_window(*self.odom_xy, count=self.points, step=self.step)
        except CurveError as exc:
            self.curve = None
            self.valid_pub.publish(Bool(data=False))
            self.get_logger().error('Sampling failed: %s' % exc)
            return
        if not self.need_publish and self.last_anchor is not None and abs(s - self.last_anchor) < self.step / 2.0:
            return
        self.need_publish, self.last_anchor = False, s
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, z, yaw in samples:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            path.poses.append(pose)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = SamplerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
