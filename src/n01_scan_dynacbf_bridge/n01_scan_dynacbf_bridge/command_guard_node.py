"""Test-only zero-default command sink, NOT an independent hardware E-stop."""
import math
import os
import time

if os.environ.get('ROS_DOMAIN_ID') != '88' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('guard only runs in ROS_DOMAIN_ID=88, ROS_LOCALHOST_ONLY=1')

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String
from .guard_logic import checked_command


class TestGuard(Node):
    def __init__(self):
        super().__init__('dcbf_test_guard')
        for name, default in (('armed', False), ('max_v', .5),
                              ('max_omega', .3), ('timeout_s', .5)):
            self.declare_parameter(name, default)
        get = lambda n: self.get_parameter(n).value
        self.armed = get('armed') is True
        self.max_v, self.max_w, self.timeout = float(get('max_v')), float(get('max_omega')), float(get('timeout_s'))
        if (not all(math.isfinite(x) for x in (self.max_v, self.max_w, self.timeout))
            or not (0 < self.max_v <= 1 and 0 < self.max_w <= 3 and .05 <= self.timeout <= 2)):
            raise ValueError('invalid limits')
        self.valid = self.move = False
        self.plan = self.valid_rx = self.move_rx = self.plan_rx = None
        self.output = self.create_publisher(Twist, '/nav/test/cmd_vel', 10)
        self.status = self.create_publisher(String, '/nav/test/guard_status', 10)
        self.create_subscription(Bool, '/nav/test/bridge_valid', self.on_valid, 10)
        self.create_subscription(Bool, '/cmd_move', self.on_move, 10)
        self.create_subscription(Float32MultiArray, '/local_plan', self.on_plan, 10)
        self.create_timer(.05, self.tick)
        self.get_logger().warning('TEST only /nav/test/cmd_vel; armed=%s' % self.armed)

    def on_valid(self, msg):
        self.valid, self.valid_rx = bool(msg.data), time.monotonic()

    def on_move(self, msg):
        self.move, self.move_rx = bool(msg.data), time.monotonic()

    def on_plan(self, msg):
        self.plan, self.plan_rx = list(msg.data), time.monotonic()

    def tick(self):
        now = time.monotonic()
        age = lambda t: now - t if t is not None else math.inf
        v, w, reason = checked_command(self.armed, self.valid, self.move, self.plan,
                                       age(self.valid_rx), age(self.move_rx), age(self.plan_rx),
                                       self.timeout, self.max_v, self.max_w)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.output.publish(cmd)
        self.status.publish(String(data=reason))


def main(args=None):
    rclpy.init(args=args)
    node = TestGuard()
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
