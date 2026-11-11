#!/usr/bin/env python3
"""Offline harness for n01_traj_tracker.

Feeds a synthetic straight-line B-spline + fake body pose, and records what the
tracker would command. NOTHING here touches the real /cmd_vel: the tracker must
be launched with cmd_vel_topic:=/cmd_vel_test.

Timeline:
  0.0 -  2.0 s  enable=false  -> expect DISABLED, zero cmd messages
  2.0 -  3.5 s  enable=true, no pose yet -> WAITING_FOR_POSE
  3.5 -  9.0 s  pose + bspline at 5 Hz -> TRACKING with vx > 0
  9.0 - 11.0 s  stop bspline -> TRAJ_STALE, zero twist
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from scan_planner_msgs.msg import Bspline

N_CTRL = 53
ORDER = 3
INTERVAL = 0.1


def make_bspline(x_index):
    msg = Bspline()
    msg.order = ORDER
    msg.traj_id = x_index
    msg.start_time = Time(sec=0, nanosec=0)
    for i in range(N_CTRL):
        x = INTERVAL * i
        msg.pos_pts.append(Point(x=float(x), y=0.0, z=0.0))
    for i in range(N_CTRL + ORDER + 1):
        msg.knots.append(float((i - ORDER) * INTERVAL))
    return msg


class Harness(Node):
    def __init__(self):
        super().__init__("traj_tracker_harness")
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pose_pub = self.create_publisher(Odometry, "/body_pose", qos)
        self.bsp_pub = self.create_publisher(Bspline, "/planning/bspline", qos)
        self.en_pub = self.create_publisher(Bool, "/n01/traj_tracker/enable", qos)

        self.cmds = []
        self.status = []
        self.create_subscription(Twist, "/cmd_vel_test", self.on_cmd, qos)
        self.create_subscription(String, "/n01/traj_tracker/status", self.on_status, qos)

        self.t0 = time.monotonic()

    def on_cmd(self, msg):
        self.cmds.append((round(time.monotonic() - self.t0, 2),
                          round(msg.linear.x, 4), round(msg.linear.y, 4),
                          round(msg.angular.z, 4)))

    def on_status(self, msg):
        self.status.append((round(time.monotonic() - self.t0, 2), msg.data))

    def spin(self):
        while time.monotonic() - self.t0 < 11.0:
            t = time.monotonic() - self.t0
            en = Bool()
            en.data = t >= 2.0
            self.en_pub.publish(en)

            if t >= 3.5:
                od = Odometry()
                od.header.frame_id = "map"
                od.child_frame_id = "base_footprint"
                od.pose.pose.position.x = 0.0
                od.pose.pose.position.y = 0.0
                od.pose.pose.orientation.w = 1.0
                self.pose_pub.publish(od)

                if t < 9.0:
                    self.bsp_pub.publish(make_bspline(int(t * 10)))
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init()
    h = Harness()
    try:
        h.spin()
    finally:
        print("=== STATUS timeline (every 10th) ===")
        for ts, s in h.status[::10]:
            print(f"[{ts:5.2f}] {s}")
        print("=== CMD messages: %d ===" % len(h.cmds))
        if h.cmds:
            print("first:", h.cmds[0])
            mid = len(h.cmds) // 2
            print("mid  :", h.cmds[mid])
            print("last :", h.cmds[-1])
            maxvx = max(abs(c[1]) for c in h.cmds)
            maxvy = max(abs(c[2]) for c in h.cmds)
            maxwz = max(abs(c[3]) for c in h.cmds)
            print(f"max |vx|={maxvx:.4f} |vy|={maxvy:.4f} |wz|={maxwz:.4f}")
        print("=== phases ===")
        pre = [c for c in h.cmds if c[0] < 2.0]
        mid_p = [c for c in h.cmds if 3.6 <= c[0] <= 9.0]
        post = [c for c in h.cmds if c[0] > 9.5]
        print(f"before enable: {len(pre)} msgs (expect 0)")
        print(f"tracking     : {len(mid_p)} msgs, vx>0 count={sum(1 for c in mid_p if c[1] > 0.01)}")
        print(f"after stale  : {len(post)} msgs, all-zero={all(abs(c[1])<1e-9 and abs(c[2])<1e-9 and abs(c[3])<1e-9 for c in post) if post else 'n/a'}")
        h.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
