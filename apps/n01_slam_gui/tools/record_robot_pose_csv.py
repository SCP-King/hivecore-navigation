#!/usr/bin/env python3
"""Record /robot_pose (or /n01/localization/pose) to CSV for PCD ego cleanup.

CSV: timestamp x y z qx qy qz qw
Run while mapping; stop with Ctrl+C after saving the PCD.
"""
import argparse
import csv
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class Recorder(Node):
    def __init__(self, args):
        super().__init__("n01_pose_csv_recorder")
        self.args = args
        self.file = open(args.output, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        self.last = None
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, args.topic, self.on_pose, qos)

    def on_pose(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last is not None:
            dx = p.x - self.last[0]
            dy = p.y - self.last[1]
            if math.hypot(dx, dy) < self.args.min_distance:
                return
        self.last = (p.x, p.y)
        self.writer.writerow([f"{stamp:.9f}", p.x, p.y, p.z,
                              q.x, q.y, q.z, q.w])
        self.file.flush()

    def close(self):
        self.file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--topic", default="/robot_pose")
    parser.add_argument("--min-distance", type=float, default=.10)
    args = parser.parse_args()
    rclpy.init()
    node = Recorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
