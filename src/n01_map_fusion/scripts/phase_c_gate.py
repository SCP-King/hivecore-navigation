#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Wait for real simulated progress before Phase C obstacle injection.

Read-only ROS node. Exits 0 with a JSON gate record when the robot has moved
at least 0.25 m from its observed start, remains >0.6 m from the goal, and is
still ahead of the configured obstacle site. Exits 2 on timeout.
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

GOAL = (11.05, 6.45)
OBSTACLE = (11.05, 7.15)  # 剩余路径前方;mock 半径 0.15 + 融合膨胀 0.40
MIN_MOVE_M = 0.25
MIN_GOAL_M = 0.60
# 注入时当前位置不得被障碍覆盖:至少超出 OCC 名义边界(0.15+0.40=0.55)
MIN_OBSTACLE_CLEAR_M = 0.55


class Gate(Node):
    def __init__(self):
        super().__init__('n01_phase_c_gate')
        self.first = None
        self.current = None
        self.ready = None
        self.create_subscription(PoseStamped, '/nav/test/pose', self._on_pose, 10)
        self.create_timer(0.05, self._check)

    def _on_pose(self, msg):
        p = (float(msg.pose.position.x), float(msg.pose.position.y))
        if self.first is None:
            self.first = p
        self.current = p

    def _check(self):
        if self.current is None or self.first is None:
            return
        x, y = self.current
        moved = math.hypot(x - self.first[0], y - self.first[1])
        goal_dist = math.hypot(x - GOAL[0], y - GOAL[1])
        obstacle_dist = math.hypot(x - OBSTACLE[0], y - OBSTACLE[1])
        # 障碍必须仍在剩余路径前方(本路线沿 -y 前进,故 y>障碍 y),
        # 且当前位置未被障碍 OCC 覆盖(距中心 >=0.55m 名义边界)。
        ahead = y > OBSTACLE[1] and obstacle_dist >= MIN_OBSTACLE_CLEAR_M
        if moved >= MIN_MOVE_M and goal_dist > MIN_GOAL_M and ahead:
            self.ready = {
                'first_pose': {'x': self.first[0], 'y': self.first[1]},
                'inject_pose': {'x': x, 'y': y},
                'displacement_m': moved,
                'goal_distance_m': goal_dist,
                'obstacle': {'x': OBSTACLE[0], 'y': OBSTACLE[1], 'radius': 0.15},
                'obstacle_center_clearance_m': obstacle_dist,
                'wall_epoch': time.time(),
            }
            # 不在 timer callback 内 shutdown；由 main 的 finally 统一关闭，
            # 避免 rclpy 在当前 executor 回调中死锁。


def main():
    rclpy.init()
    node = Gate()
    deadline = time.monotonic() + 12.0
    try:
        while rclpy.ok() and node.ready is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
    if node.ready is None:
        print(json.dumps({
            'fail': 'gate_timeout_or_no_safe_obstacle_position',
            'first_pose': node.first,
            'last_pose': node.current,
        }), file=sys.stderr)
        return 2
    print(json.dumps(node.ready))
    return 0


if __name__ == '__main__':
    sys.exit(main())
