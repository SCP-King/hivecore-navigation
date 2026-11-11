#!/usr/bin/env python3
"""Publish one nearby planning goal and measure output latency (no motion)."""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scan_planner_msgs.msg import Bspline
from std_msgs.msg import Bool, String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'))
    args = parser.parse_args()
    rclpy.init()
    node = Node('n01_planning_latency_probe')
    state = {}
    started = None
    result = {}

    def receive(key, msg):
        state[key] = (msg, time.monotonic())
        if started is None:
            return
        if key in ('global', 'spline', 'path') and key not in result:
            if key != 'spline' and not msg.poses:
                return
            if key == 'spline' and not msg.pos_pts:
                return
            # Reject retained/global samples from the preceding goal.
            stamp = msg.start_time if key == 'spline' else msg.header.stamp
            if stamp.sec * 10**9 + stamp.nanosec < goal_stamp_ns:
                return
            result[key] = {'latency_s': round(time.monotonic() - started, 4),
                           'points': len(msg.pos_pts if key == 'spline' else msg.poses)}
            print(key, json.dumps(result[key]), flush=True)

    for key, topic, typ, qos in [
        ('pose', '/body_pose', Odometry, qos_profile_sensor_data),
        ('ready', '/n01/live_navigation/navigation_ready', Bool, 10),
        ('gate', '/n01/live_navigation/goal_gate_status', String, 10),
        ('feasibility', '/n01/live_navigation/hive2_feasibility', String, 10),
        ('global', '/planning/global_path', Path, 10),
        ('spline', '/planning/bspline', Bspline, 10),
        ('path', '/n01/live_navigation/hive2_candidate_path', Path, 10),
    ]:
        node.create_subscription(typ, topic, lambda msg, key=key: receive(key, msg), qos)
    publisher = node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
    try:
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if (all(key in state for key in ('pose', 'ready'))
                    and state['ready'][0].data
                    and now - state['ready'][1] < 0.5
                    and now - state['pose'][1] < 0.5
                    and publisher.get_subscription_count()):
                break
        else:
            raise RuntimeError('Fresh pose/navigation readiness unavailable')
        consumers = {p.node_name for p in node.get_subscriptions_info_by_topic('/planning/bspline')}
        allowed = {'n01_live_runtime_supervisor', 'n01_hive2_trajectory_monitor', node.get_name()}
        if consumers - allowed:
            raise RuntimeError(f'Unexpected trajectory consumers: {consumers - allowed}')
        pose = state['pose'][0].pose.pose
        q = pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = pose.position.x + math.cos(yaw)
        goal.pose.position.y = pose.position.y + math.sin(yaw)
        if args.goal:
            goal.pose.position.x, goal.pose.position.y = args.goal
        goal.pose.orientation.w = 1.0
        goal_stamp_ns = goal.header.stamp.sec * 10**9 + goal.header.stamp.nanosec
        started = time.monotonic()
        publisher.publish(goal)
        print('goal', goal.pose.position.x, goal.pose.position.y, flush=True)
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(key in result for key in ('global', 'spline', 'path')):
                feasibility = state.get('feasibility')
                if (feasibility and feasibility[1] >= started + result['spline']['latency_s']):
                    break
        for key in ('gate', 'feasibility'):
            if key in state:
                result[key] = json.loads(state[key][0].data)
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
        return 0 if all(key in result for key in ('global', 'spline', 'path')) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
