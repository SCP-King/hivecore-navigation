#!/usr/bin/env python3
"""Explicit, supervised 0.03 m/s short motion test with independent cleanup."""
import argparse
import json
import math
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Operator confirmed clear area and emergency stop')
    parser.add_argument('--observe-seconds', type=float, default=4.0)
    parser.add_argument('--to-goal', action='store_true', help='Separately authorized full-route test')
    parser.add_argument('--goal', nargs=2, type=float)
    parser.add_argument('--scan-only',action='store_true',help='Explicit operator-authorized test without CBF')
    parser.add_argument('--max-test-seconds',type=float,default=120.)
    parser.add_argument('--max-travel-m',type=float,default=1.5)
    args = parser.parse_args()
    if args.to_goal and (not args.execute or not args.goal):
        parser.error('--to-goal requires --execute and explicit --goal X Y')
    if not (0 < args.max_test_seconds <= 300 and 0 < args.max_travel_m <= 5):
        parser.error('test watchdog bounds must be finite: <=300s, <=5m')
    rclpy.init()
    n = Node('n01_bounded_motion_test')
    state = {}
    samples = []
    child = None
    cmd_max = 0.0
    active = False
    measured = 0.0
    previous = None
    initial_odom = None
    odom_net_displacement = 0.0
    initial_map = None
    map_net_displacement = 0.0
    command_distance = 0.0
    last_cmd_time = None
    last_cmd_speed = 0.0
    path_events = []
    path_changes = []
    last_path_shape = None

    def receive(key, msg):
        nonlocal measured, previous, cmd_max, last_path_shape, initial_odom, odom_net_displacement
        nonlocal command_distance,last_cmd_time,last_cmd_speed
        nonlocal map_net_displacement
        state[key] = (msg, time.monotonic())
        if key=='pose' and active and initial_map is not None:
            p=msg.pose.pose.position
            map_net_displacement=math.hypot(p.x-initial_map[0],p.y-initial_map[1])
        if key == 'odom' and active:
            p = msg.pose.pose.position
            if initial_odom is None:initial_odom=(p.x,p.y)
            odom_net_displacement=math.hypot(p.x-initial_odom[0],p.y-initial_odom[1])
            if previous:
                measured += math.hypot(p.x-previous[0], p.y-previous[1])
            previous = (p.x, p.y)
        if key == 'cmd':
            speed = math.hypot(msg.linear.x, msg.linear.y)
            if active:
                now=time.monotonic()
                if last_cmd_time is not None:command_distance+=last_cmd_speed*max(0.,now-last_cmd_time)
                last_cmd_time,last_cmd_speed=now,speed
                cmd_max = max(cmd_max, speed)
            else:
                samples.append(speed)
        if key == 'path':
            path_events.append((time.monotonic(), len(msg.poses)))
            shape=tuple((round(p.pose.position.x,3),round(p.pose.position.y,3))
                        for p in msg.poses[::max(1,len(msg.poses)//20)])
            if shape and shape != last_path_shape:
                path_changes.append(time.monotonic())
            last_path_shape=shape
        if key == 'executor':
            data = json.loads(msg.data)
            if data.get('state') != state.get('last_executor_state'):
                state['last_executor_state'] = data.get('state')
                print('EXECUTOR', msg.data, flush=True)

    for key, topic, typ in [
        ('pose', '/body_pose', Odometry), ('odom', '/odom', Odometry),
        ('health', '/n01/live_navigation/system_health', String),
        ('feas', '/n01/live_navigation/hive2_feasibility', String),
        ('path', '/n01/navigation/path', Path),
        ('cmd', '/cmd_vel', Twist),
        ('executor', '/n01/live_navigation/slow_motion/status', String),
    ]:
        n.create_subscription(typ, topic, lambda m, k=key: receive(k, m), qos_profile_sensor_data)
    goal_pub = n.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
    zero_pub = n.create_publisher(Twist, '/cmd_vel', 10)
    enable_pub = n.create_publisher(Bool, '/n01/slow_motion/enable', 10)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=.05)

    def ready():
        now = time.monotonic()
        if any(k not in state or now-state[k][1] > .6 for k in ('pose','odom','health')):
            return False
        h = json.loads(state['health'][0].data)
        return h.get('navigation_ready') is True and h.get('blockers') == []

    try:
        spin(4)
        if not ready():
            raise RuntimeError('Fresh localization/runtime/odometry not ready')
        publishers = {p.node_name for p in n.get_publishers_info_by_topic('/cmd_vel')}
        # Test execution requires exclusive ownership of chassis velocity.
        # Even idle teleop/follow publishers can wake and override navigation.
        allowed = {n.get_name()}
        if publishers - allowed or (samples and max(samples) > .001):
            raise RuntimeError(f'Competing velocity source: publishers={publishers}, samples={len(samples)}; test refused')
        consumers = {p.node_name for p in n.get_subscriptions_info_by_topic('/planning/bspline')}
        if consumers - {'n01_live_runtime_supervisor', 'n01_hive2_trajectory_monitor'}:
            raise RuntimeError(f'Unexpected trajectory consumer: {consumers}')
        if args.execute:
            child = subprocess.Popen([
                'ros2','run','n01_live_navigation','n01_slow_motion_executor','--ros-args',
                '-p','enabled:=false','-p','max_speed_mps:=0.03',
                '-p',f'cbf_enabled:={str(not args.scan_only).lower()}',
                '-p','path_source:=unified','-p','control_mode:=four_wis',
                '-p','max_yaw_rate_radps:=0.04','-p','max_accel_mps2:=0.02',
                '-p',f'max_session_distance_m:={0.0 if args.to_goal else 0.10}',
                '-p',f'max_session_duration_s:={0.0 if args.to_goal else 12.0}',
                '-p','path_timeout_s:=0.6','-p','readiness_timeout_s:=0.6',
            ], start_new_session=True)
            spin(3)
            if state.get('last_executor_state') != 'DISABLED':
                raise RuntimeError('Disabled executor did not come online')
        p = state['pose'][0].pose.pose
        q = p.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = n.get_clock().now().to_msg()
        goal.pose.position.x = p.position.x + math.cos(yaw)
        goal.pose.position.y = p.position.y + math.sin(yaw)
        if args.goal:
            goal.pose.position.x, goal.pose.position.y = args.goal
        goal.pose.orientation.w = 1.0
        sent = time.monotonic()
        goal_pub.publish(goal)
        print('GOAL', goal.pose.position.x, goal.pose.position.y, flush=True)
        spin(1 if args.execute else args.observe_seconds)
        def candidate_valid():
            if not ('path' in state and time.monotonic()-state['path'][1]<.6
                    and len(state['path'][0].poses)>1 and 'feas' in state
                    and json.loads(state['feas'][0].data).get('candidate_review_ready') is True):
                return False
            planned_goal=state['path'][0].poses[-1].pose.position
            return math.hypot(planned_goal.x-goal.pose.position.x,
                              planned_goal.y-goal.pose.position.y)<.12
        valid = candidate_valid()
        # Cold planner/monitor startup is asynchronous; keep the executor
        # disabled while waiting for a fresh, reviewed candidate.
        wait_end=time.monotonic()+30.
        while args.execute and not valid and time.monotonic()<wait_end:
            spin(.1)
            valid=candidate_valid()
        events = [(round(t-sent,3), size) for t,size in path_events if t>=sent]
        print('PRECHECK', json.dumps({'ready':ready(),'candidate_valid':valid,'path_events':events,
            'first_valid_path_delay_s':next((t for t,size in events if size>1),None),
            'cbf_enabled':not args.scan_only}), flush=True)
        if not valid or not ready():
            raise RuntimeError('Candidate validity/runtime check failed')
        if not args.execute:
            print('CHECK_ONLY; no executor enabled', flush=True)
            return 0
        active = True
        initial_map=(state['pose'][0].pose.pose.position.x,state['pose'][0].pose.pose.position.y)
        previous = None
        enable_pub.publish(Bool(data=True))
        end = time.monotonic()+(args.max_test_seconds if args.to_goal else 13)
        while time.monotonic()<end:
            rclpy.spin_once(n, timeout_sec=.05)
            if 'path' in state and len(state['path'][0].poses)>1:
                planned_goal=state['path'][0].poses[-1].pose.position
                if math.hypot(planned_goal.x-goal.pose.position.x,planned_goal.y-goal.pose.position.y)>.12:
                    print('STOP: planner changed requested goal',flush=True)
                    break
            # Global localization is the position authority for full-route
            # navigation. Report raw odom separately: its current integration
            # is inconsistent with its own measured twist by approximately 2x.
            travel_guard=max(map_net_displacement,command_distance) if args.to_goal else measured
            if child.poll() is not None or travel_guard>=(args.max_travel_m if args.to_goal else .17) or cmd_max>.0301:
                break
            if 'executor' in state and json.loads(state['executor'][0].data).get('state') in ('SESSION_COMPLETE','ARRIVED'):
                break
        current = state['pose'][0].pose.pose.position
        print('RESULT', json.dumps({'measured_travel_m':measured,'max_cmd_speed_mps':cmd_max,
                                   'odom_net_displacement_m':odom_net_displacement,
                                   'map_net_displacement_m':map_net_displacement,
                                   'command_distance_m':command_distance,
                                   'path_publish_count':len(path_events),
                                   'path_geometry_change_count':len(path_changes),
                                   'path_geometry_change_intervals_s':[round(b-a,3) for a,b in zip(path_changes,path_changes[1:])][-30:],
                                   'goal_error_m':math.hypot(current.x-goal.pose.position.x,current.y-goal.pose.position.y),
                                   'last_executor':state.get('last_executor_state')}), flush=True)
    finally:
        if child is not None:
            enable_pub.publish(Bool(data=False))
            import os
            try:
                os.killpg(child.pid,signal.SIGINT)
                child.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if child.poll() is None:
                    os.killpg(child.pid,signal.SIGTERM)
                    child.wait(timeout=3)
            for _ in range(5):
                zero_pub.publish(Twist())
                rclpy.spin_once(n, timeout_sec=.05)
            print('EXECUTOR_STOPPED_AND_ZERO_SENT', flush=True)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
