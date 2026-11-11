"""One public navigation path, with an explicitly validated execution prefix."""
import copy
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path
from std_msgs.msg import String
from .navigation_path import splice_path


class NavigationPathNode(Node):
    def __init__(self):
        super().__init__('n01_navigation_path')
        self.reference = self.local = self.feasibility = None
        self.local_rx = self.feas_rx = 0.0
        self.last_error = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/planning/global_path', self.on_reference, qos)
        self.create_subscription(Path, '/n01/live_navigation/hive2_candidate_path', self.on_local, 10)
        self.create_subscription(String, '/n01/live_navigation/hive2_feasibility', self.on_feasibility, 10)
        self.path_pub = self.create_publisher(Path, '/n01/navigation/path', qos)
        self.status_pub = self.create_publisher(String, '/n01/navigation/path_status', 10)
        self.create_timer(.1, self.tick)

    def on_reference(self, msg):
        self.reference = msg

    def on_local(self, msg):
        self.local, self.local_rx = msg, time.monotonic()

    def on_feasibility(self, msg):
        try:
            self.feasibility = json.loads(msg.data)
            if not isinstance(self.feasibility, dict):
                self.feasibility = None
        except ValueError:
            self.feasibility = None
        self.feas_rx = time.monotonic()

    def tick(self):
        now = time.monotonic()
        try:
            if (not self.reference or not self.local or now-self.local_rx > .6
                    or now-self.feas_rx > .6 or not self.feasibility
                    or self.feasibility.get('candidate_review_ready') is not True):
                raise ValueError('waiting_for_validated_local_segment')
            if self.reference.header.frame_id != 'map' or self.local.header.frame_id != 'map':
                raise ValueError('frame_mismatch')
            f = self.feasibility
            stamp = f['trajectory_start_time']
            start_ns = stamp['sec']*10**9+stamp['nanosec']
            ref_ns = self.reference.header.stamp.sec*10**9+self.reference.header.stamp.nanosec
            if start_ns < ref_ns-300_000_000:
                raise ValueError('local_segment_from_previous_goal')
            age = (self.get_clock().now().nanoseconds-start_ns)/1e9
            if not -.2 <= age <= float(f['trajectory_duration_s']):
                raise ValueError('trajectory_window_expired')
            xy = lambda msg: [(p.pose.position.x,p.pose.position.y) for p in msg.poses]
            merged, prefix = splice_path(xy(self.reference), xy(self.local))
            suffix_count = len(merged)-prefix
            output = Path()
            output.header.frame_id = 'map'
            output.header.stamp = self.get_clock().now().to_msg()
            suffix = self.reference.poses[-suffix_count:] if suffix_count else []
            output.poses = copy.deepcopy(self.local.poses+suffix)
            for p in output.poses:
                p.header = output.header
            self.path_pub.publish(output)
            self.status_pub.publish(String(data=json.dumps({
                'valid': True, 'validated_prefix_points':prefix,
                'points':len(output.poses), 'trajectory_id':f['trajectory_id'],
                'goal_stamp_ns':ref_ns, 'remaining_trajectory_s':float(f['trajectory_duration_s'])-age,
                'path_stamp_ns':output.header.stamp.sec*10**9+output.header.stamp.nanosec,
                'execution_region':'validated_prefix_only',
            })))
            self.last_error = None
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            reason = str(exc)
            self.status_pub.publish(String(data=json.dumps({'valid':False,'reason':reason})))
            if self.last_error != reason:
                empty = Path(); empty.header.frame_id='map'
                empty.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(empty)
                self.last_error = reason


def main(args=None):
    rclpy.init(args=args)
    node = NavigationPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
