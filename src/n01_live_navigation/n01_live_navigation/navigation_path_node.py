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
from .path_pairing import PathReviewPairs


class NavigationPathNode(Node):
    def __init__(self):
        super().__init__('n01_navigation_path')
        self.pairs=PathReviewPairs()
        self.reference = self.local = self.feasibility = None
        self.local_rx = self.feas_rx = 0.0
        self.last_error = None
        self.reference_goal_stamp_ns = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/planning/global_path', self.on_reference, qos)
        self.create_subscription(Path, '/n01/live_navigation/hive2_candidate_path', self.on_local, 10)
        self.create_subscription(String, '/n01/live_navigation/hive2_feasibility', self.on_feasibility, 10)
        self.path_pub = self.create_publisher(Path, '/n01/navigation/path', qos)
        self.status_pub = self.create_publisher(String, '/n01/navigation/path_status', 10)
        self.create_timer(.1, self.tick)

    def on_reference(self, msg):
        if msg.poses:
            old=self.reference.poses[-1].pose.position if self.reference and self.reference.poses else None
            new=msg.poses[-1].pose.position
            if old is None or ((new.x-old.x)**2+(new.y-old.y)**2+(new.z-old.z)**2)>.0004:
                self.reference_goal_stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.reference = msg

    def on_local(self, msg):
        self.local, self.local_rx = msg, time.monotonic()
        self.pairs.put_path(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,msg,self.local_rx)

    def on_feasibility(self, msg):
        try:
            self.feasibility = json.loads(msg.data)
            if not isinstance(self.feasibility, dict):
                self.feasibility = None
        except ValueError:
            self.feasibility = None
        self.feas_rx = time.monotonic()
        if isinstance(self.feasibility,dict):self.pairs.put_review(self.feasibility,self.feas_rx)

    def tick(self):
        now = time.monotonic()
        try:
            if (self.feasibility and now-self.feas_rx <= .6 and
                    self.feasibility.get('kinematic_failures')):
                reasons = ','.join(f"{item['reason']}:{item['observed']:.3f}>{item['limit']:.3f}"
                                   for item in self.feasibility['kinematic_failures'])
                raise ValueError('local_kinematics_rejected:' + reasons)
            if (not self.reference or not self.local or now-self.local_rx > .6
                    or now-self.feas_rx > .6 or not self.feasibility
                    or self.feasibility.get('candidate_review_ready') is not True):
                raise ValueError('waiting_for_validated_local_segment')
            local=self.local;f=self.feasibility
            if hasattr(self,'pairs'):
                pair=self.pairs.newest(now)
                if pair is None:raise ValueError('waiting_for_matching_candidate_review')
                local,f=pair
            if self.reference.header.frame_id != 'map' or local.header.frame_id != 'map':
                raise ValueError('frame_mismatch')
            local_stamp = local.header.stamp.sec*10**9+local.header.stamp.nanosec
            if f.get('candidate_stamp_ns') != local_stamp:
                raise ValueError('waiting_for_matching_candidate_review')
            stamp = f['trajectory_start_time']
            start_ns = stamp['sec']*10**9+stamp['nanosec']
            ref_ns = self.reference_goal_stamp_ns
            if start_ns < ref_ns-300_000_000:
                raise ValueError('local_segment_from_previous_goal')
            age = (self.get_clock().now().nanoseconds-start_ns)/1e9
            if not -.2 <= age <= float(f['trajectory_duration_s']):
                raise ValueError('trajectory_window_expired')
            xy = lambda msg: [(p.pose.position.x,p.pose.position.y) for p in msg.poses]
            merged, local_count = splice_path(xy(self.reference), xy(local), reference_tail_only=True)
            prefix = min(local_count, int(f.get('validated_prefix_points', local_count)))
            if prefix < 3:
                raise ValueError('validated_prefix_too_short')
            if f.get('execution_stage')=='rotation' and prefix>=local_count:
                raise ValueError('rotation_plan_must_not_authorize_forward_preview')
            suffix_count = len(merged)-local_count
            output = Path()
            output.header.frame_id = 'map'
            output.header.stamp = self.get_clock().now().to_msg()
            suffix = self.reference.poses[-suffix_count:] if suffix_count else []
            output.poses = copy.deepcopy(local.poses+suffix)
            for p in output.poses:
                p.header = output.header
            self.path_pub.publish(output)
            self.status_pub.publish(String(data=json.dumps({
                'valid': True, 'validated_prefix_points':prefix,
                'points':len(output.poses), 'trajectory_id':f['trajectory_id'],
                'candidate_stamp_ns':f['candidate_stamp_ns'],
                'goal_stamp_ns':ref_ns, 'remaining_trajectory_s':float(f['trajectory_duration_s'])-age,
                'path_stamp_ns':output.header.stamp.sec*10**9+output.header.stamp.nanosec,
                'execution_region':('rotation_only_until_replanned'if f.get('execution_stage')=='rotation'else'validated_prefix_only'),
                'execution_review':f,
                'suffix_requires_local_replanning':prefix<len(output.poses),
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
