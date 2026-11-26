"""Map floor -> dynamic ground footprint. No assumed leg height or control IO."""
import copy
import json
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from .core import decode_pointcloud_xyz
from collections import deque
from .ground_plane import estimate_ground
from .urdf_projection import RobotProjection
from sensor_msgs.msg import JointState
from .ground_reference import project_footprint


class GroundReferenceNode(Node):
    def __init__(self):
        super().__init__('n01_ground_reference')
        defaults = {'pose_topic': '/n01/localization/pose',
                    'map_topics': ['/cloud_map', '/localization_map'],
                    'latched_map_topics': ['/n01/fast_lio/reference_map'],
                    'map_frame': 'map', 'body_frame': 'base_link',
                    'footprint_frame': 'base_footprint', 'fit_radius_m': 6.,
                    'fit_period_s': 1., 'max_plane_age_s': 8.,
                    'max_pose_age_s': 1.5, 'max_residual_m': .04,
                    'publish_tf': True,
                    'accept_fixed_footprint_tf': False}
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        if (len(set(self.cfg[k] for k in ('map_frame','body_frame','footprint_frame'))) != 3
                or not all(float(self.cfg[k]) > 0 for k in (
                    'fit_radius_m','fit_period_s','max_plane_age_s','max_pose_age_s','max_residual_m'))):
            raise ValueError('invalid ground reference configuration')
        self.pose = self.plane = self.reference_xy = None
        self.plane_time, self.last_fit = 0., -float('inf')
        self.fit_error = ""
        self.fixed_conflict, self.rsp_seen = False, False
        self.reason = 'waiting_for_body_pose_and_map_floor'
        self.tf_pub = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/n01/ground_reference/pose', 10)
        self.status_pub = self.create_publisher(String, '/n01/ground_reference/status', 10)
        self.pose_sub = self.create_subscription(Odometry, self.cfg['pose_topic'], self.on_pose, 20)
        cloud_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.map_subs = [self.create_subscription(PointCloud2, t, self.on_map, cloud_qos)
                         for t in self.cfg['map_topics']]
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_sub = self.create_subscription(TFMessage, '/tf_static', self.on_static, static_qos)
        self.latched_subs = [self.create_subscription(PointCloud2, t, self.on_latched_map, static_qos)
                             for t in self.cfg['latched_map_topics']]
        self.ground_frames=deque(maxlen=4)
        self.last_live_cloud_monotonic = 0.0
        self.cached_latched_key = None
        self.cached_latched_points = None
        self.live_sub=self.create_subscription(PointCloud2,'/cloud',self.on_live_cloud,cloud_qos)
        self.model=None;self.joints={};self.joint_times={}
        self.model_sub=self.create_subscription(String,'/robot_description',self.on_model,static_qos)
        self.joint_sub=self.create_subscription(JointState,'/joint_states',self.on_joints,cloud_qos)
        self.latched_map = None
        self.create_timer(1., self.report)

    def on_live_cloud(self,msg):
        now = time.monotonic()
        if now - self.last_live_cloud_monotonic < 0.9:
            return
        age=(self.get_clock().now().nanoseconds-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e9
        if msg.header.frame_id.lstrip('/')!=self.cfg['map_frame'] or not -.2<=age<=.6:return
        self.last_live_cloud_monotonic = now
        try:self.ground_frames.append((now,decode_pointcloud_xyz(msg,100000,None)))
        except ValueError:pass

    def on_model(self,msg):
        try:self.model=RobotProjection(msg.data)
        except Exception:self.model=None

    def on_joints(self,msg):
        age=(self.get_clock().now().nanoseconds-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e9
        if not -.2<=age<=.5:return
        now=time.monotonic()
        for name,value in zip(msg.name,msg.position):
            if np.isfinite(value):self.joints[name]=value;self.joint_times[name]=now

    def on_latched_map(self, msg):
        # Saved PCD may arrive before localization pose; keep it for later fits.
        self.latched_map = msg
        self.on_map(msg)

    def on_static(self, msg):
        # RSP batches its fixed joints. A separate IMU tilt message cannot
        # establish that the desktop body model has released the footprint.
        if any(t.header.frame_id == self.cfg['body_frame'] and t.child_frame_id == 'lidar_link'
               for t in msg.transforms):
            self.rsp_seen = True
            self.fixed_conflict = (not self.cfg['accept_fixed_footprint_tf'] and any(
                t.child_frame_id == self.cfg['footprint_frame'] or
                (t.child_frame_id == self.cfg['body_frame'] and
                 t.header.frame_id == self.cfg['footprint_frame']) for t in msg.transforms))

    def on_map(self, msg):
        now = time.monotonic()
        if self.pose is None:
            return
        p = self.pose.pose.pose.position
        fit_period = self.cfg['fit_period_s']
        if (self.plane is not None and self.reference_xy is not None
                and np.linalg.norm(np.array([p.x, p.y]) - self.reference_xy) < 0.5):
            fit_period = max(fit_period, 2.5)
        if now - self.last_fit < fit_period:
            return
        self.last_fit = now
        if msg.header.frame_id.lstrip('/') != self.cfg['map_frame']:
            self.reason = 'map_cloud_frame_mismatch'
            return
        try:
            if msg is self.latched_map:
                key = (msg.header.stamp.sec, msg.header.stamp.nanosec, len(msg.data))
                if key == self.cached_latched_key and self.cached_latched_points is not None:
                    points = self.cached_latched_points
                else:
                    points = decode_pointcloud_xyz(msg, 250000, None)
                    self.cached_latched_key = key
                    self.cached_latched_points = points
            else:
                points = decode_pointcloud_xyz(msg, 250000, None)
            frames=[pt for t,pt in self.ground_frames if now-t<3.]
            if frames:points=np.vstack([points,*frames])
            hint=None
            if self.model is not None:
                joints={k:v for k,v in self.joints.items() if now-self.joint_times.get(k,0)<.5}
                q=self.pose.pose.pose.orientation
                roll=np.arctan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
                pitch=np.arcsin(np.clip(2*(q.w*q.y-q.z*q.x),-1,1))
                try:hint=self.model.floor_hint(joints,p.z,(roll,pitch))
                except ValueError:pass
            plane = estimate_ground(points, (p.x,p.y), self.cfg['fit_radius_m'], body_z=p.z,height_hint=hint)
            if plane.residual > self.cfg['max_residual_m']:
                raise ValueError('floor_residual_too_large')
            self.plane, self.reference_xy, self.plane_time = plane, np.array([p.x,p.y]), now
            self.fit_error = ""
        except ValueError as exc:
            # Keep the last measured valid ground plane while still within
            # max_plane_age_s so a single noisy/occluded frame does not wipe
            # ground projection and stall the robot. Never fall back to 0.9 m.
            if self.plane is None or (now - self.plane_time) > self.cfg['max_plane_age_s']:
                self.plane = None
                self.reason = str(exc)
            self.fit_error = str(exc)

    def on_pose(self, msg):
        if (msg.header.frame_id.lstrip('/') != self.cfg['map_frame'] or
                msg.child_frame_id.lstrip('/') != self.cfg['body_frame']):
            self.reason = 'body_pose_frame_mismatch'
            return
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if stamp <= 0 or not -.1 <= age <= self.cfg['max_pose_age_s']:
            self.reason = 'body_pose_stale'
            return
        self.pose = msg
        if not self.rsp_seen or self.fixed_conflict:
            self.reason = 'conflicting_fixed_footprint_tf' if self.fixed_conflict else 'waiting_for_body_root_model'
            return
        if self.plane is None or time.monotonic()-self.plane_time > self.cfg['max_plane_age_s']:
            self.reason = ('floor_unavailable_or_stale' +
                           (f':{self.fit_error}' if self.fit_error else ''))
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if np.linalg.norm(np.array([p.x,p.y])-self.reference_xy) > self.cfg['fit_radius_m']/2:
            self.reason = 'outside_fitted_floor_region'
            return
        ground, relative = project_footprint((p.x,p.y,p.z),(q.x,q.y,q.z,q.w),
                                           (self.plane.a,self.plane.b,self.plane.c))
        tf = TransformStamped()
        tf.header = copy.deepcopy(msg.header)
        tf.header.frame_id = self.cfg['body_frame']
        tf.child_frame_id = self.cfg['footprint_frame']
        t, r = relative
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z = map(float,t)
        tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w = map(float,r)
        if self.cfg['publish_tf']:
            self.tf_pub.sendTransform(tf)
        pose = PoseStamped()
        pose.header = copy.deepcopy(msg.header)
        t,r = ground
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = map(float,t)
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = map(float,r)
        self.pose_pub.publish(pose)
        self.reason = 'ground_projection_ready'

    def report(self):
        if self.latched_map is not None:
            self.on_map(self.latched_map)
        msg = String()
        msg.data = json.dumps({'state': self.reason, 'plane': vars(self.plane) if self.plane else None,
                               'fixed_tf_conflict': self.fixed_conflict})
        self.status_pub.publish(msg)
        if self.reason != 'ground_projection_ready':
            self.get_logger().warning(msg.data, throttle_duration_sec=5.)


def main(args=None):
    rclpy.init(args=args)
    node = GroundReferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
