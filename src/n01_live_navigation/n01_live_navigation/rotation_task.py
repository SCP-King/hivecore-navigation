"""Collision-checked rotation task sharing the normal executor's hard gates."""
from .arrival_confirmation import measurement_confirms, prediction_metadata
from .odom_qos import odometry_integration_qos
import json,math,time
from collections import deque
import numpy as np
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from .core import decode_pointcloud_xyz,transform_points,interpolate_pose,compose_transform
from .rotation_guard import rotation_arc_clear
from .angular_travel import AngularTravel


def setup(node):
    from .footprint_client import FootprintClient
    node.rotation_footprint=FootprintClient(node)
    node.async_rotation_sweep=True
    node.rotation_anchor=None;node.rotation_settled=None;node.rotation_began=None
    node.rotation_static=None;node.rotation_plane=None;node.rotation_plane_rx=0.
    node.rotation_static_rx=0.;node.rotation_map_ready=False;node.rotation_map_status_rx=0.
    node.rotation_pose_cache=deque(maxlen=64)
    node.rotation_raw=deque(maxlen=10);node.rotation_odom=None;node.rotation_odom_rx=0.
    node.rotation_odom_anchor=None
    node.rotation_travel=AngularTravel();node.rotation_segment_anchor=None
    node.rotation_stop_since=None;node.rotation_linear_speed=None;node.rotation_yaw_speed=None
    node.rotation_tf=Buffer();node.rotation_listener=TransformListener(node.rotation_tf,node)
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def static(msg):
        if msg.header.frame_id!='map':
            node.rotation_static=None;return
        try:
            decoded=(np.empty((0,3)) if msg.width==0 and len(msg.data)==0
                                  else decode_pointcloud_xyz(msg,1000000,None))
            from contextlib import nullcontext
            scheduling=getattr(node,'scheduling',None)
            with scheduling.lock if scheduling is not None else nullcontext():
                node.rotation_static=decoded
                node.rotation_static_rx=time.monotonic()
        except ValueError:node.rotation_static=None
    def map_status(msg):
        node.rotation_map_ready=False
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):return
            node.rotation_map_ready=data.get('ready') is True
            node.rotation_map_status_rx=time.monotonic()
        except (ValueError,TypeError):pass
    def plane(msg):
        node.rotation_plane=None
        try:
            data=json.loads(msg.data);p=data['plane']
            if data['state']!='ground_projection_ready':return
            abc=np.array([p['a'],p['b'],p['c']],float)
            if not np.isfinite(abc).all() or math.hypot(*abc[:2])>math.tan(math.radians(2)) or p['residual']>.04:return
            node.rotation_plane=abc;node.rotation_plane_rx=time.monotonic()
        except (ValueError,KeyError,TypeError):pass
    def raw(msg):node.rotation_raw.append(msg)
    def scan_pose(msg):
        if msg.header.frame_id!='map' or msg.child_frame_id!='base_link':return
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if node.rotation_pose_cache and stamp<=node.rotation_pose_cache[-1][0]:return
        node.rotation_pose_cache.append((stamp,(p.x,p.y,p.z),(q.x,q.y,q.z,q.w)))
    def odom(msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        age=(node.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link' or not -.1<=age<=.3:return
        twist=msg.twist.twist
        velocity=[twist.linear.x,twist.linear.y,twist.angular.z]
        if not all(math.isfinite(v) for v in velocity):return
        values=[p.x,p.y,q.x,q.y,q.z,q.w]
        if not all(math.isfinite(v)for v in values) or abs(sum(v*v for v in values[2:])-1)>.01:return
        try:
            measured_angle=node.rotation_travel.update(
                msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,velocity[2])
        except ValueError:return
        node.rotation_linear_speed=math.hypot(*velocity[:2]);node.rotation_yaw_speed=velocity[2]
        node.rotation_odom=measured_angle;node.rotation_odom_rx=time.monotonic()
    node.rotation_subs=[node.create_subscription(Odometry,'/n01/live_navigation/scan_body_pose',scan_pose,qos_profile_sensor_data),node.create_subscription(PointCloud2,'/grid_map/occupancy',static,qos_profile_sensor_data),
        node.create_subscription(String,'/grid_map/static_overlay_status',map_status,latched),
        node.create_subscription(PointCloud2,'/livox/lidar',raw,qos_profile_sensor_data),
        node.create_subscription(String,'/n01/ground_reference/status',plane,10),
        node.create_subscription(Odometry,'/odom',odom,odometry_integration_qos())]


def live_points(node,with_stamp=False):
    # Raw full-azimuth data avoids the preprocessor's rear azimuth crop.
    # Delayed TF can match an earlier buffered scan; latest-TF substitution is forbidden.
    for msg in reversed(node.rotation_raw):
        age=(node.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if not -.1<=age<=.6:continue
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        try:
            predicted=prediction_metadata(getattr(node,'readiness',None)) is not None
            if predicted:
                stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                body=interpolate_pose(list(node.rotation_pose_cache),stamp,.2)
                if body is None:continue
                tf=node.rotation_tf.lookup_transform('base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp))
            else:
                tf=node.rotation_tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
            p=decode_pointcloud_xyz(msg,100000,None)
            p=p[np.linalg.norm(p,axis=1)>.1]
            if len(p)<30:continue
            t,q=tf.transform.translation,tf.transform.rotation
            sensor=((t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
            if predicted:sensor=compose_transform(body[0],body[1],sensor[0],sensor[1])
            points=transform_points(p,sensor[0],sensor[1])
            return (points,stamp) if with_stamp else points
        except Exception:continue
    raise ValueError('fresh raw lidar with scan-time TF unavailable')


def control(node,now):
    x,y,yaw=node.pose
    if node.rotation_anchor is None:
        linear=getattr(node,'rotation_linear_speed',None)
        angular=getattr(node,'rotation_yaw_speed',None)
        if (linear is None or angular is None or now-node.rotation_odom_rx>.3
                or linear>.01 or abs(angular)>.02):
            node.rotation_stop_since=None
            node._zero('STOPPING_FOR_TURN','waiting for measured chassis stop');return
        if getattr(node,'rotation_stop_since',None) is None:
            node.rotation_stop_since=now
        if now-node.rotation_stop_since<.3:
            node._zero('STOPPING_FOR_TURN','confirming measured stop for 0.3 seconds');return
        node.rotation_anchor=(x,y,yaw);node.rotation_began=now
        node.rotation_odom_anchor=node.rotation_odom
        node.rotation_segment_anchor=getattr(getattr(node,'rotation_travel',None),'segment',0)
    if now-node.rotation_began>180:
        node.finished=True;node.completion_state='ROTATION_TIMEOUT';node._zero('ROTATION_TIMEOUT','rotation exceeded 180s');return
    if math.hypot(x-node.rotation_anchor[0],y-node.rotation_anchor[1])>.04:
        node.finished=True;node.completion_state='ROTATION_DRIFT';node._zero('ROTATION_DRIFT','body translated over 4cm');return
    if node.rotation_odom is None or now-node.rotation_odom_rx>.3:
        node.rotation_settled=None;node._zero('ROTATION_INPUT_STALE','fresh wheel odometry required');return
    if getattr(node,'rotation_segment_anchor',0) != getattr(getattr(node,'rotation_travel',None),'segment',0):
        node.rotation_settled=None;node._zero('ROTATION_INPUT_STALE','angular velocity integration gap; stop and replan');return
    if node.rotation_odom_anchor is None:node.rotation_odom_anchor=node.rotation_odom
    wrap=lambda v:math.atan2(math.sin(v),math.cos(v))
    if abs(wrap(wrap(yaw-node.rotation_anchor[2])-wrap(node.rotation_odom-node.rotation_odom_anchor)))>math.radians(5):
        node.rotation_settled=None;node._zero('ROTATION_POSE_MISMATCH','map yaw change and measured angular travel disagree');return
    try:
        if (node.rotation_static is None or now-node.rotation_static_rx>1.
                or not node.rotation_map_ready or node.rotation_plane is None or now-node.rotation_plane_rx>2):
            raise ValueError('static map or ground plane unavailable/stale')
        xyz=(x,y,node.pose_z)
        if getattr(node,'async_rotation_sweep',False):
            from .rotation_sweep_job import poll
            clear=poll(node,now,live_points)
            if clear is None:
                node._zero('ROTATION_CHECKING','waiting for current swept-space certificate: '+getattr(node,'rotation_sweep_error',''));return
            if not clear:raise ValueError('obstacle inside requested rotation swept footprint')
        else:
            if not rotation_arc_clear(node.rotation_static,xyz,node.rotation_plane,yaw,node.target_yaw,min_observations=0,footprint=node.rotation_footprint.get()):
                raise ValueError('static obstacle inside requested rotation swept footprint')
            if not rotation_arc_clear(live_points(node),xyz,node.rotation_plane,yaw,node.target_yaw,footprint=node.rotation_footprint.get()):
                raise ValueError('live obstacle inside requested rotation swept footprint')
    except ValueError as exc:
        node.rotation_settled=None;node._zero('ROTATION_BLOCKED',str(exc));return
    error=wrap(node.target_yaw-yaw);node.rotation_error=error
    if abs(error)<math.radians(1):
        node._zero('VERIFYING_ARRIVAL','checking yaw settled')
        if node.rotation_settled is None:
            node.rotation_settled=now
            node.rotation_source_ns=(node.get_clock().now().nanoseconds
                if prediction_metadata(getattr(node,'readiness',None)) is not None else 0)
        confirmed=measurement_confirms(getattr(node,'readiness',None),
            node.rotation_source_ns,yaw=node.target_yaw)
        if now-node.rotation_settled>=.5 and confirmed:
            if getattr(node,'path_turn_active',False):
                node.rotation_complete=True;node._zero('ALIGN_PATH','path heading settled')
            else:
                node.finished=True;node.completion_state='ARRIVED';node._zero('ARRIVED','rotation target settled')
        return
    node.rotation_settled=None
    omega=max(-min(.08,node.max_yaw),min(min(.08,node.max_yaw),.7*error))
    valid,reason=node._chassis_command_valid(0.,omega,now,0.)
    if not valid:node._zero('CHASSIS_KINEMATICS_BLOCKED',reason);return
    node.cmd_linear_x=node.cmd_linear_y=node.speed=0.;node.cmd_angular_z=omega
    if node.dry_run:node._status('DRY_RUN','rotation checked; command suppressed');return
    from geometry_msgs.msg import Twist
    cmd=Twist();cmd.angular.z=omega;node.cmd_pub.publish(cmd);node._status('ROTATING','requested angle swept footprint checked')
