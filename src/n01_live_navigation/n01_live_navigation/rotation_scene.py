"""Full-azimuth scene at scan time; no motion task or actuator output."""
from collections import deque
import time
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from .core import decode_pointcloud_xyz, interpolate_pose, compose_transform, transform_points
from .arrival_confirmation import prediction_metadata


def setup(node):
    node.rotation_raw=deque(maxlen=10)
    node.rotation_pose_cache=deque(maxlen=64)
    node.readiness=None
    node.rotation_tf=Buffer()
    node.rotation_listener=TransformListener(node.rotation_tf,node)
    def raw(msg):node.rotation_raw.append(msg)
    def scan_pose(msg):
        if msg.header.frame_id!='map' or msg.child_frame_id!='base_link':return
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if node.rotation_pose_cache and stamp<=node.rotation_pose_cache[-1][0]:return
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        node.rotation_pose_cache.append((stamp,(p.x,p.y,p.z),(q.x,q.y,q.z,q.w)))
    def health(msg):
        import json
        try:node.readiness=json.loads(msg.data)
        except ValueError:node.readiness=None
    node.scene_subs=[node.create_subscription(PointCloud2,'/livox/lidar',raw,qos_profile_sensor_data),
        node.create_subscription(Odometry,'/n01/live_navigation/scan_body_pose',scan_pose,qos_profile_sensor_data),
        node.create_subscription(String,'/n01/live_navigation/system_health',health,10)]


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
            if getattr(node,'scene_perception',None) is not None:
                if getattr(node,'ground_plane',None) is None:continue
                node.scene_perception.next_ground_plane=tuple(node.ground_plane)
                if not predicted:
                    body_tf=node.rotation_tf.lookup_transform('map','base_link',Time.from_msg(msg.header.stamp))
                    bt,bq=body_tf.transform.translation,body_tf.transform.rotation
                    body=((bt.x,bt.y,bt.z),(bq.x,bq.y,bq.z,bq.w))
                points,ground=node.scene_perception.process(points,body,stamp/1e9)
                points=points[~ground]
            return (points,stamp) if with_stamp else points
        except Exception:continue
    raise ValueError('fresh raw lidar with scan-time TF unavailable')

