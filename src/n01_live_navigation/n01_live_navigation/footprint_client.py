"""Shared fresh geometry subscription; no fallback to a smaller guessed body."""
import time
import numpy as np
from geometry_msgs.msg import PolygonStamped
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from .robot_footprint import describe

class FootprintClient:
    def __init__(self,node):
        self.node=node;self.polygon=None;self.rx=0.;self.stamp=0
        q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub=node.create_subscription(PolygonStamped,'/n01/robot_footprint',self.receive,q)
    def receive(self,m):
        self.polygon=None
        if m.header.frame_id!='base_link':return
        p=np.array([(v.x,v.y)for v in m.polygon.points])
        if len(p)<3 or len(p)>256 or not np.isfinite(p).all():return
        self.polygon=p;self.rx=time.monotonic();self.stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
    def get(self):
        age=(self.node.get_clock().now().nanoseconds-self.stamp)/1e9
        if self.polygon is None or time.monotonic()-self.rx>1. or not -.1<=age<=1.:
            raise ValueError('urdf_footprint_missing_or_stale')
        return self.polygon
    def description(self):
        try:return describe(self.get())
        except ValueError as e:return dict(model='urdf_unavailable',reason=str(e))
