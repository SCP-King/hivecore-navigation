"""One non-actuating geometry publisher for planner, reviewer, rotation and GUI."""
import time,json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import PolygonStamped,Point32
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from .urdf_projection import RobotProjection
from .robot_footprint import steering_hull,describe,sweep_joints

class RobotFootprintNode(Node):
    def __init__(self):
        super().__init__('n01_robot_footprint')
        self.model=None;self.positions={};self.times={};self.cached_key=None;self.cached=None
        q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(PolygonStamped,'/n01/robot_footprint',q)
        self.status=self.create_publisher(String,'/n01/robot_footprint/status',q)
        self.create_subscription(String,'/robot_description',self.on_model,q)
        self.create_subscription(JointState,'/joint_states',self.on_joints,qos_profile_sensor_data)
        self.create_timer(.2,self.publish)
    def on_model(self,m):
        self.model=None;self.cached_key=None
        try:self.model=RobotProjection(m.data)
        except ValueError as e:self.get_logger().error(str(e))
    def on_joints(self,m):
        now=time.monotonic();age=(self.get_clock().now().nanoseconds-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))/1e9
        if not -.1<=age<=1.:return
        for k,v in zip(m.name,m.position):
            if np.isfinite(v):self.positions[k]=v;self.times[k]=now
    def publish(self):
        msg=PolygonStamped();msg.header.frame_id='base_link';msg.header.stamp=self.get_clock().now().to_msg()
        status={'ready':False,'reason':'waiting_for_urdf_and_fresh_joints'}
        try:
            if self.model is None:raise ValueError('robot_description_unavailable')
            now=time.monotonic();j={k:v for k,v in self.positions.items()if now-self.times[k]<=1.}
            j=sweep_joints(self.model,j)
            key=tuple((k,j[k])for k in sorted(j)if not k.startswith('wheel_')and not k.endswith('_ankle_yaw'))
            if key!=self.cached_key:
                self.cached=steering_hull(self.model,j);self.cached_key=key
            for x,y in self.cached:msg.polygon.points.append(Point32(x=float(x),y=float(y),z=0.))
            status=dict(ready=True,reason='ready',**describe(self.cached))
        except ValueError as e:status['reason']=str(e)
        self.pub.publish(msg);m=String();m.data=json.dumps(status);self.status.publish(m)

def main(args=None):
    rclpy.init(args=args);node=RobotFootprintNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
