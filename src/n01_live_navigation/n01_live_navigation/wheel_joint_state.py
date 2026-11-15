"""Translate real CAN wheel feedback for the existing robot_state_publisher.

Publishes only the four continuous wheel joints; no leg joints, TF, fabricated
positions, hardware commands or retransmission after source data stops.
"""
import copy
import math
import os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from control_msgs.msg import DynamicJointState
from sensor_msgs.msg import JointState

WHEELS=('wheel_fl','wheel_fr','wheel_rl','wheel_rr')


def convert(msg):
    if len(msg.joint_names)!=len(msg.interface_values) or len(set(msg.joint_names))!=len(msg.joint_names):
        raise ValueError('invalid CAN joint layout')
    source={}
    for name,item in zip(msg.joint_names,msg.interface_values):
        if len(item.interface_names)!=len(item.values) or len(set(item.interface_names))!=len(item.interface_names):
            raise ValueError('invalid CAN interface layout')
        source[name]=dict(zip(item.interface_names,item.values))
    if any(name not in source or 'position' not in source[name] for name in WHEELS):
        raise ValueError('missing actual wheel position feedback')
    out=JointState();out.header=copy.deepcopy(msg.header);out.name=list(WHEELS)
    out.position=[float(source[name]['position']) for name in WHEELS]
    for field in ('velocity','effort'):
        if all(field in source[name] for name in WHEELS):
            setattr(out,field,[float(source[name][field])for name in WHEELS])
    if not all(math.isfinite(v)for v in out.position+out.velocity+out.effort):
        raise ValueError('nonfinite CAN wheel feedback')
    return out


class WheelJointState(Node):
    def __init__(self):
        super().__init__('n01_wheel_joint_state')
        self.declare_parameter('owner_pid',0)
        self.owner=int(self.get_parameter('owner_pid').value)
        self.owner_start=self.owner_identity() if self.owner else None
        self.publisher=self.create_publisher(JointState,'/joint_states',10)
        self.sub=self.create_subscription(DynamicJointState,'/wheels_can/joint_state',self.on_state,qos_profile_sensor_data)
        self.create_timer(.5,self.check_owner)

    def check_owner(self):
        # Optional standalone commissioning instance follows the already
        # running SLAM launcher without taking ownership of its hardware.
        if self.owner:
            if self.owner_identity()!=self.owner_start or self.owner_start is None:rclpy.shutdown()

    def owner_identity(self):
        try:return (Path('/proc')/str(self.owner)/'stat').read_text().rsplit(')',1)[1].split()[19]
        except (OSError,IndexError):return None

    def on_state(self,msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age=(self.get_clock().now().nanoseconds-stamp)/1e9
        if stamp<=0 or not -.1<=age<=.5:return
        try:self.publisher.publish(convert(msg))
        except ValueError as exc:self.get_logger().warning(str(exc),throttle_duration_sec=5.)


def main(args=None):
    rclpy.init(args=args);node=WheelJointState()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
