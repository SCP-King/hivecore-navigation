"""Navigation-local measured-twist integration; raw chassis odometry stays unchanged."""
import math
import numpy as np
from .core import normalize_quaternion, rotate_translate, quaternion_multiply


class TwistOdometry:
    def __init__(self,max_gap=.08):
        self.max_gap=max_gap;self.last=None;self.position=None;self.segment_start=0
        self.yaw=None;self.last_yaw_rate=None;self.max_observed_gap=0.

    def update(self,stamp_ns,raw_position,orientation,velocity,angular_velocity=None):
        q=normalize_quaternion(orientation)
        if len(velocity)!=3 or not all(math.isfinite(x) for x in velocity):
            raise ValueError('odom_twist_invalid')
        if np.linalg.norm(velocity)>3.:raise ValueError('odom_twist_outside_physical_budget')
        if self.last is not None and stamp_ns<=self.last[0]:
            raise ValueError('odom_timestamp_not_advancing')
        if angular_velocity is not None:
            if (len(angular_velocity)!=3 or not all(math.isfinite(x) for x in angular_velocity)
                    or np.linalg.norm(angular_velocity)>5.):
                raise ValueError('odom_angular_twist_invalid')
        elif self.last_yaw_rate is not None:
            raise ValueError('odom_angular_twist_missing')
        if self.last is not None:self.max_observed_gap=max(self.max_observed_gap,(stamp_ns-self.last[0])/1e9)
        restart=self.last is None or (stamp_ns-self.last[0])/1e9>self.max_gap
        if self.last is not None and angular_velocity is None:
            dot=min(1.,abs(float(np.dot(q,self.last[2]))))
            restart=restart or 2*math.acos(dot)>.25
        restart=restart or (angular_velocity is not None and self.last_yaw_rate is None)
        if angular_velocity is not None:
            # R2 planar heading comes from measured body angular velocity. Raw
            # /odom yaw has jumps inconsistent with both SLAM and measured twist.
            # Retain raw roll/pitch by applying only a odom-Z yaw correction.
            x,y,z,w=q
            raw_yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            yaw_rate=float(angular_velocity[2])
            if restart:
                self.yaw=raw_yaw
            else:
                dt=(stamp_ns-self.last[0])/1e9
                self.yaw+=.5*(self.last_yaw_rate+yaw_rate)*dt
                self.yaw=math.atan2(math.sin(self.yaw),math.cos(self.yaw))
            delta=math.atan2(math.sin(self.yaw-raw_yaw),math.cos(self.yaw-raw_yaw))
            q=normalize_quaternion(quaternion_multiply((0.,0.,math.sin(delta/2),math.cos(delta/2)),q))
            self.last_yaw_rate=yaw_rate
        world=np.asarray(rotate_translate(velocity,(0.,0.,0.),q))
        if restart:
            self.position=np.asarray(raw_position,dtype=float)
            self.segment_start=int(stamp_ns)
        else:
            dt=(stamp_ns-self.last[0])/1e9
            self.position=self.position+.5*(self.last[1]+world)*dt
        self.last=(int(stamp_ns),world,q)
        return tuple(self.position),q
