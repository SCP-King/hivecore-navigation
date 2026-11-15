"""Timestamp-aware odometry prediction with bounded, causal SLAM correction.

Raw observations are never restamped. Both scans and live control evaluate the
same correction history at their own real timestamps; no velocity extrapolation
is performed beyond the last measured odometry sample.
"""
from bisect import bisect_right
import math
import numpy as np
from .core import motion_compensated_pose, quaternion_slerp


def angle(a, b):
    return 2*math.acos(min(1., abs(float(np.dot(a, b)))))


class DelayedPoseFusion:
    def __init__(self, horizon=2., travel_limit=.12, turn_limit=.20,
                 correction_time=.8, correction_speed=.04,
                 correction_yaw_rate=.08, jump_distance=.15, jump_angle=.17):
        values=(horizon,travel_limit,turn_limit,correction_time,correction_speed,
                correction_yaw_rate,jump_distance,jump_angle)
        if not all(math.isfinite(v) and v>0 for v in values):
            raise ValueError('invalid prediction bounds')
        self.horizon=horizon;self.travel_limit=travel_limit;self.turn_limit=turn_limit
        self.correction_time=correction_time;self.correction_speed=correction_speed
        self.correction_yaw_rate=correction_yaw_rate
        self.jump_distance=jump_distance;self.jump_angle=jump_angle
        self.reset()

    def reset(self):
        self.records=[];self.last_source=0;self.anchor=None;self.fault=''
        self.innovation_m=0.;self.innovation_rad=0.

    def accept(self, source_ns, map_anchor, odom_anchor, effective_ns, odom_effective):
        if self.fault:raise ValueError(self.fault)
        if source_ns<=self.last_source:return False
        if not 0<=effective_ns-source_ns<=self.horizon*1e9:
            raise ValueError('localization_outside_prediction_horizon')
        target=motion_compensated_pose(map_anchor,odom_anchor,odom_effective)
        previous=self._evaluate(effective_ns,odom_effective) if self.records else target
        distance=float(np.linalg.norm(np.subtract(target[0],previous[0])))
        rotation=angle(target[1],previous[1])
        self.innovation_m=distance;self.innovation_rad=rotation
        if self.records and (distance>self.jump_distance or rotation>self.jump_angle):
            self.fault='localization_correction_jump_requires_reset'
            raise ValueError(self.fault)
        duration=max(self.correction_time,
                     (distance+2*self.travel_limit*rotation)/self.correction_speed,
                     rotation/self.correction_yaw_rate) if self.records else 0.
        self.records.append((int(effective_ns),odom_effective,previous,target,duration))
        self.records=self.records[-128:]
        self.last_source=int(source_ns);self.anchor=(int(source_ns),odom_anchor)
        self.innovation_m=distance;self.innovation_rad=rotation
        return True

    def _evaluate(self, stamp_ns, odom):
        index=bisect_right([r[0] for r in self.records],stamp_ns)-1
        if index<0:raise ValueError('prediction_before_correction_history')
        start,reference,previous,target,duration=self.records[index]
        u=1. if duration==0 else min(1.,max(0.,(stamp_ns-start)/1e9/duration))
        # Interpolate body poses about the effective body origin, not absolute
        # map<-odom translations (which amplify rotation at distant odom origins).
        a=motion_compensated_pose(previous,reference,odom)
        b=motion_compensated_pose(target,reference,odom)
        p=tuple((1-u)*x+u*y for x,y in zip(a[0],b[0]))
        return p,quaternion_slerp(a[1],b[1],u)

    def pose(self, stamp_ns, odom):
        if self.fault:raise ValueError(self.fault)
        if self.anchor is None:raise ValueError('prediction_waiting_localization_anchor')
        age=(stamp_ns-self.anchor[0])/1e9
        if not -.2<=age<=self.horizon:raise ValueError('prediction_horizon_exceeded')
        distance=float(np.linalg.norm(np.subtract(odom[0],self.anchor[1][0])))
        rotation=angle(odom[1],self.anchor[1][1])
        if distance>self.travel_limit:raise ValueError('prediction_distance_exceeded')
        if rotation>self.turn_limit:raise ValueError('prediction_turn_exceeded')
        return self._evaluate(stamp_ns,odom)
