"""Offline historical TF reconstruction, without ROS, DDS or actuator APIs.

Selected Hive2 calibration is for *replay exploration only*, and NEVER asserts
that a historical SLAM localization was valid or an RSP source unique.
"""
from bisect import bisect_left
from dataclasses import dataclass
import math

SELECTED_MOUNT_HEIGHT_M = 0.90
SELECTED_LIDAR_X_M = 0.295


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def normalize(q):
    if len(q) != 4 or not all(math.isfinite(x) for x in q):
        raise ValueError("nonfinite quaternion")
    norm = math.sqrt(sum(x*x for x in q))
    if norm < 1e-8:
        raise ValueError("invalid zero quaternion")
    return tuple(x/norm for x in q)


def rotate(q, xyz):
    qx,qy,qz,qw = normalize(q)
    px,py,pz = xyz
    ax = 2*(qy*pz-qz*py)
    ay = 2*(qz*px-qx*pz)
    az = 2*(qx*py-qy*px)
    return (px+qw*ax+qy*az-qz*ay,
            py+qw*ay+qz*ax-qx*az,
            pz+qw*az+qx*ay-qy*ax)


def multiply(a,b):
    ax,ay,az,aw=normalize(a)
    bx,by,bz,bw=normalize(b)
    return normalize((aw*bx+ax*bw+ay*bz-az*by,
                      aw*by-ax*bz+ay*bw+az*bx,
                      aw*bz+ax*by-ay*bx+az*bw,
                      aw*bw-ax*bx-ay*by-az*bz))


def combine(p1,q1,p2,q2=(0.,0.,0.,1.)):
    r=rotate(q1,p2)
    return tuple(p1[i]+r[i] for i in range(3)), multiply(q1,q2)


def interpolate(a,b,alpha):
    pa,qa=a; pb,qb=b
    qa=normalize(qa); qb=normalize(qb)
    dot=sum(x*y for x,y in zip(qa,qb))
    if dot<0:
        qb=tuple(-x for x in qb); dot=-dot
    if dot>.9995:
        q=normalize(tuple((1-alpha)*x+alpha*y for x,y in zip(qa,qb)))
    else:
        angle=math.acos(max(-1.,min(1.,dot)))
        s=math.sin(angle)
        c1=math.sin((1-alpha)*angle)/s
        c2=math.sin(alpha*angle)/s
        q=normalize(tuple(c1*x+c2*y for x,y in zip(qa,qb)))
    p=tuple((1-alpha)*pa[i]+alpha*pb[i] for i in range(3))
    return p,q


@dataclass(frozen=True)
class StampedPose:
    stamp_ns: int
    translation: tuple
    quaternion: tuple


class HistoricalMapTf:
    """Interpolates map->base_footprint from historical TF, bounded by age.

    Only the exact map->base_footprint edge is accepted; no guessing map=odom.
    Result is a bag-data reconstruction, not a proof of successful localization.
    """
    def __init__(self, max_age_s=1.0, max_interval_s=2.0):
        if not (0.01 <= max_age_s <= 10.0 and 0.01 <= max_interval_s <= 20.):
            raise ValueError("invalid TF search window")
        self.max_age_ns=int(max_age_s*1e9)
        self.max_interval_ns=int(max_interval_s*1e9)
        self.times=[]
        self.poses=[]

    def add(self, stamp_ns, xyz, q):
        if stamp_ns <= 0 or not all(math.isfinite(v) for v in xyz):
            return False
        try:
            quat=normalize(q)
        except ValueError:
            return False
        self.poses.append(StampedPose(int(stamp_ns),tuple(xyz),quat))
        return True

    def finish(self):
        if not self.poses:
            self.times=[]
            return
        ordered=sorted(self.poses,key=lambda x:x.stamp_ns)
        unique=[]
        for point in ordered:
            if unique and unique[-1].stamp_ns==point.stamp_ns:
                unique[-1]=point
            else:
                unique.append(point)
        self.poses=unique
        self.times=[p.stamp_ns for p in unique]

    def lookup(self, stamp_ns):
        if not self.times or stamp_ns<=0:
            return None
        index=bisect_left(self.times,stamp_ns)
        if index < len(self.times) and self.times[index]==stamp_ns:
            p=self.poses[index]
            return p.translation,p.quaternion,0.
        if 0<index<len(self.times):
            before,after=self.poses[index-1],self.poses[index]
            gap=after.stamp_ns-before.stamp_ns
            age=min(stamp_ns-before.stamp_ns,after.stamp_ns-stamp_ns)
            if gap<=self.max_interval_ns and age<=self.max_age_ns:
                pos,quat=interpolate(
                    (before.translation,before.quaternion),
                    (after.translation,after.quaternion),
                    (stamp_ns-before.stamp_ns)/gap)
                return pos,quat,0.
        nearest = (self.poses[0] if index==0 else
                   self.poses[-1] if index>=len(self.poses) else
                   min((self.poses[index-1],self.poses[index]),
                       key=lambda p:abs(p.stamp_ns-stamp_ns)))
        age=abs(nearest.stamp_ns-stamp_ns)
        if age>self.max_age_ns:
            return None
        return nearest.translation,nearest.quaternion,age/1e9


def mapped_mounts(footprint_pose):
    """map->body and map->lidar based on selected replay-only geometry."""
    position,quaternion=footprint_pose[:2]
    body=combine(position,quaternion,(0.,0.,SELECTED_MOUNT_HEIGHT_M))
    lidar=combine(body[0],body[1],(SELECTED_LIDAR_X_M,0.,0.))
    return body,lidar
