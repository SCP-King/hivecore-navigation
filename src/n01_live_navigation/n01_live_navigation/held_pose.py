"""Reject newly stamped copies of a held SLAM board when odometry moved."""
import math
from .core import pose_samples_equivalent
from .delayed_pose import angle


class HeldPoseGuard:
    def __init__(self):self.reference=None
    def accept(self,stamp_ns,position,orientation,odom):
        if self.reference is not None:
            stamp,p,q,reference_odom=self.reference
            if pose_samples_equivalent((stamp,p,q),position,orientation,
                                       position_tolerance_m=1e-7,orientation_tolerance_rad=1e-7):
                if reference_odom is not None:
                    if odom is None:return False
                    if (math.dist(reference_odom[0],odom[0])>.005 or
                            angle(reference_odom[1],odom[1])>.01):return False
                    return True  # Keep the motion baseline; do not erase creep.
        self.reference=(stamp_ns,position,orientation,odom)
        return True
