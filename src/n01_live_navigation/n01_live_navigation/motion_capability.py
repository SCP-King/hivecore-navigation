"""Shared forward-motion domain from runtime chassis geometry and native IK.

Geometric admission only: timed wheel transitions and collision sweeps remain
mandatory downstream. No physical limit is inferred from a turning radius.
"""
from functools import lru_cache
import math
import numpy as np
from .hive2_kinematics import inverse, forward

class ForwardCapability:
    def __init__(self, geometry, steering, speed):
        self.geometry=geometry
        self.steering=np.asarray(steering,float)
        self.speed=float(speed)
        if not math.isfinite(self.speed) or self.speed<=geometry.spin_threshold:
            raise ValueError('forward_capability_requires_translation_speed')
        self.max_curvature=geometry.max_angular_speed/self.speed
        # Sample wheel-heading space, rather than arbitrarily imposing a
        # one-metre radius. Native IK filters every candidate below.
        length=max(geometry.wheel_base_x/2,geometry.wheel_radius)
        angle=math.atan(self.max_curvature*length)
        candidates=[math.tan(a)/length for a in np.linspace(-angle,angle,9)]
        candidates.extend([0.,-.5,.5,-1.,1.])
        self.choices=tuple(sorted(set(round(k,8) for k in candidates if self.accepts(k))))
        if not self.choices:raise ValueError('no_native_forward_motion_available')

    @lru_cache(maxsize=2048)
    def accepts(self, curvature):
        if not math.isfinite(curvature) or abs(curvature)>self.max_curvature+1e-6:return False
        angles,drive=inverse((self.speed,0.,self.speed*curvature),self.steering,geometry=self.geometry)
        # Native drive allocation uses feedback steering contact offsets.
        # Re-evaluate at the target posture for geometric reachability; the
        # current-to-target transition is checked by the timed sequence screen.
        settled,settled_drive=inverse((self.speed,0.,self.speed*curvature),angles,geometry=self.geometry)
        actual,_=forward(settled,[0.]*4,settled_drive,geometry=self.geometry)
        requested=np.array([self.speed,0.,self.speed*curvature])
        if np.max(np.abs(actual-requested))>max(1e-4,.02*np.max(np.abs(requested))):
            return False  # Native IK may clamp wheel angles instead of raising.
        return bool(np.max(np.abs(angles))<=min(math.pi,self.geometry.controller_steering_limit)+1e-6
                    and max(np.max(np.abs(drive)),np.max(np.abs(settled_drive)))<=20.+1e-6)

    def segment_allowed(self, states):
        return all(self.accepts(s[3]) for s in states)
