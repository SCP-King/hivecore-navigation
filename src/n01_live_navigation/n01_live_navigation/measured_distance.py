"""Planar travel from stamped body speed, independent of raw pose integration."""
import math
class MeasuredDistance:
    def __init__(self):self.last=None;self.total=0.;self.gap=False
    def update(self,stamp_ns,vx,vy):
        if not all(math.isfinite(v)for v in (vx,vy)) or math.hypot(vx,vy)>3.:
            raise ValueError('invalid_measured_speed')
        speed=math.hypot(vx,vy)
        if self.last is not None:
            dt=(stamp_ns-self.last[0])/1e9
            if dt<=0:return self.total
            self.gap=dt>.08
            if not self.gap:self.total+=(speed+self.last[1])*.5*dt
        self.last=(stamp_ns,speed)
        return self.total
