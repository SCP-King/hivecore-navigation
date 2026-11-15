"""Measured angular travel, using message time; raw odometry yaw is not an input."""
import math


class AngularTravel:
    def __init__(self, max_gap=.08):
        self.last = None
        self.angle = 0.
        self.segment = 0
        self.max_gap = max_gap
        self.max_observed_gap = 0.

    def update(self, stamp_ns, rate):
        if stamp_ns <= 0 or not math.isfinite(rate) or abs(rate) > 5.:
            raise ValueError('invalid measured angular velocity')
        if self.last is not None:
            stamp, previous = self.last
            dt = (stamp_ns-stamp)/1e9
            if dt <= 0:
                raise ValueError('angular velocity timestamp not advancing')
            self.max_observed_gap=max(self.max_observed_gap,dt)
            if dt > self.max_gap:
                self.segment += 1
                self.angle = 0.
            else:
                self.angle += .5*(rate+previous)*dt
        self.last = (stamp_ns, rate)
        return self.angle
