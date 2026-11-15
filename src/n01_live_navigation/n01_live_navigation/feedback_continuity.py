"""Bounded feedback continuity; never refresh source timestamps."""
class SteeringRateGuard:
    def __init__(self, limit=1.57, hard_limit=3.14, hold=.15, recovery=.2):
        self.limit=limit; self.hard_limit=hard_limit
        self.hold=hold; self.recovery=recovery
        self.over_since=None; self.good_since=None; self.blocked=False; self.degraded=False

    def update(self, rates, now):
        import math
        if not rates or not all(math.isfinite(v) for v in rates):
            self.blocked=True;self.degraded=True;return False
        peak=max(abs(v) for v in rates)
        self.degraded=peak>self.limit or self.blocked
        if peak>self.hard_limit:
            self.blocked=True; self.good_since=None
        elif peak>self.limit:
            self.good_since=None
            if self.over_since is None:self.over_since=now
            if now-self.over_since>=self.hold:self.blocked=True
        else:
            self.over_since=None
            if peak<=self.limit*.9:
                if self.good_since is None:self.good_since=now
                if now-self.good_since>=self.recovery:self.blocked=False
            else:self.good_since=None
        self.degraded=peak>self.limit or self.blocked
        return not self.blocked


def bounded_pose_hold(rx_age, stamp_age, rx_limit, stamp_limit, anchor_limit,
                      status_fresh, odom_fresh):
    if not -.2<=stamp_age<=stamp_limit or rx_age<0:
        return False
    if rx_age<=rx_limit:
        return True
    return (status_fresh and odom_fresh and rx_age<=anchor_limit
            and stamp_age<=anchor_limit)
