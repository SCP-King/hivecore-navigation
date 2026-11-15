"""Require sustained, advancing localization before restarting a stopped task."""
class RecoveryGate:
    def __init__(self, stable_s=1.2, min_updates=4, timeout_s=30., gap_tolerance_s=.3):
        self.stable_s=stable_s;self.min_updates=min_updates;self.timeout_s=timeout_s
        self.started=None;self.good_since=None;self.last_revision=None;self.updates=0
        self.gap_tolerance_s=gap_tolerance_s;self.bad_since=None

    def update(self, now, fresh, revision):
        if self.started is None:self.started=now
        if not fresh:
            if self.bad_since is None:self.bad_since=now
            if now-self.bad_since>=self.gap_tolerance_s:
                self.good_since=None;self.updates=0;self.last_revision=revision
        else:
            if self.bad_since is not None:
                gap=now-self.bad_since
                if gap>=self.gap_tolerance_s:
                    self.good_since=None;self.updates=0
                elif self.good_since is not None:
                    self.good_since+=gap  # Unavailable time never counts as stable.
                self.bad_since=None
            if self.good_since is None:
                self.good_since=now;self.last_revision=revision;self.updates=0
            elif revision!=self.last_revision:
                self.last_revision=revision;self.updates+=1
            if now-self.good_since>=self.stable_s and self.updates>=self.min_updates:
                return 'ready'
        return 'timeout' if now-self.started>=self.timeout_s else 'waiting'
