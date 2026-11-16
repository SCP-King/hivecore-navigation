"""Native SCAN event ordering and geometry-preserving sampling (no ROS I/O)."""
import hashlib
import json
import math


class SplineEvents:
    """One producer epoch per accepted goal; no receive-time trajectory clock.

    Bspline carries the producer goal ID. Repeated events never refresh their
    source start, and revisions are ordered within one accepted goal.
    """
    def __init__(self):
        self.latest = None

    @staticmethod
    def fingerprint(msg):
        payload = [int(msg.goal_id), int(msg.order), int(msg.traj_id),
                   int(msg.start_time.sec), int(msg.start_time.nanosec),
                   list(msg.knots), [(p.x, p.y, p.z) for p in msg.pos_pts],
                   list(msg.yaw_pts), float(msg.yaw_dt)]
        raw = json.dumps(payload, allow_nan=False, separators=(',', ':')).encode()
        return hashlib.sha256(raw).hexdigest()

    def classify(self, msg, goal_ns):
        start = int(msg.start_time.sec)*10**9+int(msg.start_time.nanosec)
        if goal_ns <= 0:
            raise ValueError('missing_accepted_goal')
        if int(msg.goal_id) != goal_ns:
            raise ValueError('trajectory_goal_mismatch')
        if start < goal_ns:
            raise ValueError('trajectory_from_previous_goal')
        identity = (int(msg.traj_id), start, self.fingerprint(msg))
        if self.latest is not None:
            old_id, old_start, old_hash = self.latest
            if identity[0] < old_id or start < old_start:
                raise ValueError('out_of_order_trajectory')
            if identity[0] == old_id:
                if identity != self.latest:
                    raise ValueError('conflicting_trajectory_revision')
                return 'duplicate'
        return 'new'

    def commit(self, msg):
        self.latest = (int(msg.traj_id),
                       int(msg.start_time.sec)*10**9+int(msg.start_time.nanosec),
                       self.fingerprint(msg))


def sample_native_prefix(curve, x, y, preview_m, step=.02):
    """Sample, never fit/snap/re-search, the original position spline.

    Path is a transitional sampled view; source knots/time/ID are unchanged.
    End the sampled prefix exactly at preview/end, without repeated endpoints.
    """
    if not math.isfinite(preview_m) or not .3 <= preview_m <= 2.:
        raise ValueError('invalid_execution_preview')
    anchor = curve.nearest_arc_length(x, y)
    length = min(preview_m, curve.length-anchor)
    if length < 1e-5:
        raise ValueError('scan_prefix_exhausted; replan_required')
    count = max(2, int(math.ceil(length/step))+1)
    _, states = curve.sample_window(x, y, count=count, step=max(.001, length/(count-1)))
    return states
