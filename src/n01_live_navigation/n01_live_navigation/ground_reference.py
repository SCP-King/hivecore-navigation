"""Gravity-aligned ground projection; no assumed body/leg height.

The plane and body pose must be in the same gravity-aligned map frame.
This computes geometry only: callers must establish TF ownership before
broadcasting, and must reject stale/ambiguous plane estimates themselves.
"""
import math
import numpy as np
from .core import compose_transform, invert_transform


def project_footprint(body_position, body_orientation, plane):
    """Return map<-ground and body<-ground for z=a*x+b*y+c.

    Use vertical projection at the body's XY, keep heading, and remove body
    roll/pitch. A sloping floor changes Z, not gravity alignment. No TF, URDF,
    map points or body pose are modified here.
    """
    p = np.asarray(body_position, dtype=float)
    q = np.asarray(body_orientation, dtype=float)
    abc = np.asarray(plane, dtype=float)
    if (p.shape != (3,) or q.shape != (4,) or abc.shape != (3,)
            or not all(np.isfinite(v).all() for v in (p, q, abc))
            or np.linalg.norm(q) < 1e-8):
        raise ValueError('invalid ground projection inputs')
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    # Project the body's forward axis into the horizontal plane.
    forward_x, forward_y = 1-2*(y*y+z*z), 2*(x*y+w*z)
    if math.hypot(forward_x, forward_y) < 1e-6:
        raise ValueError('body heading undefined for vertical forward axis')
    yaw = math.atan2(forward_y, forward_x)
    ground_p = [float(p[0]), float(p[1]), float(abc @ [p[0], p[1], 1.])]
    ground_q = [0., 0., math.sin(yaw/2), math.cos(yaw/2)]
    body_from_map = invert_transform(p.tolist(), q.tolist())
    relative = compose_transform(*body_from_map, ground_p, ground_q)
    return (ground_p, ground_q), relative
