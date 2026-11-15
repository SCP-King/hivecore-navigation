"""Requested-turn swept volume; runtime uses the shared URDF footprint."""
import math
import numpy as np


def rotation_clear(points, position, plane, radius=.80, min_observations=30):
    p=np.asarray(points,dtype=float)
    xyz=np.asarray(position,dtype=float);abc=np.asarray(plane,dtype=float)
    if (p.ndim!=2 or p.shape[1]!=3 or xyz.shape!=(3,) or abc.shape!=(3,)
            or not np.isfinite(xyz).all() or not np.isfinite(abc).all()
            or not math.isfinite(radius) or radius<.76):
        raise ValueError('invalid rotation clearance geometry')
    p=p[np.isfinite(p).all(axis=1)]
    if len(p)<min_observations:
        raise ValueError('insufficient rotation obstacle observations')
    floor=p[:,:2]@abc[:2]+abc[2]
    obstacle=((p[:,2]>floor+.06)&(p[:,2]>=xyz[2]-.90)&(p[:,2]<=xyz[2]+.25))
    distance=np.linalg.norm(p[:,:2]-xyz[:2],axis=1)
    return not bool(np.any(obstacle & (distance<=radius)))


def rotation_arc_clear(points, position, plane, start_yaw, target_yaw, min_observations=30, footprint=None):
    """Check the shortest commanded turn with the shared runtime footprint.

    Sample every <= 2 degrees, covering between samples by expanding each
    footprint by its maximum angular displacement. No whole-circle veto.
    """
    from .heading_search import FootprintCollision
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or not all(math.isfinite(v) for v in (*position, start_yaw, target_yaw)):
        raise ValueError('invalid rotation clearance geometry')
    p = p[np.isfinite(p).all(axis=1)]
    if len(p) < min_observations:
        raise ValueError('insufficient rotation obstacle observations')
    angle = math.atan2(math.sin(target_yaw-start_yaw), math.cos(target_yaw-start_yaw))
    count = max(1, math.ceil(abs(angle)/math.radians(2)))
    yaw = np.linspace(start_yaw, start_yaw+angle, count+1)
    bounds = [[position[0]-2., position[1]-2.], [position[0]+2., position[1]+2.]]
    collision = FootprintCollision(p, plane, position[2], bounds, footprint=footprint)
    if footprint is not None:
        # Cover the continuous body rotation between sampled poses.
        collision.radius += np.linalg.norm(footprint,axis=1).max()*2*math.sin(abs(angle)/count/4)
    return collision.free(np.column_stack((np.full(len(yaw), position[0]),
                                          np.full(len(yaw), position[1]), yaw)))
