"""Forward-speed CBF projection for measured point obstacles and moving tracks.

Solves a scalar convex QP min (v-v_nom)^2 subject to 0<=v<=v_nom and
2*r.(v*heading-v_obstacle)+alpha*(r.r-radius^2)>=0. This is a velocity
safety filter, not the separate offline MPC-D-CBF solver or collision certification.
"""
import math


def project_speed(position, heading, desired, obstacles, radius=.55, alpha=1.0):
    if not all(math.isfinite(x) for x in (*position, heading, desired, radius, alpha)) or desired<0 or radius<=0 or alpha<=0:
        return 0., 'INVALID_INPUT'
    low, high = 0., desired
    direction = (math.cos(heading),math.sin(heading))
    try:
        if any(len(o)!=5 or not all(math.isfinite(v) for v in o) or o[4]<0 for o in obstacles):
            return 0., 'INVALID_OBSTACLE'
    except (TypeError, ValueError):
        return 0., 'INVALID_OBSTACLE'
    for ox,oy,vx,vy,extra_radius in obstacles:
        if not all(math.isfinite(x) for x in (ox,oy,vx,vy,extra_radius)) or extra_radius<0:
            return 0., 'INVALID_OBSTACLE'
        rx,ry = position[0]-ox,position[1]-oy
        h = rx*rx+ry*ry-(radius+extra_radius)**2
        if h<=0:
            return 0., 'CBF_OVERLAP_STOP'
        a = 2*(rx*direction[0]+ry*direction[1])
        b = 2*(rx*vx+ry*vy)-alpha*h
        if abs(a)<1e-10:
            if b>0: return 0., 'CBF_INFEASIBLE_STOP'
        elif a>0:
            low = max(low,b/a)
        else:
            high = min(high,b/a)
        if high<low:
            return 0., 'CBF_INFEASIBLE_STOP'
    speed = max(0.,min(desired,high))
    return speed, ('CBF_LIMIT' if speed<desired-1e-9 else 'CBF_CLEAR')
def point_obstacles(points, tracks):
    """Measured cloud support, with valid tracked velocities on matching points.

    Unknown-velocity candidates remain stationary instantaneous cloud constraints;
    do not invent a 0.75m solid disk around a cluster centroid. A valid moving
    track requires observed bounds so its velocity can be assigned to support.
    """
    import numpy as np
    xyz=np.asarray(points,dtype=float)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not np.isfinite(xyz).all():
        raise ValueError('invalid_obstacle_points')
    velocities=np.zeros((len(xyz),2))
    for t in tracks:
        if t.get('velocity_valid') is not True:
            continue
        low=np.asarray(t.get('minimum_map',[]),dtype=float)
        high=np.asarray(t.get('maximum_map',[]),dtype=float)
        velocity=np.asarray(t.get('velocity_map_mps',[]),dtype=float)
        if low.shape!=(3,) or high.shape!=(3,) or velocity.shape!=(3,) or not all(np.isfinite(a).all() for a in (low,high,velocity)) or (high<low).any():
            raise ValueError('invalid_dynamic_track_support')
        selected=((xyz>=low)&(xyz<=high)).all(axis=1)
        velocities[selected]=velocity[:2]
    return [(float(p[0]),float(p[1]),float(v[0]),float(v[1]),0.) for p,v in zip(xyz,velocities)]
