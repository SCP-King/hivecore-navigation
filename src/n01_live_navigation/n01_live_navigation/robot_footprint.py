"""URDF collision hull covering all wheel spins and ankle steering angles.

Each rotational sweep uses a circumscribed regular polygon. Its convex hull
contains the continuous circle, rather than joining samples with unsafe chords.
Leg positions remain measured; fresh joints are required by the publisher.
"""
import math
import numpy as np
from scipy.spatial import ConvexHull

ENVIRONMENT_MARGIN = .06


def rotation_sweep(points, center, axis, steps=36):
    p=np.asarray(points,float)-center;axis=np.asarray(axis,float)
    axis=axis/np.linalg.norm(axis)
    axial=(p@axis)[:,None]*axis
    radial=p-axial
    tangent=np.cross(axis,radial)
    angles=np.arange(steps)*2*math.pi/steps
    outer=1/math.cos(math.pi/steps)
    return (center+axial[:,None,:]+outer*(radial[:,None,:]*np.cos(angles)[None,:,None]
            +tangent[:,None,:]*np.sin(angles)[None,:,None])).reshape(-1,3)


def sweep_joints(model, fresh):
    joints={}
    for _,name,kind,_,_ in model.parents.values():
        if kind=='fixed':continue
        if kind=='continuous' or name.endswith('_ankle_yaw'):
            # These variables are analytically swept over their full range;
            # zero is only the sweep's reference angle, not measured feedback.
            joints[name]=0.
        elif name not in fresh or not math.isfinite(fresh[name]):
            raise ValueError('urdf_leg_joint_stale_or_missing:'+name)
        else:joints[name]=fresh[name]
    return joints


def steering_hull(model,joints):
    model.polygons(joints)
    shapes=[]
    for link,points,_ in model.geometry:
        p=points.copy();child=link
        while child!='base_link':
            parent,name,kind,_,axis=model.parents[child]
            if kind=='continuous' or name.endswith('_ankle_yaw'):
                frame=model.transforms[child]
                p=rotation_sweep(p,frame[:3,3],frame[:3,:3]@axis)
            child=parent
        shapes.append(p[:,:2])
    points=np.concatenate(shapes)
    return points[ConvexHull(points).vertices]


def polygon_hits(points, polygon, margin=ENVIRONMENT_MARGIN):
    """Point inside convex CCW polygon or within rounded edge margin."""
    q=np.asarray(points,float).reshape(-1,2);p=np.asarray(polygon,float)
    if p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or not np.isfinite(p).all():
        raise ValueError('invalid_urdf_footprint')
    b=np.roll(p,-1,axis=0);edge=b-p;length=np.sum(edge*edge,axis=1)
    if np.any(length<1e-14):raise ValueError('degenerate_urdf_footprint')
    normal=np.column_stack((edge[:,1],-edge[:,0]))/np.sqrt(length)[:,None]
    signed=q@normal.T-np.sum(p*normal,axis=1)
    outside=signed.max(axis=1)
    hit=outside<=1e-10
    near=(outside>1e-10)&(outside<=margin+1e-10)
    if near.any():
        v=q[near,None,:]-p[None,:,:]
        t=np.clip(np.sum(v*edge[None,:,:],axis=2)/length,0,1)
        distance=np.sum((v-t[:,:,None]*edge[None,:,:])**2,axis=2).min(axis=1)
        hit[near]=distance<=margin*margin+1e-12
    return hit



def describe(polygon):
    return dict(model='urdf_steering_hull',vertices_m=np.asarray(polygon).tolist(),
                environment_margin_m=ENVIRONMENT_MARGIN,
                steering_coverage='all_ankle_angles_and_wheel_spins',
                basis='runtime_urdf_and_measured_leg_joints')
