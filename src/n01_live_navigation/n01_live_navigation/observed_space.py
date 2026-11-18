"""Projected observation quality, separate from 3D obstacle occupancy."""
import numpy as np
from .robot_footprint import polygon_hits


def observed_states(states,footprint,observed,resolution=.1,margin=.06):
    states=np.asarray(states,float).reshape(-1,4);polygon=np.asarray(footprint,float)
    cells=set(map(tuple,np.floor(np.asarray(observed).reshape(-1,3)[:,:2]/resolution).astype(np.int64)))
    rho=float(np.linalg.norm(polygon,axis=1).max())+margin
    result=[]
    for x,y,z,yaw in states:
        lo=np.floor((np.array([x,y])-rho)/resolution).astype(int)
        hi=np.floor((np.array([x,y])+rho)/resolution).astype(int)
        gx,gy=np.meshgrid(np.arange(lo[0],hi[0]+1),np.arange(lo[1],hi[1]+1))
        keys=np.column_stack((gx.ravel(),gy.ravel()))
        delta=(keys+.5)*resolution-[x,y];c,s=np.cos(yaw),np.sin(yaw)
        local=np.column_stack((c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1]))
        required=keys[polygon_hits(local,polygon,margin)]
        result.append(all(tuple(v)in cells for v in required))
    return np.asarray(result,bool)


def path_observed(states,footprint,observed,resolution=.1,margin=.06):
    return bool(observed_states(states,footprint,observed,resolution,margin).all())


def executable_prefix(states,valid,max_speed,max_accel,reaction_s,margin=.06):
    """Keep a contiguous verified prefix and derive speed from stopping distance.

    No extra search and no permission to skip an invalid initial section.
    """
    states=np.asarray(states,float).reshape(-1,4);valid=np.asarray(valid,bool)
    if len(valid)!=len(states) or max_accel<=0 or max_speed<=0 or reaction_s<0:raise ValueError('invalid_prefix_budget')
    failures=np.flatnonzero(~valid);end=int(failures[0])if len(failures)else len(states)
    if end==len(states):return states.tolist(),float(max_speed)
    if end<2:raise ValueError('no_verified_initial_motion_corridor')
    prefix=states[:end];length=float(np.linalg.norm(np.diff(prefix[:,:2],axis=0),axis=1).sum())
    if length<=margin:raise ValueError('verified_corridor_shorter_than_environment_margin')
    speed=max_accel*(np.sqrt(reaction_s*reaction_s+2*(length-margin)/max_accel)-reaction_s)
    return prefix.tolist(),min(float(max_speed),float(speed))
