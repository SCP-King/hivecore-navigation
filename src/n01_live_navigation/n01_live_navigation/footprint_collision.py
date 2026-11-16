"""Shared scene collision query; never searches or changes a trajectory."""
import math
import numpy as np
from scipy.spatial import cKDTree

ENVIRONMENT_MARGIN = .06


class FootprintCollision:
    def __init__(self, points, plane, z, bounds, voxel_size=.05, footprint=None, height_margin=0.):
        p = np.asarray(points, float).reshape((-1, 3))
        plane = np.asarray(plane, float)
        self.bounds = np.asarray(bounds, float)
        if (plane.shape != (3,) or self.bounds.shape != (2, 2)
                or not np.isfinite(plane).all() or not np.isfinite(self.bounds).all()
                or not math.isfinite(z) or not math.isfinite(height_margin) or not 0<=height_margin<=.05 or not 0 <= voxel_size <= .1
                or np.any(self.bounds[1] <= self.bounds[0])):
            raise ValueError('invalid_collision_geometry')
        p = p[np.isfinite(p).all(axis=1)]
        floor = p[:, :2] @ plane[:2] + plane[2]
        p = p[(p[:, 2] > floor+.03) & (p[:, 2] >= z-.90-height_margin) & (p[:, 2] <= z+.25+height_margin)]
        self.points=p[:,:2]
        if footprint is None:raise ValueError("urdf_footprint_required")
        self.polygon=np.asarray(footprint,float)
        if self.polygon is not None:
            from .robot_footprint import polygon_hits
            polygon_hits([[0.,0.]],self.polygon)
        self.tree = cKDTree(p[:, :2]) if len(p) else None
        # Occupancy points are voxel centres. Cover their half diagonal plus
        # the maximum displacement between 2 cm collision samples.
        self.radius = ENVIRONMENT_MARGIN

    def free_batch(self, trajectories):
        s = np.asarray(trajectories, float)
        if s.ndim != 3 or s.shape[2] < 3 or not np.isfinite(s).all():
            raise ValueError('invalid_collision_trajectories')
        if self.polygon is not None:
            from .robot_footprint import polygon_hits
            rho=float(np.linalg.norm(self.polygon,axis=1).max())+self.radius
            flat=s.reshape(-1,s.shape[-1]);c=np.cos(flat[:,2]);sn=np.sin(flat[:,2])
            px=self.polygon[:,0];py=self.polygon[:,1]
            wx=flat[:,0,None]+c[:,None]*px-sn[:,None]*py
            wy=flat[:,1,None]+sn[:,None]*px+c[:,None]*py
            hit=((wx.min(axis=1)-self.radius<self.bounds[0,0]) |
                 (wx.max(axis=1)+self.radius>self.bounds[1,0]) |
                 (wy.min(axis=1)-self.radius<self.bounds[0,1]) |
                 (wy.max(axis=1)+self.radius>self.bounds[1,1]))
            if self.tree is not None:
                groups=self.tree.query_ball_point(flat[:,:2],rho)
                counts=np.array([len(g)for g in groups])
                if counts.sum():
                    owners=np.repeat(np.arange(len(flat)),counts)
                    ids=np.concatenate([g for g in groups if len(g)]).astype(int)
                    delta=self.points[ids]-flat[owners,:2]
                    local=np.column_stack((c[owners]*delta[:,0]+sn[owners]*delta[:,1],
                                           -sn[owners]*delta[:,0]+c[owners]*delta[:,1]))
                    for begin in range(0,len(local),1024):
                        mask=polygon_hits(local[begin:begin+1024],self.polygon,self.radius)
                        hit[owners[begin:begin+1024][mask]]=True
            return ~hit.reshape(s.shape[0],s.shape[1]).any(axis=1)

    def free(self, states):
        s = np.asarray(states, float)
        if s.ndim != 2 or s.shape[1] < 3 or not np.isfinite(s).all():
            return False
        return bool(self.free_batch(s[None, ...])[0])

