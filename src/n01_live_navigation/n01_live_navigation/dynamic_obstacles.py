"""Bounded geometric tracks and short-horizon occupancy, no semantic labels.

Only current measured clusters build motion tracks. Predictions are an extra
collision layer, never fed to the ray caster as sensor observations.
"""
from collections import deque
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import linear_sum_assignment


class ObstacleTracks:
    def __init__(self,horizons=(.25,.5,.75)):
        if not horizons or any(not math.isfinite(h) or not 0<h<=2. for h in horizons):raise ValueError("invalid_prediction_horizons")
        self.horizons=tuple(horizons)
        self.tracks={};self.next_id=0;self.stamp=0.;self.stats={};self.last_forecast=np.empty((0,3))

    def update(self, points, stamp):
        points=np.asarray(points,float).reshape(-1,3)
        if not math.isfinite(stamp) or stamp<self.stamp:return np.empty((0,3))
        if stamp==self.stamp:return self.last_forecast.copy()
        if self.stamp and stamp-self.stamp>1.:self.tracks.clear()
        self.stamp=stamp
        points=points[np.isfinite(points).all(axis=1)]
        if len(points):
            _,ids=np.unique(np.floor(points[:,:2]/.12).astype(np.int64),axis=0,return_index=True)
            points=points[ids]
        # Bounded CPU work; untracked raw returns remain in the actual scene.
        if len(points)>5000:
            self.tracks.clear();self.stats={'reason':'tracking_budget_exceeded','dynamic_tracks':0};self.last_forecast=np.empty((0,3));return self.last_forecast
        parent=np.arange(len(points))
        def root(i):
            while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
            return i
        if len(points):
            for a,b in cKDTree(points[:,:2]).query_pairs(.23):
                a,b=root(a),root(b)
                if a!=b:parent[b]=a
        clusters={}
        for i in range(len(points)):clusters.setdefault(root(i),[]).append(i)
        observations=[]
        for ids in clusters.values():
            p=points[ids]
            if len(p)>=3 and np.ptp(p[:,:2],axis=0).max()<=1.5:
                observations.append((np.mean(p[:,:2],axis=0),p))
        observations=observations[:128]
        self.tracks={k:v for k,v in self.tracks.items()if stamp-v[-1][0]<=.5}
        keys=list(self.tracks);matched={}
        if keys and observations:
            costs=np.array([[np.linalg.norm(self.tracks[k][-1][1]-o[0])for o in observations]for k in keys])
            rows,cols=linear_sum_assignment(costs)
            for r,c in zip(rows,cols):
                elapsed=stamp-self.tracks[keys[r]][-1][0]
                if costs[r,c]<=.2+2.*elapsed:matched[c]=keys[r]
        forecasts=[];moving=0
        for i,(center,p) in enumerate(observations):
            k=matched.get(i)
            if k is None:k=self.next_id;self.next_id+=1;self.tracks[k]=deque(maxlen=8)
            history=self.tracks[k];history.append((stamp,center.copy()))
            if len(history)<4 or stamp-history[0][0]<.3:continue
            t=np.array([h[0]-stamp for h in history]);xy=np.array([h[1]for h in history])
            design=np.column_stack((t,np.ones(len(t))));fit=np.linalg.lstsq(design,xy,rcond=None)[0]
            speed=float(np.linalg.norm(fit[0]));error=float(np.linalg.norm(design@fit-xy,axis=1).max())
            if not .15<=speed<=2. or error>.07:continue
            # Inconsistent recent directions are not evidence of predictable motion.
            previous=history[-2][1];recent=(center-previous)/max(.01,stamp-history[-2][0])
            if np.linalg.norm(recent-fit[0])>.8:continue
            moving+=1
            for horizon in self.horizons:
                predicted=p.copy();predicted[:,:2]+=fit[0]*horizon
                # Small position uncertainty, independent of robot/environment margin.
                for offset in ((0,0),(.05,0),(-.05,0),(0,.05),(0,-.05)):
                    q=predicted.copy();q[:,:2]+=offset;forecasts.append(q)
        self.stats={'tracks':len(self.tracks),'dynamic_tracks':moving,'clusters':len(observations)}
        self.last_forecast=np.concatenate(forecasts) if forecasts else np.empty((0,3))
        if len(self.last_forecast)>20000:
            _,ids=np.unique(np.floor(self.last_forecast/.08).astype(np.int64),axis=0,return_index=True)
            self.last_forecast=self.last_forecast[ids]
            if len(self.last_forecast)>20000:
                self.last_forecast=np.empty((0,3));self.stats['reason']='forecast_budget_exceeded'
        return self.last_forecast
