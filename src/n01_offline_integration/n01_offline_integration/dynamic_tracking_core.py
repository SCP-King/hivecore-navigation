"""Bounded, non-actuating XY obstacle hypotheses and constant-velocity prediction.

Research/functional prototype only. Point-cloud clusters have no semantic class
or verified identity. Static scene artifacts can masquerade as motion.
No ROS dependencies, no MPC/CBF, no ability to publish robot commands.
"""
from collections import deque
from dataclasses import dataclass
import math


def finite2(point):
    return len(point) >= 2 and all(math.isfinite(v) for v in point[:2])


def clusters_from_points(points, body_xy, cell_size=0.35, radius=8.,
                         height_min=-0.70, height_max=1.0,
                         min_cells=2, max_cells=90, min_points=6,
                         max_points=20000):
    """Spatial 8-connected occupied XY voxel clusters, map-aligned.

    Points are already in map frame. Returns bounded cluster centroids with
    box extents. No detections are emitted for too-small or too-large groups.
    This is *not* a proof that a cluster is a dynamic obstacle.
    """
    if not finite2(body_xy):
        return []
    if not (0.1 <= cell_size <= 1. and 1 <= radius <= 40 and
            1 <= min_cells < max_cells <= 10000 and
            1 <= min_points <= 200 and 100 <= max_points <= 100000):
        raise ValueError("unsafe voxel extraction settings")
    cells={}
    x0,y0,z0=body_xy[:3]
    inspected=0
    for point in points:
        if inspected >= max_points:
            break
        inspected+=1
        if len(point)<3 or not all(math.isfinite(v) for v in point[:3]):
            continue
        x,y,z=point[:3]
        if (abs(x-x0)>radius or abs(y-y0)>radius or
                not (height_min <= z-z0 <= height_max)):
            continue
        key=(math.floor(x/cell_size),math.floor(y/cell_size))
        row=cells.get(key)
        if row is None:
            cells[key]=[x,y,z,1,x,x,y,y]
        else:
            row[0]+=x
            row[1]+=y
            row[2]+=z
            row[3]+=1
            row[4]=min(row[4],x);row[5]=max(row[5],x)
            row[6]=min(row[6],y);row[7]=max(row[7],y)
    seen=set()
    groups=[]
    for first in cells:
        if first in seen:
            continue
        pending=deque([first]);seen.add(first)
        region=[]
        while pending:
            key=pending.popleft()
            region.append(key)
            ix,iy=key
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    adjacent=(ix+dx,iy+dy)
                    if adjacent in cells and adjacent not in seen:
                        seen.add(adjacent);pending.append(adjacent)
        if not min_cells<=len(region)<=max_cells:
            continue
        n=sum(cells[k][3] for k in region)
        if n<min_points:
            continue
        cx=sum(cells[k][0] for k in region)/n
        cy=sum(cells[k][1] for k in region)/n
        cz=sum(cells[k][2] for k in region)/n
        bounds=(min(cells[k][4] for k in region),
                max(cells[k][5] for k in region),
                min(cells[k][6] for k in region),
                max(cells[k][7] for k in region))
        groups.append({"xyz":[cx,cy,cz], "points":n, "cells":len(region),
                       "extent_xy_m":[bounds[1]-bounds[0],bounds[3]-bounds[2]]})
    return sorted(groups,key=lambda x:(-x["points"],x["xyz"][0],x["xyz"][1]))


@dataclass
class Track:
    track_id: int
    xyz: tuple
    vx: float
    vy: float
    stamp_ns: int
    hits: int
    missed: int
    cells: int
    extent_xy_m: tuple


class Tracker:
    """Greedy gated XY association; no Kalman, no semantic motion guarantee."""
    def __init__(self, max_track_count=60, match_dist_m=1.2,
                 max_gap_s=2., min_hits=3, motion_threshold_mps=.18,
                 max_plausible_mps=3.0, prediction_steps_s=(.5,1.,1.5)):
        if not (1<=max_track_count<=200 and .2<=match_dist_m<=4
                and .2<=max_gap_s<=10 and 2<=min_hits<=20
                and 0<=motion_threshold_mps<=2 and
                motion_threshold_mps < max_plausible_mps <=10):
            raise ValueError("invalid tracker bounds")
        self.max_track_count=max_track_count
        self.match_dist_m=match_dist_m
        self.max_gap_ns=int(max_gap_s*1e9)
        self.min_hits=min_hits
        self.motion_threshold_mps=motion_threshold_mps
        self.max_plausible_mps=max_plausible_mps
        self.steps=tuple(prediction_steps_s)
        if not self.steps or any(not .01<=t<=5 for t in self.steps):
            raise ValueError("prediction horizon out of bounds")
        self.tracks=[]
        self.next_id=1
        self.last_stamp_ns=None
        self.clock_resets=0

    def update(self, stamp_ns, clusters):
        if stamp_ns<=0:
            raise ValueError("invalid scan timestamp")
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            # Duplicate or reversing times cannot be used for velocity.
            self.tracks.clear()
            self.clock_resets+=1
            self.last_stamp_ns=stamp_ns
            return self.snapshot(stamp_ns,reason="nonmonotonic_timestamp_reset")
        self.last_stamp_ns=stamp_ns
        self.tracks=[t for t in self.tracks if
                     0 < stamp_ns-t.stamp_ns <= self.max_gap_ns]
        proposals=[]
        for i,t in enumerate(self.tracks):
            for j,c in enumerate(clusters):
                p=c["xyz"]
                if not finite2(p): continue
                d=math.hypot(t.xyz[0]-p[0],t.xyz[1]-p[1])
                dt=(stamp_ns-t.stamp_ns)*1e-9
                if d<=min(self.match_dist_m,self.max_plausible_mps*dt+.35):
                    proposals.append((d,i,j))
        proposals.sort()
        matched_tracks=set()
        matched_clusters=set()
        for _,i,j in proposals:
            if i in matched_tracks or j in matched_clusters:
                continue
            t=self.tracks[i];p=clusters[j]["xyz"]
            dt=(stamp_ns-t.stamp_ns)*1e-9
            if dt<=0:continue
            dx=(p[0]-t.xyz[0])/dt
            dy=(p[1]-t.xyz[1])/dt
            # Low-pass reduces centroid voxel jitter, not an uncertainty model.
            t.vx=.35*t.vx+.65*dx
            t.vy=.35*t.vy+.65*dy
            t.xyz=tuple(p)
            t.stamp_ns=stamp_ns
            t.hits+=1
            t.missed=0
            t.cells=clusters[j]["cells"]
            t.extent_xy_m=tuple(clusters[j]["extent_xy_m"])
            matched_tracks.add(i);matched_clusters.add(j)
        for i,t in enumerate(self.tracks):
            if i not in matched_tracks:
                t.missed+=1
        for j,c in enumerate(clusters):
            if j in matched_clusters or len(self.tracks)>=self.max_track_count:
                continue
            if not finite2(c.get("xyz",())):
                continue
            self.tracks.append(Track(
                self.next_id,tuple(c["xyz"]),0.,0.,stamp_ns,1,0,
                int(c["cells"]),tuple(c["extent_xy_m"])))
            self.next_id+=1
        # Exclude missed tracks from published "live" hypotheses.
        return self.snapshot(stamp_ns)

    def snapshot(self,stamp_ns,reason="updated"):
        out=[]
        moving=0
        for t in self.tracks:
            if t.stamp_ns!=stamp_ns:
                continue
            speed=math.hypot(t.vx,t.vy)
            candidate=t.hits>=self.min_hits and (
                self.motion_threshold_mps<=speed<=self.max_plausible_mps)
            if candidate:moving+=1
            out.append({
                "id":t.track_id,"position_map_m":[round(v,4) for v in t.xyz],
                "velocity_xy_mps":[round(t.vx,4),round(t.vy,4)],
                "estimated_speed_mps":round(speed,4),
                "extent_xy_m":[round(v,3) for v in t.extent_xy_m],
                "hits":t.hits,
                "classification":"MOVING_HYPOTHESIS_UNVERIFIED" if candidate
                                else "UNCONFIRMED_CLUSTER",
                "predicted_positions":[
                    {"dt_s":dt,"xyz_map_m":[
                        round(t.xyz[0]+t.vx*dt,4),
                        round(t.xyz[1]+t.vy*dt,4),
                        round(t.xyz[2],4)]}
                    for dt in self.steps] if candidate else []
            })
        return {"schema":"n01.offline.tracks.v1","stamp_ns":stamp_ns,
                "frame_id":"map", "reason":reason,
                "tracks":out,"moving_hypotheses":moving,
                "clock_resets":self.clock_resets,
                "algorithm":"VOXEL_CLUSTER_GREEDY_ASSOCIATION_CONSTANT_VELOCITY",
                "kalman_prediction_implemented":False,
                "collision_safety_verified":False,
                "motion_commands_enabled":False}
