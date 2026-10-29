"""Research-only spatial background and object extent filters.

Uses a rolling voting map of cluster voxel cells in MAP coordinates.
A cell observed persistently is *possibly static*, not proven static;
false suppression is possible for slowly moving/paused objects.
Does not change the cloud sent to SCAN: only filters motion hypotheses.
"""
from collections import Counter, deque
import math


class StaticBackgroundFilter:
    def __init__(self, history_frames=8, min_observations=5,
                 overlap_threshold=.65, max_extent_m=1.8, max_gap_s=2.0):
        if not (5<=history_frames<=30 and
                3<=min_observations<=history_frames and
                .5<=overlap_threshold<=1. and
                .3<=max_extent_m<=5. and .1<=max_gap_s<=10.):
            raise ValueError("invalid static background filter limits")
        self.history=deque(maxlen=history_frames)
        self.min_observations=min_observations
        self.overlap_threshold=overlap_threshold
        self.max_extent_m=max_extent_m
        self.max_gap_ns=int(max_gap_s*1e9)
        self.last_stamp_ns=None
        self.clock_resets=0
        self.stats={"frames":0,"oversized_rejected":0,"background_rejected":0}

    def update(self, stamp_ns, clusters):
        if stamp_ns<=0:
            raise ValueError("zero or negative timestamp")
        if (self.last_stamp_ns is not None and
            (stamp_ns<=self.last_stamp_ns or
             stamp_ns-self.last_stamp_ns>self.max_gap_ns)):
            self.history.clear()
            self.clock_resets+=1
        self.last_stamp_ns=stamp_ns
        counts=Counter(cell for observed in self.history for cell in observed)
        stable={cell for cell,n in counts.items() if n>=self.min_observations}
        accepted=[]
        oversized=0
        background=0
        present=set()
        for cluster in clusters:
            keys=set(cluster.get("voxel_cells",()))
            # No valid voxel ids => no trustworthy dynamic association.
            if not keys:
                continue
            present.update(keys)
            ext=cluster["extent_xy_m"]
            if len(ext)<2 or not all(math.isfinite(v) and v>=0 for v in ext):
                oversized+=1
                continue
            if max(ext)>self.max_extent_m:
                oversized+=1
                continue
            overlap=len(keys & stable)/len(keys)
            if overlap>=self.overlap_threshold:
                background+=1
                continue
            # Internal implementation detail is not sent to JSON/CBF.
            accepted.append({k:v for k,v in cluster.items()
                             if k!="voxel_cells"})
        # Track all clustered cells, including rejected large structures.
        self.history.append(present)
        self.stats["frames"]+=1
        self.stats["oversized_rejected"]+=oversized
        self.stats["background_rejected"]+=background
        return accepted,{
            "raw_clusters":len(clusters),
            "filtered_clusters":len(accepted),
            "oversized_clusters_rejected":oversized,
            "stable_background_clusters_rejected":background,
            "stable_background_cell_count":len(stable),
            "background_clock_resets":self.clock_resets,
            "background_warmed":len(self.history)>=self.min_observations,
        }
