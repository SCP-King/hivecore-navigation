"""G2.3: exact replay-timestamp alignment and persistent *diagnostic* path geometry.

No ROS dependency and no control authority. SCAN Path header stamps are produced
from node wall clock in existing bridge; they MUST NOT be matched against
historical scan stamps. Instead pair /body_pose and dynamic_tracks on identical
recorded scan nanoseconds, then locate body on the last Path geometry.

Retained reference geometry is an UNVERIFIED WHAT-IF assumption, never a
current authorized trajectory. Track loss, missing path and spatial mismatch
remain explicit inconclusive states, never a safety PASS.
"""
from collections import OrderedDict
import math

from .path_risk_core import evaluate_path_preview


def finite_xy(value):
    return (isinstance(value, (tuple,list)) and len(value)>=2
            and all(isinstance(v,(int,float)) and
                    not isinstance(v,bool) and math.isfinite(v)
                    for v in value[:2]))


def project_path(path, body_xy):
    """Return (distance to path, along-path meters, path starting at projection).

    Preserve shape and order; never extrapolate from the last waypoint.
    """
    if not isinstance(path,list) or len(path)<2 or not finite_xy(body_xy):
        return None
    best=None
    total=0.
    for k,(a,b) in enumerate(zip(path,path[1:])):
        if not finite_xy(a) or not finite_xy(b):
            return None
        dx,dy=b[0]-a[0],b[1]-a[1]
        length=math.hypot(dx,dy)
        if length<=1e-8:
            continue
        u=max(0.,min(1.,((body_xy[0]-a[0])*dx+
                         (body_xy[1]-a[1])*dy)/(length*length)))
        xy=(a[0]+u*dx,a[1]+u*dy)
        error=math.hypot(xy[0]-body_xy[0],xy[1]-body_xy[1])
        if best is None or error<best[0]:
            tail=[xy]+[tuple(p[:2]) for p in path[k+1:]]
            best=(error,total+u*length,tail)
        total+=length
    return best


class ReplayPathSynchronizer:
    """Bounded exact-stamp pairing, one diagnostic per replay cloud timestamp."""

    def __init__(self, max_path_distance_m=1.5, max_pairs=16,
                 assumed_speed_mps=.45, assumed_clearance_m=.60):
        if (not .1<=max_path_distance_m<=5. or
                not 2<=max_pairs<=128):
            raise ValueError("invalid synchronizer bounds")
        self.max_path_distance_m=float(max_path_distance_m)
        self.max_pairs=max_pairs
        self.assumed_speed_mps=assumed_speed_mps
        self.assumed_clearance_m=assumed_clearance_m
        self.path=None
        self.path_header_ns=None
        self.path_seq=0
        self.poses=OrderedDict()
        self.reports=OrderedDict()
        self.last_completed_ns=None
        self.frames_aligned=0
        self.proximity_pairs=0
        self.proximity_conflicts=0
        self.no_path_frames=0
        self.path_outside_frames=0
        self.replay_reversals=0
        self.replay_gaps=0

    def set_path(self, points, header_stamp_ns=0):
        self.path_seq+=1
        self.path_header_ns=header_stamp_ns
        if (not isinstance(points,(list,tuple)) or not 2<=len(points)<=3000 or
                any(not finite_xy(p) for p in points)):
            self.path=None
            return False
        self.path=[(float(p[0]),float(p[1])) for p in points]
        return True

    def _push(self, cache, stamp_ns, value):
        if not isinstance(stamp_ns,int) or isinstance(stamp_ns,bool) or stamp_ns<=0:
            return False
        cache[stamp_ns]=value
        cache.move_to_end(stamp_ns)
        while len(cache)>self.max_pairs:
            cache.popitem(last=False)
        return True

    def put_body(self, stamp_ns, xy):
        if not finite_xy(xy):
            return None
        if not self._push(self.poses,stamp_ns,(float(xy[0]),float(xy[1]))):
            return None
        return self._pair(stamp_ns)

    def put_tracks(self, message):
        if (not isinstance(message,dict) or
            message.get("schema")!="n01.offline.tracks.v1" or
            message.get("frame_id")!="map" or
            message.get("status")!="PROCESSED"):
            return None
        stamp_ns=message.get("stamp_ns")
        tracks=message.get("tracks")
        if not isinstance(tracks,list) or len(tracks)>200:
            return None
        if not self._push(self.reports,stamp_ns,tracks):
            return None
        return self._pair(stamp_ns)

    def _pair(self, stamp_ns):
        if stamp_ns not in self.poses or stamp_ns not in self.reports:
            return None
        body=self.poses.pop(stamp_ns)
        tracks=self.reports.pop(stamp_ns)
        if self.last_completed_ns is not None and stamp_ns<=self.last_completed_ns:
            self.replay_reversals+=1
            self.poses.clear()
            self.reports.clear()
            self.last_completed_ns=stamp_ns
            # Different replay epoch: a previous Path cannot be assumed valid.
            self.path=None
            return self._inconclusive("REPLAY_TIMESTAMP_REVERSED",stamp_ns)
        if self.last_completed_ns is not None and stamp_ns-self.last_completed_ns>2_000_000_000:
            self.replay_gaps+=1
        self.last_completed_ns=stamp_ns
        self.frames_aligned+=1
        if self.path is None:
            self.no_path_frames+=1
            return self._inconclusive("NO_RETAINED_REFERENCE_GEOMETRY",stamp_ns)
        projection=project_path(self.path,body)
        if projection is None:
            return self._inconclusive("INVALID_REFERENCE_GEOMETRY",stamp_ns)
        deviation,progress,local_path=projection
        if deviation>self.max_path_distance_m:
            self.path_outside_frames+=1
            result=self._inconclusive("BODY_OUTSIDE_RETAINED_PATH_CORRIDOR",stamp_ns)
        elif len(local_path)<2 or all(
                math.hypot(q[0]-local_path[0][0],q[1]-local_path[0][1])<=1e-8
                for q in local_path[1:]):
            result=self._inconclusive("AT_REFERENCE_PATH_END",stamp_ns)
        else:
            result=evaluate_path_preview(
                local_path,tracks,assumed_speed_mps=self.assumed_speed_mps,
                assumed_clearance_m=self.assumed_clearance_m)
            if result["evaluated_pairs"]>0:
                self.proximity_pairs+=1
            if result["status"]=="POTENTIAL_PROXIMITY_CONFLICT":
                self.proximity_conflicts+=1
        result.update({
            "recorded_scan_stamp_ns":stamp_ns,
            "time_alignment":"EXACT_RECORDED_BODY_POSE_AND_TRACK_STAMP",
            "path_header_stamp_ns":self.path_header_ns,
            "path_header_is_historical_stamp":False,
            "path_reference_seq":self.path_seq,
            "reference_provenance":"RETAINED_SCAN_PATH_GEOMETRY_UNVERIFIED",
            "current_body_map_xy_m":[round(v,4) for v in body],
            "body_to_reference_distance_m":round(deviation,4),
            "projected_path_progress_m":round(progress,4),
            "reference_reused_without_replanning":True,
        })
        return result

    def _inconclusive(self,reason,stamp_ns):
        result=evaluate_path_preview([],[])
        result["status"]=reason
        result.update({
            "recorded_scan_stamp_ns":stamp_ns,
            "time_alignment":"EXACT_RECORDED_BODY_POSE_AND_TRACK_STAMP",
            "path_header_is_historical_stamp":False,
            "path_reference_seq":self.path_seq,
            "reference_provenance":"UNVERIFIED_OR_UNAVAILABLE",
            "reference_reused_without_replanning":False,
        })
        return result

    def evidence(self):
        return {
            "aligned_replay_frames":self.frames_aligned,
            "aligned_frames_with_proximity_pairs":self.proximity_pairs,
            "potential_conflict_frames":self.proximity_conflicts,
            "aligned_frames_before_first_path":self.no_path_frames,
            "body_outside_reference_frames":self.path_outside_frames,
            "replay_timestamp_reversals":self.replay_reversals,
            "replay_scan_gaps_over_2s":self.replay_gaps,
            "retained_reference_seq":self.path_seq,
            "reference_retained":self.path is not None,
            "unmatched_body_queue":len(self.poses),
            "unmatched_tracks_queue":len(self.reports),
        }
