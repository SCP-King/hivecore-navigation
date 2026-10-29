"""Open-loop historical-frame Path diagnostic, never a safety clearance.

Record /body_pose and /dynamic_tracks only with equal *recorded scan stamps*.
SCAN Path has a live wall-clock header and is retained as UNVERIFIED geometry.
When Path arrives late, buffered older matched scans may be compared POST HOC;
these results MUST NOT be described as contemporaneous planning or control.

No ROS dependency. No actuator API, no CBF/MPC solver or safety authorization.
"""
from collections import OrderedDict, deque
import math

from .path_risk_core import evaluate_path_preview


def finite_xy(value):
    return (isinstance(value,(tuple,list)) and len(value)>=2
            and all(isinstance(v,(int,float)) and
                    not isinstance(v,bool) and math.isfinite(v)
                    for v in value[:2]))


def project_path(path,body_xy):
    """Nearest XY projection, along-path progress and remaining geometry."""
    if not isinstance(path,list) or len(path)<2 or not finite_xy(body_xy):
        return None
    best=None
    progress=0.
    for i,(a,b) in enumerate(zip(path,path[1:])):
        if not finite_xy(a) or not finite_xy(b):
            return None
        dx,dy=b[0]-a[0],b[1]-a[1]
        length=math.hypot(dx,dy)
        if length<=1e-8:
            continue
        u=max(0.,min(1.,((body_xy[0]-a[0])*dx+
                         (body_xy[1]-a[1])*dy)/(length*length)))
        xy=(a[0]+u*dx,a[1]+u*dy)
        deviation=math.hypot(xy[0]-body_xy[0],xy[1]-body_xy[1])
        if best is None or deviation<best[0]:
            best=(deviation,progress+u*length,
                  [xy]+[tuple(p[:2]) for p in path[i+1:]])
        progress+=length
    return best


class ReplayPathSynchronizer:
    """Bounded exact-stamp matching, strict Path projection and retro evidence."""

    def __init__(self, max_path_distance_m=1.5, max_pairs=16,
                 assumed_speed_mps=.45, assumed_clearance_m=.60,
                 max_posthoc_frames=160):
        if not (.1<=max_path_distance_m<=5. and
                2<=max_pairs<=128 and
                8<=max_posthoc_frames<=1000):
            raise ValueError("invalid synchronizer bounds")
        self.max_path_distance_m=max_path_distance_m
        self.max_pairs=max_pairs
        self.assumed_speed_mps=assumed_speed_mps
        self.assumed_clearance_m=assumed_clearance_m
        self.path=None
        self.path_header_ns=None
        self.path_seq=0
        self.poses=OrderedDict()
        self.reports=OrderedDict()
        self.before_path=deque(maxlen=max_posthoc_frames)
        self._posthoc_out=[]
        self.last_completed_ns=None
        self.frames_aligned=0
        self.live_projected_frames=0
        self.live_proximity_pairs=0
        self.live_conflicts=0
        self.no_path_frames=0
        self.path_outside_frames=0
        self.path_end_frames=0
        self.posthoc_compared_frames=0
        self.posthoc_projected_frames=0
        self.posthoc_proximity_pairs=0
        self.posthoc_conflicts=0
        self.posthoc_outside_frames=0
        self.replay_reversals=0
        self.replay_gaps=0
        # Preserve old field names used by earlier tests/consumers.
        self.proximity_pairs=0
        self.proximity_conflicts=0

    def set_path(self,points,header_stamp_ns=0):
        self.path_seq+=1
        self.path_header_ns=header_stamp_ns
        if (not isinstance(points,(list,tuple)) or not 2<=len(points)<=3000
                or any(not finite_xy(p) for p in points)):
            self.path=None
            self.before_path.clear()
            self._posthoc_out=[]
            return False
        self.path=[(float(p[0]),float(p[1])) for p in points]
        # Post-hoc comparison is only performed on frames paired before
        # the first reference arrived; never count them as new live frames.
        self._posthoc_out=[]
        for stamp,body,tracks in self.before_path:
            result=self._compare(stamp,body,tracks,posthoc=True)
            self._posthoc_out.append(result)
        self.before_path.clear()
        return True

    def drain_posthoc(self):
        items=self._posthoc_out
        self._posthoc_out=[]
        return items

    def _push(self,cache,stamp,value):
        if not isinstance(stamp,int) or isinstance(stamp,bool) or stamp<=0:
            return False
        cache[stamp]=value
        cache.move_to_end(stamp)
        while len(cache)>self.max_pairs:
            cache.popitem(last=False)
        return True

    def put_body(self,stamp_ns,xy):
        if not finite_xy(xy):
            return None
        if not self._push(self.poses,stamp_ns,tuple(float(v) for v in xy[:2])):
            return None
        return self._pair(stamp_ns)

    def put_tracks(self,message):
        if (not isinstance(message,dict)
                or message.get("schema")!="n01.offline.tracks.v1"
                or message.get("frame_id")!="map"
                or message.get("status")!="PROCESSED"):
            return None
        stamp=message.get("stamp_ns")
        tracks=message.get("tracks")
        if not isinstance(tracks,list) or len(tracks)>200:
            return None
        if not self._push(self.reports,stamp,tracks):
            return None
        return self._pair(stamp)

    def _pair(self,stamp):
        if stamp not in self.poses or stamp not in self.reports:
            return None
        body=self.poses.pop(stamp)
        tracks=self.reports.pop(stamp)
        if self.last_completed_ns is not None and stamp<=self.last_completed_ns:
            self.replay_reversals+=1
            self.poses.clear()
            self.reports.clear()
            self.before_path.clear()
            self._posthoc_out=[]
            self.last_completed_ns=stamp
            self.path=None
            return self._inconclusive("REPLAY_TIMESTAMP_REVERSED",stamp)
        if (self.last_completed_ns is not None and
                stamp-self.last_completed_ns>2_000_000_000):
            self.replay_gaps+=1
        self.last_completed_ns=stamp
        self.frames_aligned+=1
        if self.path is None:
            self.no_path_frames+=1
            self.before_path.append((stamp,body,tracks))
            return self._inconclusive("NO_RETAINED_REFERENCE_GEOMETRY",stamp)
        return self._compare(stamp,body,tracks,posthoc=False)

    def _compare(self,stamp,body,tracks,posthoc):
        if posthoc:
            self.posthoc_compared_frames+=1
        projection=project_path(self.path,body)
        if projection is None:
            result=self._inconclusive("INVALID_REFERENCE_GEOMETRY",stamp)
            deviation=progress=None
            usable=False
        else:
            deviation,progress,local_path=projection
            if deviation>self.max_path_distance_m:
                if posthoc:self.posthoc_outside_frames+=1
                else:self.path_outside_frames+=1
                result=self._inconclusive("BODY_OUTSIDE_RETAINED_PATH_CORRIDOR",stamp)
                usable=False
            elif len(local_path)<2 or all(
                    math.hypot(q[0]-local_path[0][0],
                               q[1]-local_path[0][1])<=1e-8
                    for q in local_path[1:]):
                if not posthoc:self.path_end_frames+=1
                result=self._inconclusive("AT_REFERENCE_PATH_END",stamp)
                usable=False
            else:
                result=evaluate_path_preview(
                    local_path,tracks,assumed_speed_mps=self.assumed_speed_mps,
                    assumed_clearance_m=self.assumed_clearance_m)
                usable=True
                if posthoc:self.posthoc_projected_frames+=1
                else:self.live_projected_frames+=1
                if result["evaluated_pairs"]>0:
                    if posthoc:self.posthoc_proximity_pairs+=1
                    else:
                        self.live_proximity_pairs+=1
                        self.proximity_pairs+=1
                if result["status"]=="POTENTIAL_PROXIMITY_CONFLICT":
                    if posthoc:self.posthoc_conflicts+=1
                    else:
                        self.live_conflicts+=1
                        self.proximity_conflicts+=1
        result.update({
            "recorded_scan_stamp_ns":stamp,
            "time_alignment":"EXACT_RECORDED_BODY_POSE_AND_TRACK_STAMP",
            "path_header_stamp_ns":self.path_header_ns,
            "path_header_is_historical_stamp":False,
            "path_reference_seq":self.path_seq,
            "reference_provenance":(
                "POST_HOC_PATH_GEOMETRY_COMPARISON_NOT_HISTORICAL_PLANNING"
                if posthoc else "RETAINED_SCAN_PATH_GEOMETRY_UNVERIFIED"),
            "post_hoc_reference_comparison":posthoc,
            "contemporaneous_path_at_recorded_scan":False,
            "current_body_map_xy_m":[round(v,4) for v in body],
            "body_to_reference_distance_m":(
                round(deviation,4) if deviation is not None else None),
            "projected_path_progress_m":(
                round(progress,4) if progress is not None else None),
            "path_projection_usable":usable,
            "reference_reused_without_replanning":usable and not posthoc,
        })
        return result

    def _inconclusive(self,reason,stamp):
        result=evaluate_path_preview([],[])
        result["status"]=reason
        result.update({
            "recorded_scan_stamp_ns":stamp,
            "time_alignment":"EXACT_RECORDED_BODY_POSE_AND_TRACK_STAMP",
            "path_header_is_historical_stamp":False,
            "path_reference_seq":self.path_seq,
            "reference_provenance":"UNVERIFIED_OR_UNAVAILABLE",
            "reference_reused_without_replanning":False,
            "path_projection_usable":False,
            "post_hoc_reference_comparison":False,
            "contemporaneous_path_at_recorded_scan":False,
        })
        return result

    def evidence(self):
        return {
            "aligned_replay_frames":self.frames_aligned,
            "aligned_frames_with_proximity_pairs":self.live_proximity_pairs,
            "potential_conflict_frames":self.live_conflicts,
            "aligned_frames_before_first_path":self.no_path_frames,
            "body_outside_reference_frames":self.path_outside_frames,
            "path_end_frames":self.path_end_frames,
            "live_projected_reference_frames":self.live_projected_frames,
            "posthoc_compared_frames":self.posthoc_compared_frames,
            "posthoc_projected_reference_frames":self.posthoc_projected_frames,
            "posthoc_proximity_pairs":self.posthoc_proximity_pairs,
            "posthoc_potential_conflicts":self.posthoc_conflicts,
            "posthoc_body_outside_frames":self.posthoc_outside_frames,
            "posthoc_buffer_pending":len(self.before_path),
            "replay_timestamp_reversals":self.replay_reversals,
            "replay_scan_gaps_over_2s":self.replay_gaps,
            "retained_reference_seq":self.path_seq,
            "reference_retained":self.path is not None,
            "unmatched_body_queue":len(self.poses),
            "unmatched_tracks_queue":len(self.reports),
        }
