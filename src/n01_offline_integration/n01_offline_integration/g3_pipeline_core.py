"""G3 offline-only orchestration: recorded Bag data -> horizon CBF search.

An exact recorded scan stamp is required for both /body_pose and the tracked
cloud. SCAN Path's wall-clock header is NOT a historical bag timestamp.
If Path arrives late, previously paired frames are replayed POST HOC as an
explicit counterfactual geometry comparison, never a historical control act.
"""
from collections import OrderedDict, deque
import math

from .g3_mpc_cbf_core import solve_discrete_mpc_cbf
from .g3_hive2_wheel_screen import screen_g3_result


class G3HistoricalCoordinator:
    def __init__(self,max_pending=32,max_backfill=160):
        if not 4<=max_pending<=128 or not 8<=max_backfill<=500:
            raise ValueError("invalid bounded G3 replay settings")
        self.poses=OrderedDict()
        self.tracks=OrderedDict()
        self.max_pending=max_pending
        self.prepath=deque(maxlen=max_backfill)
        self.path=None
        self.path_seq=0
        self.path_wall_header_ns=None
        self.last_stamp_ns=None
        self.input_pairs=0
        self.solve_attempts=0
        self.nominal_only=0
        self.dynamic_constraint_solutions=0
        self.infeasible=0
        self.invalid_path=0
        self.posthoc_attempts=0
        self.unmatched_on_exit=0
        self.clock_reversals=0
        self.last_solver_status="NO_INPUT"
        self.last_result=None
        self.full_reports=0
        self.hive2_screened_frames=0
        self.hive2_chassis_exceeded_frames=0
        self.hive2_scan_exceeded_frames=0
        self.hive2_within_translation_frames=0

    def _push(self,cache,stamp,value):
        if type(stamp) is not int or stamp<=0:
            return False
        cache[stamp]=value
        cache.move_to_end(stamp)
        while len(cache)>self.max_pending:
            cache.popitem(last=False)
        return True

    def put_pose(self,stamp,xyz):
        if not isinstance(xyz,(tuple,list)) or len(xyz)<3 or not all(
                isinstance(v,(int,float)) and math.isfinite(v) for v in xyz[:3]):
            return []
        if not self._push(self.poses,stamp,tuple(xyz[:3])):
            return []
        return self._pair(stamp)

    def put_tracks(self,payload):
        if not isinstance(payload,dict) or payload.get("schema")!=(
                "n01.offline.tracks.v1") or payload.get("status")!="PROCESSED" or (
                payload.get("frame_id")!="map"):
            return []
        stamp=payload.get("stamp_ns")
        tracks=payload.get("tracks")
        if not isinstance(tracks,list) or len(tracks)>200:
            return []
        if not self._push(self.tracks,stamp,tracks):
            return []
        return self._pair(stamp)

    def _pair(self,stamp):
        if stamp not in self.poses or stamp not in self.tracks:
            return []
        body=self.poses.pop(stamp)
        tracks=self.tracks.pop(stamp)
        if self.last_stamp_ns is not None and stamp<=self.last_stamp_ns:
            self.clock_reversals+=1
            self.poses.clear()
            self.tracks.clear()
            self.prepath.clear()
            self.path=None
            self.last_stamp_ns=stamp
            return [self._not_solved("REVERSED_RECORDED_SCAN_TIME",stamp)]
        self.last_stamp_ns=stamp
        self.input_pairs+=1
        if self.path is None:
            self.prepath.append((stamp,body,tracks))
            return [self._not_solved("WAITING_FOR_SCAN_PATH",stamp)]
        return [self._solve(stamp,body,tracks,posthoc=False)]

    def set_path(self,points,path_header_ns=0):
        self.path_seq+=1
        self.path_wall_header_ns=path_header_ns
        if (not isinstance(points,(tuple,list)) or
            not 2<=len(points)<=3000 or
            any(not isinstance(p,(tuple,list)) or len(p)<2 or
                not all(isinstance(v,(int,float)) and math.isfinite(v)
                        for v in p[:2]) for p in points)):
            self.path=None
            self.prepath.clear()
            self.invalid_path+=1
            return []
        self.path=[(float(p[0]),float(p[1])) for p in points]
        backlog=list(self.prepath)
        self.prepath.clear()
        return [self._solve(stamp,body,tracks,posthoc=True)
                for stamp,body,tracks in backlog]

    def _not_solved(self,reason,stamp):
        return {
            "schema":"n01.offline.g3_mpc_cbf.v1",
            "status":reason,"solver_executed":False,
            "recorded_scan_stamp_ns":stamp,
            "candidate_xy_m":[],
            "moving_constraint_tracks":0,
            "cbf_constraints_checked":0,
            "safety_authorization":"NEVER",
            "actuator_output":"DISABLED",
            "post_hoc_reference_comparison":False,
            "contemporaneous_path_at_recorded_scan":False,
            "path_reference_seq":self.path_seq,
        }

    def _solve(self,stamp,body,tracks,posthoc):
        # Path is an unverified reference; controls must NEVER be executed.
        result=solve_discrete_mpc_cbf(self.path,body,tracks)
        self.last_result=result
        self.last_solver_status=result["status"]
        if result["solver_executed"]:
            self.solve_attempts+=1
            if result["status"]=="FEASIBLE_NOMINAL_ONLY_NO_DYNAMIC_CONSTRAINTS":
                self.nominal_only+=1
            elif result["status"]=="FEASIBLE_CBF_CONSTRAINED_CANDIDATE_UNVERIFIED":
                self.dynamic_constraint_solutions+=1
            elif result["status"].startswith("INFEASIBLE"):
                self.infeasible+=1
        # G3.3 post-screen: compare the candidate to actual CONFIGURED Hive2
        # limits; do not transform G3 point-mass trajectory into wheel commands.
        # Unknown steering angles, yaw and measured twist remain INCONCLUSIVE.
        wheel_screen=screen_g3_result(result)
        result["hive2_motion_limit_screen"]=wheel_screen
        if wheel_screen["samples"] > 0:
            self.hive2_screened_frames+=1
            if wheel_screen["chassis_config_limits_exceeded"]:
                self.hive2_chassis_exceeded_frames+=1
            if wheel_screen["scan_planning_limits_exceeded"]:
                self.hive2_scan_exceeded_frames+=1
            if not wheel_screen["chassis_config_limits_exceeded"] and not (
                    wheel_screen["scan_planning_limits_exceeded"]):
                self.hive2_within_translation_frames+=1
        if posthoc:
            self.posthoc_attempts+=1
        self.full_reports+=1
        result.update({
            "recorded_scan_stamp_ns":stamp,
            "source":"REAL_HISTORICAL_BAG_DETECTED_TARGET_HYPOTHESES",
            "path_reference_seq":self.path_seq,
            "path_header_stamp_ns":self.path_wall_header_ns,
            "path_header_is_historical_stamp":False,
            "post_hoc_reference_comparison":bool(posthoc),
            "contemporaneous_path_at_recorded_scan":False,
            "actual_locomotion_dynamics_used":False,
            "calibrated_hive2_braking_model_used":False,
            "model_input_pose_map_xyz_m":list(body),
        })
        return result

    def evidence(self):
        return {
            "exact_recorded_stamp_pairs":self.input_pairs,
            "solver_executed_frames":self.solve_attempts,
            "nominal_only_solutions":self.nominal_only,
            "dynamic_cbf_candidate_solutions":self.dynamic_constraint_solutions,
            "hive2_xy_screened_candidate_frames":self.hive2_screened_frames,
            "hive2_chassis_limits_exceeded_frames":self.hive2_chassis_exceeded_frames,
            "hive2_scan_limits_exceeded_frames":self.hive2_scan_exceeded_frames,
            "hive2_translation_within_limits_frames_unverified_4ws":self.hive2_within_translation_frames,
            "hive2_4ws_execution_verified":False,
            "infeasible_solver_frames":self.infeasible,
            "posthoc_solver_frames":self.posthoc_attempts,
            "invalid_path_messages":self.invalid_path,
            "last_solver_status":self.last_solver_status,
            "waiting_pairs_for_path":len(self.prepath),
            "pending_pose_messages":len(self.poses),
            "pending_track_messages":len(self.tracks),
            "recorded_time_reversals":self.clock_reversals,
            "path_reference_seq":self.path_seq,
            "no_robot_commands_published":True,
        }
