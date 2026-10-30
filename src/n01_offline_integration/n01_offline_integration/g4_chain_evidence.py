"""G4.7: independent, fail-closed evidence for the historical Bag -> SCAN -> G3 chain.

Event ordering is based on observer callbacks, NOT ROS message header time.
Passing this *functional observation* never certifies terrain, collisions,
dynamic-object truth, closed-loop behavior, or permission to move a robot.
"""
import math
from .scan_reference_artifact import SCHEMA

EVIDENCE_SCHEMA = "n01.offline.g4_chain_evidence.v1"


class ChainEvidence:
    def __init__(self, reference):
        if (reference.get("schema") != SCHEMA or
            reference.get("frame_id") != "map" or
            reference.get("safety_authorization") != "NEVER" or
            reference.get("actuator_output") != "DISABLED" or
            reference.get("physical_execution_forbidden") is not True):
            raise ValueError("untrusted G4.5 reference provenance")
        points = reference.get("path_xyz_m")
        if not isinstance(points, list) or not 2 <= len(points) <= 200:
            raise ValueError("invalid reference waypoints")
        if any(not isinstance(p, list) or len(p) != 3 or
               any(type(x) not in (int, float) or not math.isfinite(x) for x in p)
               for p in points):
            raise ValueError("non-finite reference point")
        self.points = points
        self.counts = {
            "reference_publish_status": 0,
            "matching_initial_paths": 0,
            "nonmatching_initial_paths": 0,
            "native_valid_bspline_before_reference": 0,
            "native_valid_bspline_after_reference": 0,
            "native_invalid_bspline_after_reference": 0,
            "scan_bridge_paths_after_reference": 0,
            "dynamic_track_reports_after_reference": 0,
            "g3_solver_reports_after_reference": 0,
            "g3_candidate_paths_after_reference": 0,
        }
        self.statuses = []
        self.started = False
        self.cmd_vel_seen = False
        self.graph_query_failed = False

    def reference_status(self, data):
        if not isinstance(data, dict) or data.get("schema") != "n01.offline.g4_reference_status.v1":
            return
        if data.get("actuator_output") != "DISABLED" or data.get("safety_authorization") != "NEVER":
            self.cmd_vel_seen = True
        status = data.get("status")
        if isinstance(status, str):
            self.statuses.append(status)
            self.statuses = self.statuses[-30:]
            if status == "UNVERIFIED_REFERENCE_SENT_TO_OFFLINE_SCAN":
                self.counts["reference_publish_status"] += 1

    def initial_path(self, frame, xyz):
        if (frame != "map" or len(xyz) != len(self.points) or
            any(len(p) != 3 or
                any(not math.isfinite(v) for v in p) or
                math.dist(p, expect) > 1e-3
                for p, expect in zip(xyz, self.points))):
            self.counts["nonmatching_initial_paths"] += 1
            return
        self.counts["matching_initial_paths"] += 1
        # Only a matching /initial_path can establish the G4 ordering marker.
        self.started = True

    def native_bspline(self, valid):
        if self.started:
            key = ("native_valid_bspline_after_reference" if valid
                   else "native_invalid_bspline_after_reference")
            self.counts[key] += 1
        elif valid:
            self.counts["native_valid_bspline_before_reference"] += 1

    def bridge_path(self):
        if self.started:
            self.counts["scan_bridge_paths_after_reference"] += 1

    def dynamic_report(self):
        if self.started:
            self.counts["dynamic_track_reports_after_reference"] += 1

    def g3_report(self):
        if self.started:
            self.counts["g3_solver_reports_after_reference"] += 1

    def g3_path(self):
        if self.started:
            self.counts["g3_candidate_paths_after_reference"] += 1

    def finalize(self):
        c = self.counts
        blockers = []
        if self.cmd_vel_seen:
            blockers.append("CMD_VEL_PUBLISHER_OR_UNSAFE_REFERENCE_STATUS")
        if self.graph_query_failed:
            blockers.append("ROS_GRAPH_GUARD_QUERY_FAILED")
        if c["matching_initial_paths"] != 1:
            blockers.append("G4_REFERENCE_NOT_OBSERVED_EXACTLY_ONCE")
        if c["reference_publish_status"] != 1:
            blockers.append("G4_PUBLISHER_STATUS_NOT_OBSERVED_EXACTLY_ONCE")
        if c["native_valid_bspline_after_reference"] == 0:
            blockers.append("NO_NATIVE_SCAN_BSPLINE_AFTER_G4_REFERENCE")
        if c["scan_bridge_paths_after_reference"] == 0:
            blockers.append("NO_SCAN_BRIDGE_PATH_AFTER_G4_REFERENCE")
        if c["dynamic_track_reports_after_reference"] == 0:
            blockers.append("NO_G2_REPORT_AFTER_G4_REFERENCE")
        if c["g3_solver_reports_after_reference"] == 0:
            blockers.append("NO_G3_REPORT_AFTER_G4_REFERENCE")
        if c["g3_candidate_paths_after_reference"] == 0:
            blockers.append("NO_G3_CANDIDATE_PATH_AFTER_G4_REFERENCE")
        return {
            "schema": EVIDENCE_SCHEMA,
            "result": ("G4_TO_G3_TOPIC_FLOW_OBSERVED_NOT_SAFETY_CERTIFIED"
                       if not blockers else "G4_CHAIN_INCOMPLETE"),
            "issues": blockers,
            "counts": dict(c),
            "reference_status_history": list(self.statuses),
            "strict_callback_observation_order": True,
            "static_full_pcd_integrated_into_scan_grid": False,
            "g3_output_proven_to_follow_the_new_g4_route": False,
            "terrain_and_collision_verified": False,
            "dynamic_obstacles_ground_truth_verified": False,
            "historical_open_loop_only": True,
            "safety_authorization": "NEVER",
            "actuator_output": "DISABLED",
        }
