"""Non-actuating dynamic-avoidance interface placeholder.

Consumes SCAN reference paths, bridge state and OFFLINE moving-obstacle
hypotheses. Reports the constant-velocity predictor interface; NO Kalman,
MPC/CBF solver, safety authorization or command publisher exists.
"""
import json
import os
if os.environ.get("ROS_DOMAIN_ID") != "88" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise RuntimeError("Dynamic port is offline-only")
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Path
from std_msgs.msg import String


class DynamicPort(Node):
    def __init__(self):
        super().__init__("n01_dynamic_avoidance_interface")
        self.path_count=0
        self.last_ready="NOT_READY"
        self.track_messages=0
        self.processed_track_frames=0
        self.last_tracks={}
        self.max_moving_hypotheses=0
        self.risk_reports=0
        self.risk_evaluations=0
        self.risk_conflicts=0
        self.last_risk_status="NOT_RECEIVED"
        self.risk_aligned_frames=0
        self.risk_path_reused_frames=0
        self.risk_min_barrier_proxy=None
        self.risk_worst_track_id=None
        self.risk_worst_scan_stamp_ns=None
        self.risk_posthoc_projections=0
        self.risk_posthoc_evaluations=0
        self.g3_reports_seen=0
        self.g3_solver_attempts=0
        self.g3_dynamic_candidate_solutions=0
        self.g3_last_status="NOT_RECEIVED"
        self.status=self.create_publisher(String,"/n01/offline/dynamic_port_status",10)
        self.create_subscription(Path,"/nav/test/global_path",self.on_path,10)
        self.create_subscription(String,"/n01/offline/readiness",self.on_ready,10)
        self.create_subscription(String,"/n01/offline/dynamic_tracks",
                                 self.on_tracks,10)
        self.create_subscription(String,"/n01/offline/path_risk",
                                 self.on_risk,10)
        self.create_subscription(String,"/n01/offline/g3_solver_report",
                                 self.on_g3,10)
        self.create_timer(1.0,self.tick)

    def on_path(self,msg):
        if msg.poses:
            self.path_count+=1

    def on_ready(self,msg):
        try:
            self.last_ready=json.loads(msg.data).get("readiness","NOT_READY")
        except ValueError:
            self.last_ready="NOT_READY"

    def on_tracks(self,msg):
        self.track_messages+=1
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):
                raise ValueError("non-object dynamic track report")
            self.last_tracks=data
            if data.get("schema")=="n01.offline.tracks.v1" and data.get("status")=="PROCESSED":
                self.processed_track_frames+=1
                self.max_moving_hypotheses=max(
                    self.max_moving_hypotheses,int(data.get("moving_hypotheses",0)))
        except (ValueError,TypeError):
            self.last_tracks={"parse_error":True}

    def on_risk(self,msg):
        self.risk_reports+=1
        try:
            risk=json.loads(msg.data)
            if not isinstance(risk,dict):
                raise ValueError("non-object risk report")
            if risk.get("schema")=="n01.offline.path_risk.v1":
                self.last_risk_status=risk.get("status","INVALID")
                self.risk_evaluations=max(
                    self.risk_evaluations,int(risk.get("evaluated_previews",0)))
                self.risk_conflicts=max(
                    self.risk_conflicts,int(risk.get("potential_conflict_previews",0)))
                self.risk_aligned_frames=max(
                    self.risk_aligned_frames,int(risk.get("aligned_replay_frames",0)))
                self.risk_path_reused_frames=max(
                    self.risk_path_reused_frames,
                    int(risk.get("aligned_replay_frames",0))-
                    int(risk.get("aligned_frames_before_first_path",0))-
                    int(risk.get("body_outside_reference_frames",0)))
                self.risk_min_barrier_proxy=risk.get(
                    "minimum_barrier_proxy_across_recorded_frames_m2")
                self.risk_worst_track_id=risk.get("worst_recorded_frame_track_id")
                self.risk_worst_scan_stamp_ns=risk.get("worst_recorded_frame_stamp_ns")
                self.risk_posthoc_projections=max(
                    self.risk_posthoc_projections,
                    int(risk.get("posthoc_projected_reference_frames",0)))
                self.risk_posthoc_evaluations=max(
                    self.risk_posthoc_evaluations,
                    int(risk.get("retrospective_evaluable_previews",0)))
        except (ValueError,TypeError):
            self.last_risk_status="INVALID"

    def on_g3(self,msg):
        self.g3_reports_seen+=1
        try:
            item=json.loads(msg.data)
            if item.get("schema")!="n01.offline.g3_mpc_cbf.v1":
                return
            self.g3_solver_attempts=max(
                self.g3_solver_attempts,int(item.get("solver_executed_frames",0)))
            self.g3_dynamic_candidate_solutions=max(
                self.g3_dynamic_candidate_solutions,
                int(item.get("dynamic_cbf_candidate_solutions",0)))
            self.g3_last_status=item.get("last_solver_status","UNKNOWN")
        except (AttributeError,ValueError,TypeError,OverflowError):
            self.g3_last_status="BAD_REPORT"

    def tick(self):
        result={"dynamic_detection":("HEURISTIC_VOXEL_TRACKING_RESEARCH_ONLY"
                  if self.processed_track_frames else "WAITING_FOR_PROCESSED_CLOUD"),
                "short_horizon_prediction":("CONSTANT_VELOCITY_RESEARCH_ONLY"
                  if self.processed_track_frames else "NOT_READY"),
                "trajectory_prediction_verified_on_real_dynamic_obstacles":False,
                "dynamic_tracks_messages_seen":self.track_messages,
                "dynamic_processed_frames_seen":self.processed_track_frames,
                "maximum_moving_hypotheses_seen":self.max_moving_hypotheses,
                "last_dynamic_input_status":self.last_tracks.get("status","NOT_RECEIVED"),
                "path_risk_interface":"DIAGNOSTIC_ONLY_NO_ACTUATION",
                "path_risk_reports_seen":self.risk_reports,
                "path_risk_evaluations_seen":self.risk_evaluations,
                "path_risk_conflict_previews_seen":self.risk_conflicts,
                "path_risk_exact_replay_frames":self.risk_aligned_frames,
                "path_risk_retained_reference_frames_estimated":self.risk_path_reused_frames,
                "path_risk_minimum_barrier_proxy_m2":self.risk_min_barrier_proxy,
                "path_risk_worst_track_id":self.risk_worst_track_id,
                "path_risk_worst_recorded_scan_ns":self.risk_worst_scan_stamp_ns,
                "path_risk_posthoc_projected_frames":self.risk_posthoc_projections,
                "path_risk_posthoc_evaluable_previews":self.risk_posthoc_evaluations,
                "last_path_risk_status":self.last_risk_status,
                "g3_offline_approximate_mpc_cbf":"FINITE_CONTROL_BEAM_SEARCH_RESEARCH_ONLY",
                "g3_solver_reports_seen":self.g3_reports_seen,
                "g3_solver_attempts_seen":self.g3_solver_attempts,
                "g3_dynamic_candidate_solutions_seen":self.g3_dynamic_candidate_solutions,
                "g3_last_solver_status":self.g3_last_status,
                "kalman_prediction":"NOT_IMPLEMENTED",
                "mpc_cbf_solver":"NOT_IMPLEMENTED",
                "actuator_output":"DISABLED",
                "command_publisher_created":False,
                "reference_path_messages_seen":self.path_count,
                "upstream_readiness":self.last_ready}
        self.status.publish(String(data=json.dumps(result,sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node=DynamicPort()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=="__main__":
    main()
