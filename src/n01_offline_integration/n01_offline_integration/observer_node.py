"""Offline-only integration observer with atomic JSON evidence file."""
import json
import os
from pathlib import Path as FilePath
import time

if os.environ.get("ROS_DOMAIN_ID") != "88" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise RuntimeError("Offline observer refuses non-isolated ROS environment")

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped
from scan_planner_msgs.msg import Bspline
from n01_scan_dynacbf_bridge.scan_g1_monitor import valid_bspline


class IntegrationObserver(Node):
    def __init__(self):
        super().__init__("n01_offline_integration_observer")
        self.declare_parameter("historical_bag_mode", False)
        self.historical_mode = bool(self.get_parameter("historical_bag_mode").value)
        self.declare_parameter("duration_s", 75.0)
        self.declare_parameter("report_path", "")
        self.duration = float(self.get_parameter("duration_s").value)
        target = self.get_parameter("report_path").value
        if not target:
            target = os.environ.get("N01_OFFLINE_REPORT_PATH","")
        if not target or not 10 <= self.duration <= 180:
            raise ValueError("duration and report_path required")
        self.path = FilePath(target)
        self.start = time.monotonic()
        self.done = False
        self.counts = {"transformed_clouds":0,"wrong_frame_clouds":0,
                       "body_pose":0,"sensor_pose":0,"map_messages":0,
                       "max_inflated_points":0,"goals":0,"splines":0,"valid_splines":0,
                       "invalid_splines":0,"readiness_updates":0,
                       "reference_paths":0,"dynamic_port_status":0,
                       "path_risk_messages":0,"path_risk_evaluable":0,
                       "path_risk_aligned_frames":0,
                       "path_risk_retained_frames":0,
                       "path_risk_posthoc_messages":0,
                       "path_risk_posthoc_projected_frames":0,
                       "path_risk_conflict_frames":0,
                       "dynamic_track_messages":0,"dynamic_track_processed":0,
                       "moving_hypothesis_frames":0,
                       "g3_solver_reports":0,
                       "g3_exact_stamp_pairs":0,
                       "g3_solver_attempts":0,
                       "g3_nominal_only_frames":0,
                       "g3_dynamic_cbf_candidate_frames":0,
                       "g3_posthoc_solver_frames":0,
                       "g3_candidate_paths":0}
        self.health = {}
        self.bag_health = {}
        self.cmd_seen = False
        self.create_subscription(PointCloud2,"/cloud",self.on_cloud,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,"/grid_map/occupancy_inflate",
                                 self.on_map,qos_profile_sensor_data)
        self.create_subscription(Odometry,"/body_pose",self.on_body,qos_profile_sensor_data)
        self.create_subscription(Odometry,"/sensor_pose",self.on_sensor,qos_profile_sensor_data)
        self.create_subscription(PoseStamped,"/move_base_simple/goal",self.on_goal,10)
        self.create_subscription(Bspline,"/planning/bspline",self.on_spline,10)
        self.create_subscription(String,"/n01/offline/readiness",self.on_health,10)
        if self.historical_mode:
            self.create_subscription(String,"/n01/offline/bag_replay_status",
                                     self.on_bag_health,10)
        self.create_subscription(RosPath,"/nav/test/global_path",self.on_path,10)
        self.create_subscription(String,"/n01/offline/dynamic_port_status",
                                 self.on_dynamic_port,10)
        self.create_subscription(String,"/n01/offline/dynamic_tracks",
                                 self.on_dynamic_tracks,10)
        self.dynamic_port = {}
        self.dynamic_tracks = {}
        self.path_risk = {}
        self.g3 = {}
        self.create_subscription(String,"/n01/offline/path_risk",
                                 self.on_path_risk,10)
        if self.historical_mode:
            self.create_subscription(String,"/n01/offline/g3_solver_report",
                                     self.on_g3,10)
            self.create_subscription(RosPath,"/n01/offline/g3_candidate_path",
                                     self.on_g3_candidate,10)
        self.create_timer(.5,self.tick)

    def on_cloud(self, msg):
        if msg.header.frame_id == "map":
            self.counts["transformed_clouds"] += 1
        else:
            self.counts["wrong_frame_clouds"] += 1

    def on_map(self, msg):
        self.counts["map_messages"] += 1
        self.counts["max_inflated_points"] = max(
            self.counts["max_inflated_points"],int(msg.width)*int(msg.height))

    def on_body(self, _):
        self.counts["body_pose"] += 1

    def on_sensor(self, _):
        self.counts["sensor_pose"] += 1

    def on_goal(self, _):
        self.counts["goals"] += 1

    def on_spline(self, msg):
        self.counts["splines"] += 1
        self.counts["valid_splines" if valid_bspline(msg) else "invalid_splines"] += 1

    def on_path(self, msg):
        if msg.poses and msg.header.frame_id == "map":
            self.counts["reference_paths"] += 1

    def on_g3_candidate(self,msg):
        if msg.poses and msg.header.frame_id=="map":
            self.counts["g3_candidate_paths"]+=1

    def on_g3(self,msg):
        self.counts["g3_solver_reports"]+=1
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict) or data.get("schema")!=(
                    "n01.offline.g3_mpc_cbf.v1"):
                return
            self.g3=data
            # ROS candidate messages may be bursty during post-hoc Bag
            # backfill; count the producer's cumulative evidence as well.
            self.counts["g3_candidate_paths"]=max(
                self.counts["g3_candidate_paths"],
                int(data.get("g3_candidate_paths_published",0)))
            keys={
                "g3_exact_stamp_pairs":"exact_recorded_stamp_pairs",
                "g3_solver_attempts":"solver_executed_frames",
                "g3_nominal_only_frames":"nominal_only_solutions",
                "g3_dynamic_cbf_candidate_frames":"dynamic_cbf_candidate_solutions",
                "g3_posthoc_solver_frames":"posthoc_solver_frames",
            }
            for local,remote in keys.items():
                self.counts[local]=max(
                    self.counts[local],int(data.get(remote,0)))
        except (ValueError,TypeError,OverflowError):
            self.g3={"parse_error":True}

    def on_path_risk(self,msg):
        self.counts["path_risk_messages"] += 1
        try:
            risk=json.loads(msg.data)
            if not isinstance(risk,dict):
                raise ValueError("non-object risk report")
            self.path_risk=risk
            # Cumulative metadata on each report remains available even when
            # individual post-hoc messages are lost by a ROS queue.
            self.counts["path_risk_posthoc_projected_frames"]=max(
                self.counts["path_risk_posthoc_projected_frames"],
                int(risk.get("posthoc_projected_reference_frames",0)))
            if risk.get("is_retrospective_path_comparison") is True:
                self.counts["path_risk_posthoc_messages"]+=1
            if risk.get("is_new_aligned_recorded_frame") is True:
                self.counts["path_risk_aligned_frames"] += 1
                if risk.get("reference_reused_without_replanning") is True:
                    self.counts["path_risk_retained_frames"] += 1
                if risk.get("evaluated_pairs",0)>0:
                    self.counts["path_risk_evaluable"] += 1
                if risk.get("status")=="POTENTIAL_PROXIMITY_CONFLICT":
                    self.counts["path_risk_conflict_frames"] += 1
        except (ValueError,TypeError):
            self.path_risk={"parse_error":True}

    def on_dynamic_tracks(self,msg):
        self.counts["dynamic_track_messages"] += 1
        try:
            data=json.loads(msg.data)
            self.dynamic_tracks=data
            if data.get("status")=="PROCESSED" and data.get("frame_id")=="map":
                self.counts["dynamic_track_processed"] += 1
                if int(data.get("moving_hypotheses",0))>0:
                    self.counts["moving_hypothesis_frames"] += 1
        except (ValueError,TypeError):
            self.dynamic_tracks={"parse_error":True}

    def on_dynamic_port(self, msg):
        self.counts["dynamic_port_status"] += 1
        try:
            self.dynamic_port = json.loads(msg.data)
        except ValueError:
            self.dynamic_port = {"parse_error":True}

    def on_bag_health(self,msg):
        try:
            self.bag_health=json.loads(msg.data)
        except ValueError:
            self.bag_health={"parse_error":True}

    def on_health(self, msg):
        self.counts["readiness_updates"] += 1
        try:
            self.health = json.loads(msg.data)
        except ValueError:
            self.health = {"parse_error":True}

    def tick(self):
        try:
            self.cmd_seen = self.cmd_seen or bool(
                self.get_publishers_info_by_topic("/cmd_vel"))
        except Exception:
            self.cmd_seen = True
        if not self.done and time.monotonic() - self.start >= self.duration:
            self.finish()

    def finish(self):
        if self.done:
            return
        self.done = True
        c=self.counts
        issues=[]
        if self.cmd_seen or self.health.get("cmd_vel_seen") or self.bag_health.get("cmd_vel_publishers_seen"):
            issues.append("unexpected_cmd_vel_publisher")
        if c["wrong_frame_clouds"]:
            issues.append("cloud_not_in_map")
        if not c["transformed_clouds"] or not c["body_pose"] or not c["sensor_pose"]:
            issues.append("tf_pointcloud_adapter_not_observed")
        if not c["map_messages"] or not c["max_inflated_points"]:
            issues.append("no_real_scan_inflated_map")
        if not c["valid_splines"]:
            issues.append("no_real_scan_valid_bspline")
        if not c["goals"]:
            issues.append("no_goal_observed")
        if not c["reference_paths"]:
            issues.append("no_bspline_reference_path")
        if not c["dynamic_port_status"]:
            issues.append("no_dynamic_interface_status")
        elif (self.dynamic_port.get("mpc_cbf_solver") != "NOT_IMPLEMENTED"
              or self.dynamic_port.get("actuator_output") != "DISABLED"):
            issues.append("dynamic_port_incorrectly_armed")
        if self.historical_mode:
            if not c["path_risk_messages"]:
                issues.append("no_offline_path_risk_diagnostic")
            if not c["path_risk_aligned_frames"]:
                issues.append("no_exact_timestamp_aligned_replay_frames")
            if not (c["path_risk_retained_frames"] or
                    c["path_risk_posthoc_projected_frames"]):
                issues.append("no_usable_path_projection_live_or_posthoc")
            if not c["dynamic_track_processed"]:
                issues.append("no_dynamic_map_cloud_processing")
            if not c["g3_exact_stamp_pairs"]:
                issues.append("g3_no_real_bag_exact_stamp_inputs")
            if not c["g3_solver_attempts"]:
                issues.append("g3_real_bag_data_never_reached_optimizer")
            if not c["g3_candidate_paths"]:
                issues.append("g3_no_candidate_path_from_real_bag")
            if not self.bag_health:
                issues.append("historical_bag_status_missing")
            if not self.bag_health.get("output_clouds",0):
                issues.append("historical_bag_cloud_not_delivered")
        if self.health.get("tf_conflict"):
            issues.append("base_link_height_conflict")
        if not self.historical_mode and self.health.get("readiness") != "OFFLINE_FIXTURE_ONLY":
            issues.append("fixture_readiness_not_observed")
        data={"test_type":("N01_HISTORICAL_BAG_FUNCTIONAL_REPLAY" if self.historical_mode
                           else "N01_OFFLINE_INTEGRATION_FIXTURE"),
              "result":(
                  ("BAG_FUNCTIONAL_PATH_OBSERVED" if not issues else
                   "BAG_INPUTS_OBSERVED_BUT_PATH_INCOMPLETE" if c["transformed_clouds"] and c["body_pose"]
                   else "BAG_DATA_FLOW_NOT_COMPLETE")
                  if self.historical_mode else
                  ("OFFLINE_INTEGRATION_OBSERVED" if not issues else "NOT_COMPLETE")),
              "issues":issues,"counts":c,"last_bridge_health":self.health,
              "dynamic_port":self.dynamic_port,
              "last_dynamic_tracks":self.dynamic_tracks,
              "last_path_risk":self.path_risk,
              "g3_offline_solver":self.g3,
              "g3_discrete_mpc_cbf_prototype_executed":
                  bool(c["g3_solver_attempts"]),
              "historical_bag_health":self.bag_health,
              "historical_bag_mode":self.historical_mode,
              "old_height_from_bag_ignored_in_reconstructed_transform":self.historical_mode,
              "cmd_vel_publishers_seen":self.cmd_seen,
              "slam_real_status_tested":False,"real_bag_tested":self.historical_mode,
              "real_hive2_motion_tested":False,
              "dynamic_mpc_cbf_tested":False,
              "g1_full_collision_acceptance":False}
        self.path.parent.mkdir(parents=True,exist_ok=True)
        tmp=self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
        tmp.replace(self.path)
        print("N01_OFFLINE_INTEGRATION="+json.dumps(data,ensure_ascii=False),flush=True)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node=IntegrationObserver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=="__main__":
    main()
