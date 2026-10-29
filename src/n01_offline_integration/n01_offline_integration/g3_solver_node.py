"""G3 read-only Bag -> SCAN reference -> discrete MPC-CBF candidate publisher.

Offline diagnostics and candidate Path only. No Twist, TF, controller, motion
service, action, or actuator. These candidate points are NOT execution goals.
ROS_DOMAIN_ID 88, localhost-only and explicit historical Bag mode required.
"""
import json
import math
import os

if (os.environ.get("ROS_DOMAIN_ID")!="88" or
        os.environ.get("ROS_LOCALHOST_ONLY")!="1" or
        os.environ.get("N01_HISTORICAL_BAG_ONLY")!="1"):
    raise RuntimeError("G3 candidate optimizer requires isolated historical Bag mode")

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from .g3_pipeline_core import G3HistoricalCoordinator


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


class OfflineG3Solver(Node):
    def __init__(self):
        super().__init__("n01_g3_offline_optimizer")
        self.pipeline=G3HistoricalCoordinator()
        self.received_paths=0
        self.received_poses=0
        self.received_tracks=0
        self.published_reports=0
        self.published_candidates=0
        self.last=None
        self.report_pub=self.create_publisher(
            String,"/n01/offline/g3_solver_report",10)
        # A research-only diagnostic path. NOT /cmd_vel, /plan, /goal, etc.
        self.path_pub=self.create_publisher(
            Path,"/n01/offline/g3_candidate_path",10)
        self.create_subscription(Path,"/nav/test/global_path",self.on_path,10)
        self.create_subscription(Odometry,"/body_pose",self.on_body,
                                 qos_profile_sensor_data)
        self.create_subscription(String,"/n01/offline/dynamic_tracks",
                                 self.on_tracks,10)
        self.create_timer(2.,self.heartbeat)
        self.get_logger().warning(
            "G3 OFFLINE APPROXIMATE MPC-CBF; robot model not Hive2; "
            "candidate-only; absolutely NO actuator output")

    def on_path(self,msg):
        self.received_paths+=1
        if msg.header.frame_id.lstrip("/")!="map":
            self.pipeline.set_path([],stamp_ns(msg.header.stamp))
            return
        pts=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        for result in self.pipeline.set_path(pts,stamp_ns(msg.header.stamp)):
            self.publish_result(result)

    def on_body(self,msg):
        self.received_poses+=1
        if msg.header.frame_id.lstrip("/")!="map":
            return
        p=msg.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):
            return
        for result in self.pipeline.put_pose(
                stamp_ns(msg.header.stamp),(p.x,p.y,p.z)):
            self.publish_result(result)

    def on_tracks(self,msg):
        self.received_tracks+=1
        try:
            data=json.loads(msg.data)
        except (ValueError,TypeError):
            return
        for result in self.pipeline.put_tracks(data):
            self.publish_result(result)

    def publish_result(self,result):
        self.last=dict(result)
        if result.get("candidate_xy_m") and result.get("solver_executed"):
            candidate=Path()
            candidate.header.frame_id="map"
            # Deliberately current ROS display stamp; original historical
            # scan stamp remains in JSON and is NOT treated as current TF.
            candidate.header.stamp=self.get_clock().now().to_msg()
            z=float(result.get("model_input_pose_map_xyz_m",[0,0,.9])[2])
            for x,y in result["candidate_xy_m"]:
                pose=PoseStamped()
                pose.header=candidate.header
                pose.pose.position.x=float(x)
                pose.pose.position.y=float(y)
                pose.pose.position.z=z
                pose.pose.orientation.w=1.
                candidate.poses.append(pose)
            self.path_pub.publish(candidate)
            self.published_candidates+=1
        self._publish(dict(result),is_new=True)

    def _publish(self,result,is_new):
        self.published_reports+=1
        result.update(self.pipeline.evidence())
        result.update({
            "g3_report_seq":self.published_reports,
            "g3_candidate_paths_published":self.published_candidates,
            "reference_paths_received":self.received_paths,
            "pose_messages_received":self.received_poses,
            "track_reports_received":self.received_tracks,
            "is_new_solver_input":bool(is_new),
            "pipeline_mode":"HISTORICAL_OFFLINE_NO_MOTION_OUTPUT",
            "physical_robot_model_validated":False,
            "actuator_output":"DISABLED",
            "safety_authorization":"NEVER",
        })
        self.report_pub.publish(String(data=json.dumps(
            result,ensure_ascii=False,sort_keys=True)))

    def heartbeat(self):
        if self.last is None:
            report={
                "schema":"n01.offline.g3_mpc_cbf.v1",
                "status":"WAITING_FOR_RECORDED_BAG_AND_SCAN_PATH",
                "solver_executed":False,
                "candidate_xy_m":[],
                "moving_constraint_tracks":0,
            }
        else:
            report=dict(self.last)
            report["last_input_status"]=report.get("status")
            report["status"]="SNAPSHOT_NOT_NEW_OPTIMIZATION"
        self._publish(report,is_new=False)


def main(args=None):
    rclpy.init(args=args)
    node=OfflineG3Solver()
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
