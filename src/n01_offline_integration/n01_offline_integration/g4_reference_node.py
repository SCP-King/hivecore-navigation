"""Offline SCAN reference publisher; never generates motor or motion commands.

Publishes ONLY /initial_path plus research status, with strict historical Bag
isolation and observed starting-pose agreement. The reference is a G4.5
research artifact, NOT a clearance-approved navigation plan.
"""
import json
import math
import os
from pathlib import Path as FilePath

if (os.environ.get("ROS_DOMAIN_ID")!="88" or
    os.environ.get("ROS_LOCALHOST_ONLY")!="1" or
    os.environ.get("N01_HISTORICAL_BAG_ONLY")!="1" or
    os.environ.get("N01_G4_RESEARCH_ENABLE")!="1"):
    raise RuntimeError("G4.6 publisher requires explicit isolated historical RESEARCH mode")

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy)

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

from .scan_reference_artifact import SCHEMA


def load_reference(path):
    ref=json.loads(FilePath(path).read_text(encoding="utf-8"))
    if (ref.get("schema")!=SCHEMA or
        ref.get("status")!="UNVERIFIED_RESEARCH_REFERENCE_FOR_OFFLINE_SCAN_ONLY" or
        ref.get("frame_id")!="map" or
        ref.get("physical_execution_forbidden") is not True or
        ref.get("actuator_output")!="DISABLED" or
        ref.get("safety_authorization")!="NEVER" or
        ref.get("reference_is_not_planned_scan_trajectory") is not True):
        raise ValueError("reference provenance/physical-control safety contract invalid")
    pts=ref.get("path_xyz_m")
    if not isinstance(pts,list) or not 2<=len(pts)<=200:
        raise ValueError("G4 reference requires 2..200 path samples")
    for p in pts:
        if (not isinstance(p,list) or len(p)!=3 or not
                all(type(v) in (int,float) and math.isfinite(v) for v in p)):
            raise ValueError("nonfinite/malformed research waypoint")
    return ref


class G4ResearchReferencePublisher(Node):
    def __init__(self):
        super().__init__("n01_g4_research_reference")
        self.declare_parameter("reference_file","")
        self.declare_parameter("max_start_offset_m",.75)
        self.declare_parameter("scan_body_height_m",.4)
        self.declare_parameter("max_start_vertical_offset_m",.5)
        self.declare_parameter("require_static_overlay",False)
        source=self.get_parameter("reference_file").value
        distance=float(self.get_parameter("max_start_offset_m").value)
        body_height=float(self.get_parameter("scan_body_height_m").value)
        z_limit=float(self.get_parameter("max_start_vertical_offset_m").value)
        if (not source or not .1<=distance<=1.0 or
                not .1<=body_height<=1.5 or not .1<=z_limit<=1.0):
            raise ValueError("explicit G4.5 reference_file and bounded start offset required")
        self.ref=load_reference(source)
        self.limit=distance
        self.body_height=body_height
        self.z_limit=z_limit
        self.require_static_overlay=bool(
            self.get_parameter("require_static_overlay").value)
        self.static_ready=not self.require_static_overlay
        self.wait_count=0
        self.released=False
        self.blocked=False
        self.latest_body=None
        self.pub=self.create_publisher(Path,"/initial_path",10)
        self.status_pub=self.create_publisher(String,"/n01/offline/g4_reference_status",10)
        self.create_subscription(Odometry,"/body_pose",self.on_body,qos_profile_sensor_data)
        if self.require_static_overlay:
            latched=QoSProfile(depth=1,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(
                Bool,"/grid_map/static_overlay_ready",self.on_static_ready,latched)
        self.create_timer(.5,self.maybe_publish)
        self.get_logger().warning(
            "G4.6 RESEARCH reference only: no static-map SCAN fusion, "
            "no verified floor/clearance, no robot control")

    def on_static_ready(self,msg):
        self.static_ready=bool(msg.data)

    def on_body(self,msg):
        if msg.header.frame_id.lstrip("/")!="map":
            return
        pos=msg.pose.pose.position
        if all(math.isfinite(x) for x in (pos.x,pos.y,pos.z)):
            self.latest_body=(pos.x,pos.y,pos.z)

    def status(self,name,**kwargs):
        data={
            "schema":"n01.offline.g4_reference_status.v1",
            "status":name,
            "research_only":True,
            "scan_trajectory_not_yet_verified":True,
            "static_full_pcd_integrated_into_scan_grid":(
                self.require_static_overlay and self.static_ready),
            "static_overlay_required":self.require_static_overlay,
            "physical_execution_forbidden":True,
            "actuator_output":"DISABLED",
            "safety_authorization":"NEVER",
        }
        data.update(kwargs)
        self.status_pub.publish(String(data=json.dumps(data,sort_keys=True)))

    def maybe_publish(self):
        if self.released or self.blocked:
            return
        # Read-only graph guard, as in the historical Bag pipeline.
        try:
            if self.get_publishers_info_by_topic("/cmd_vel"):
                self.blocked=True
                self.status("BLOCKED_UNEXPECTED_CMD_VEL_PUBLISHER")
                self.get_logger().error("Unanticipated /cmd_vel publisher - refuse reference")
                return
        except Exception as exc:
            self.blocked=True
            self.status("BLOCKED_CMD_VEL_GRAPH_QUERY_FAILED",reason=str(exc))
            return
        if self.latest_body is None:
            return
        if not self.static_ready:
            self.wait_count+=1
            if self.wait_count % 8 == 1:
                self.status("WAITING_NATIVE_SCAN_STATIC_PCD_OVERLAY")
            return
        if self.pub.get_subscription_count()==0:
            return
        pts=self.ref["path_xyz_m"]
        gap=math.hypot(self.latest_body[0]-pts[0][0],
                       self.latest_body[1]-pts[0][1])
        z_gap=abs(self.latest_body[2]-(pts[0][2]+self.body_height))
        if gap>self.limit or z_gap>self.z_limit:
            # Never create an unobserved XY shortcut or large vertical jump.
            self.wait_count+=1
            if self.wait_count % 8 == 1:
                self.status("WAITING_MATCHING_BAG_START_POSE",
                            horizontal_error_m=round(gap,3),
                            vertical_error_m=round(z_gap,3),
                            horizontal_limit_m=self.limit,
                            vertical_limit_m=self.z_limit)
            return
        result=Path()
        result.header.frame_id="map"
        result.header.stamp=self.get_clock().now().to_msg()
        for x,y,z in pts:
            point=PoseStamped()
            point.header=result.header
            point.pose.position.x=x
            point.pose.position.y=y
            # SCAN navi_mode=3 adds its body_height to reference ground z.
            point.pose.position.z=z
            point.pose.orientation.w=1.
            result.poses.append(point)
        self.pub.publish(result)
        self.released=True
        self.status("UNVERIFIED_REFERENCE_SENT_TO_OFFLINE_SCAN",
                    start_offset_m=round(gap,3),
                    start_vertical_offset_m=round(z_gap,3),
                    published_waypoints=len(result.poses),
                    native_scan_static_overlay_ready=self.static_ready)
        self.get_logger().warning(
            "UNVERIFIED RESEARCH /initial_path sent: %d points, start offset %.3fm"
            % (len(result.poses),gap))


def main(args=None):
    rclpy.init(args=args)
    node=G4ResearchReferencePublisher()
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
