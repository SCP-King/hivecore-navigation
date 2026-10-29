"""Offline map-frame obstacle hypothesis detector and short-horizon predictor.

Consumes only /cloud and /body_pose, outputs JSON diagnostics (no controller).
Map-frame tracking is essential: sensor-frame apparent motion is not evidence
of a moving obstacle. Hypotheses are not safety-rated obstacle identification.
"""
import json
import math
import os
import struct
import time

if (os.environ.get("ROS_DOMAIN_ID")!="88" or
        os.environ.get("ROS_LOCALHOST_ONLY")!="1"):
    raise RuntimeError("Dynamic tracker strictly offline in isolated domain")

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from .dynamic_tracking_core import Tracker, clusters_from_points
from .dynamic_background import StaticBackgroundFilter


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def cloud_xyz_sample(msg, sample_limit):
    if msg.header.frame_id.lstrip("/")!="map":
        raise ValueError("input_pointcloud_not_in_map")
    if not (msg.height>=1 and msg.width>=1 and msg.point_step>=12
            and msg.row_step>=msg.width*msg.point_step
            and len(msg.data)>=msg.row_step*msg.height):
        raise ValueError("malformed_pointcloud_layout")
    fields={}
    for item in msg.fields:
        if item.name in ("x","y","z"):
            if (item.datatype!=PointField.FLOAT32 or item.count!=1
                    or item.offset<0 or item.offset+4>msg.point_step
                    or item.name in fields):
                raise ValueError("xyz_fields_not_float32")
            fields[item.name]=item.offset
    if set(fields)!={"x","y","z"}:
        raise ValueError("missing_xyz")
    count=msg.height*msg.width
    step=max(1,math.ceil(count/sample_limit))
    fmt=(">" if msg.is_bigendian else "<")+"f"
    unpack=struct.Struct(fmt)
    data=memoryview(msg.data)
    for i in range(0,count,step):
        off=(i//msg.width)*msg.row_step+(i%msg.width)*msg.point_step
        yield (unpack.unpack_from(data,off+fields["x"])[0],
               unpack.unpack_from(data,off+fields["y"])[0],
               unpack.unpack_from(data,off+fields["z"])[0])


class OfflineDynamicTracker(Node):
    def __init__(self):
        super().__init__("n01_offline_dynamic_tracker")
        def p(name,value):
            self.declare_parameter(name,value)
            return self.get_parameter(name).value
        self.voxel=float(p("voxel_size_m",.35))
        self.range=float(p("roi_range_m",8.))
        self.limit=int(p("max_sampled_points",16000))
        self.min_cells=int(p("min_cluster_cells",2))
        self.max_cells=int(p("max_cluster_cells",90))
        self.min_points=int(p("min_cluster_sampled_points",6))
        self.max_pose_delta_ns=int(float(p("max_pose_stamp_delta_s",.6))*1e9)
        if not(100<=self.limit<=100000 and .1<=self.voxel<=1.
               and 2<=self.max_cells<=10000 and 1<=self.min_cells<self.max_cells
               and 2<=self.min_points<=200 and
               0<self.max_pose_delta_ns<=2e9):
            raise ValueError("invalid dynamic tracker limits")
        self.tracker=Tracker()
        self.background=StaticBackgroundFilter(
            max_extent_m=float(p("max_dynamic_extent_m",1.8)))
        self.last_filter_stats={}
        self.pose=None
        self.pose_stamp=None
        self.last_scan_stamp=None
        self.total_clouds=0
        self.processed=0
        self.no_pose=0
        self.invalid=0
        self.rejected_stamp=0
        self.max_hypotheses=0
        self.moving_frames=0
        self.started=time.monotonic()
        self.pub=self.create_publisher(String,"/n01/offline/dynamic_tracks",10)
        self.create_subscription(Odometry,"/body_pose",self.on_pose,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,"/cloud",self.on_cloud,qos_profile_sensor_data)
        self.create_timer(2.,self.heartbeat)
        self.get_logger().warning(
            "OFFLINE HEURISTIC TRACKING; dynamic hypotheses unverified; no actuator output")

    def on_pose(self,msg):
        if msg.header.frame_id.lstrip("/")!="map":
            return
        xyz=(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
        if not all(math.isfinite(v) for v in xyz):
            return
        t=stamp_ns(msg.header.stamp)
        if t<=0:
            return
        self.pose=xyz
        self.pose_stamp=t

    def report(self,payload):
        payload.update({
            "source":"OFFLINE_MAP_CLOUD",
            "input_clouds":self.total_clouds,
            "processed_clouds":self.processed,
            "missing_body_pose_frames":self.no_pose,
            "rejected_invalid_clouds":self.invalid,
            "rejected_pose_timestamp_frames":self.rejected_stamp,
            "max_moving_hypotheses_seen":self.max_hypotheses,
            "static_background_filter_enabled":True,
            "static_background_rejected_total":self.background.stats["background_rejected"],
            "oversized_clusters_rejected_total":self.background.stats["oversized_rejected"],
            "moving_hypothesis_frames":self.moving_frames,
            "readiness":"RESEARCH_ONLY_NOT_SAFETY_RATED",
            "actuator_output":"DISABLED"})
        out=String()
        out.data=json.dumps(payload,sort_keys=True,ensure_ascii=False)
        self.pub.publish(out)

    def on_cloud(self,msg):
        self.total_clouds+=1
        now=stamp_ns(msg.header.stamp)
        if now<=0 or self.pose is None or self.pose_stamp is None:
            self.no_pose+=1
            self.report({"status":"NO_MAP_POSE", "tracks":[],
                         "moving_hypotheses":0})
            return
        if abs(now-self.pose_stamp)>self.max_pose_delta_ns:
            self.rejected_stamp+=1
            self.report({"status":"POSE_CLOUD_STAMP_MISMATCH", "tracks":[],
                         "moving_hypotheses":0})
            return
        try:
            result=clusters_from_points(
                cloud_xyz_sample(msg,self.limit),self.pose,
                cell_size=self.voxel,radius=self.range,
                min_cells=self.min_cells,max_cells=self.max_cells,
                min_points=self.min_points,max_points=self.limit)
            selected,self.last_filter_stats=self.background.update(now,result)
            tracks=self.tracker.update(now,selected)
        except (ValueError,TypeError,OverflowError,struct.error) as exc:
            self.invalid+=1
            self.report({"status":"INVALID_FRAME:"+type(exc).__name__,
                         "tracks":[],"moving_hypotheses":0})
            return
        self.processed+=1
        moving=tracks["moving_hypotheses"]
        self.max_hypotheses=max(self.max_hypotheses,moving)
        if moving:self.moving_frames+=1
        tracks["status"]="PROCESSED"
        tracks["cluster_count"]=len(result)
        tracks["candidate_cluster_count"]=len(selected)
        tracks["background_filter"]=self.last_filter_stats
        tracks["hypotheses_are_ground_truth"]=False
        self.report(tracks)

    def heartbeat(self):
        # Heartbeat distinguishes a dead node from no moving obstacles.
        if self.total_clouds==0:
            self.report({"status":"WAITING_FOR_MAP_CLOUD",
                         "moving_hypotheses":0, "tracks":[]})


def main(args=None):
    rclpy.init(args=args)
    node=OfflineDynamicTracker()
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
