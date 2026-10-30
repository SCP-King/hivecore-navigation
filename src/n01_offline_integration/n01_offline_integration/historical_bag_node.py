"""White-listed, non-actuating historical rosbag2 -> SCAN functional replay.

Never publishes raw /tf, /tf_static, /odom, /cmd_vel or any control message.
A complete old bag may contain both z=.30 and z=.90 for the same TF edge:
this module records the conflict, discards both historical mount edges and
applies user-selected z=.90 ONLY inside its offline point transform.

Unlike the strict online bridge, this is an OPEN-LOOP HISTORICAL REPLAY.
No LocalizationStatus / tracking success or current motion state is asserted.
"""
import json
import math
import os
from pathlib import Path
import time

if (os.environ.get("ROS_DOMAIN_ID") != "88" or
        os.environ.get("ROS_LOCALHOST_ONLY") != "1" or
        os.environ.get("N01_HISTORICAL_BAG_ONLY") != "1"):
    raise RuntimeError("Historical Bag replay must be explicitly isolated")

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.serialization import deserialize_message
import rosbag2_py
import yaml

from .core import xyz_transform_bytes
from .historical_tf import (
    HistoricalMapTf, stamp_to_ns, mapped_mounts, rotate,
    SELECTED_MOUNT_HEIGHT_M, SELECTED_LIDAR_X_M,
)

BAG_CLOUD = "/livox/lidar/geo_filtered"
BAG_TF = ("/tf", "/tf_static")


def odom(stamp, child, pose):
    p,q=pose
    msg=Odometry()
    msg.header.stamp=stamp
    msg.header.frame_id="map"
    msg.child_frame_id=child
    msg.pose.pose.position.x=float(p[0])
    msg.pose.pose.position.y=float(p[1])
    msg.pose.pose.position.z=float(p[2])
    (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = tuple(float(v) for v in q)
    # Deliberate zero placeholder for SCAN stationary replay input.
    # NEVER use as a physical velocity estimate for mobile robot control.
    return msg


class RecordedBagReplay(Node):
    def __init__(self):
        super().__init__("n01_historical_bag_replay")
        def param(k,v):
            self.declare_parameter(k,v)
            return self.get_parameter(k).value
        source=str(param("bag_path","")).strip()
        root=Path(source).expanduser()
        if not source or not (root/"metadata.yaml").is_file():
            raise RuntimeError("Explicit bag_path directory with metadata.yaml required")
        self.bag_root=str(root)
        self.max_clouds=int(param("max_clouds",180))
        self.scan_rate_hz=float(param("scan_rate_hz",4.))
        self.max_points=int(param("max_points",300000))
        self.max_age_s=float(param("max_map_tf_age_s",1.0))
        self.goal_forward=float(param("goal_forward_m",1.5))
        self.mount_profile=str(param("historical_mount_profile","LEGACY_HIVE2_0P90"))
        if self.mount_profile not in ("LEGACY_HIVE2_0P90",
                                     "MAPPING_SLAM_DYNAMIC_0P30"):
            raise ValueError("unknown historical mount profile; NO automatic TF guessing")
        self.mount_height_m=(.30 if self.mount_profile=="MAPPING_SLAM_DYNAMIC_0P30"
                             else SELECTED_MOUNT_HEIGHT_M)
        self.dynamic_030_mount_seen=False
        if not (10<=self.max_clouds<=2000 and .5<=self.scan_rate_hz<=30
                and 100<=self.max_points<=1000000 and .05<=self.max_age_s<=5.
                and .3<=self.goal_forward<=3.):
            raise ValueError("invalid bounded Bag functional replay settings")
        self.tf_history=HistoricalMapTf(max_age_s=self.max_age_s,
                                        max_interval_s=2*self.max_age_s)
        self.stats={
            "mode":"HISTORICAL_BAG_OPEN_LOOP_NOT_LOCALIZATION_TRACKING",
            "bag_path":self.bag_root, "height_used_m":self.mount_height_m,
            "historical_mount_profile":self.mount_profile,
            "original_slam_tf_internal_query_verified":False,
            "original_pcd_alignment_verified_by_replay":False,
            "lidar_x_used_m":SELECTED_LIDAR_X_M,
            "raw_tf_republished":False,"raw_bag_topics_republished":False,
            "cmd_vel_created":False,"cmd_vel_publishers_seen":False,
            "localization_status_in_bag":"NOT_REQUIRED_IN_OPEN_LOOP_REPLAY",
            "strict_p0_localization_verified":False,
            "input_clouds":0,"output_clouds":0,"rejected_clouds":0,
            "skipped_no_map_tf":0,"skipped_wrong_frame":0,
            "skipped_malformed":0,"goal_messages":0,
            "tf_map_samples":0,"tf_height_values_recorded":[],
            "historical_tf_height_conflict":False,
            "finished":False,
        }
        self.blocked=False
        self.report_pub=self.create_publisher(String,"/n01/offline/bag_replay_status",10)
        self.cloud_pub=self.create_publisher(PointCloud2,"/cloud",qos_profile_sensor_data)
        self.body_pub=self.create_publisher(Odometry,"/body_pose",qos_profile_sensor_data)
        self.sensor_pub=self.create_publisher(Odometry,"/sensor_pose",qos_profile_sensor_data)
        self.goal_pub=self.create_publisher(PoseStamped,"/move_base_simple/goal",10)
        self.origin_pose=None
        self.goal_times=(12.,32.,52.)
        self.goal_seq=0
        self.started=time.monotonic()
        self.reader=None
        self.get_logger().warning(
            "HISTORICAL OPEN-LOOP BAG: no SLAM tracking claim, no raw TF, no /cmd_vel")
        self.prepare()
        # Fill map before giving goal, let downstream SCAN initialize first.
        self.create_timer(1./self.scan_rate_hz,self.step)
        self.create_timer(1.,self.publish_status)

    def new_reader(self, topics):
        meta=yaml.safe_load(
            (Path(self.bag_root)/"metadata.yaml").read_text(encoding="utf-8")) or {}
        baginfo=meta.get("rosbag2_bagfile_information",{})
        storage=baginfo.get("storage_identifier","sqlite3")
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=self.bag_root,storage_id=storage),
                    rosbag2_py.ConverterOptions(
                        input_serialization_format="cdr",
                        output_serialization_format="cdr"))
        all_types={t.name:t.type for t in reader.get_all_topics_and_types()}
        if all_types.get(BAG_CLOUD)!="sensor_msgs/msg/PointCloud2":
            raise RuntimeError("Bag missing filtered Livox PointCloud2")
        if all_types.get("/tf")!="tf2_msgs/msg/TFMessage":
            raise RuntimeError("Bag missing /tf TFMessage")
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
        return reader

    def prepare(self):
        source=self.new_reader(list(BAG_TF))
        heights=set()
        while source.has_next():
            topic,data,_=source.read_next()
            if topic not in BAG_TF:
                continue
            msg=deserialize_message(data,TFMessage)
            for tf in msg.transforms:
                parent=tf.header.frame_id.lstrip("/")
                child=tf.child_frame_id.lstrip("/")
                if parent=="base_footprint" and child=="base_link":
                    z=round(tf.transform.translation.z,5)
                    if math.isfinite(z):
                        heights.add(z)
                        if topic=="/tf" and abs(z-.30)<=1e-4:
                            self.dynamic_030_mount_seen=True
                elif parent=="map" and child=="base_footprint":
                    t=tf.transform.translation
                    q=tf.transform.rotation
                    self.tf_history.add(
                        stamp_to_ns(tf.header.stamp),
                        (t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
        self.tf_history.finish()
        self.stats["tf_map_samples"]=len(self.tf_history.times)
        self.stats["tf_height_values_recorded"]=sorted(heights)
        self.stats["historical_tf_height_conflict"]=len(heights)>1
        self.stats["recorded_dynamic_030_mount_seen"]=self.dynamic_030_mount_seen
        if (self.mount_profile=="MAPPING_SLAM_DYNAMIC_0P30"
                and not self.dynamic_030_mount_seen):
            raise RuntimeError("MAPPING_PROFILE_REQUIRES_ORIGINAL_DYNAMIC_0P30_TF")
        if not self.tf_history.times:
            raise RuntimeError("No historical map->base_footprint TF: cannot derive map pointcloud")
        self.reader=self.new_reader([BAG_CLOUD])
        self.get_logger().warning(
            "Recorded map transforms=%d; base height values=%s; old TF is NOT republished"
            % (len(self.tf_history.times),sorted(heights)))

    def should_block(self):
        if self.blocked:
            return True
        try:
            endpoints=self.get_publishers_info_by_topic("/cmd_vel")
            if endpoints:
                self.stats["cmd_vel_publishers_seen"]=True
                self.blocked=True
                self.get_logger().error("Unexpected /cmd_vel publisher - stop Bag output")
        except Exception as exc:
            self.blocked=True
            self.get_logger().error("Cannot inspect command publishers: %s" % exc)
        return self.blocked

    def step(self):
        if self.should_block() or self.stats["finished"]:
            return
        if self.stats["input_clouds"]>=self.max_clouds:
            self.stats["finished"]=True
            self.get_logger().info("Historical Bag replay limit reached")
            return
        if not self.reader.has_next():
            self.stats["finished"]=True
            self.get_logger().info("Historical Bag reached EOF")
            return
        topic,data,_=self.reader.read_next()
        if topic!=BAG_CLOUD:  # defense in depth
            return
        self.stats["input_clouds"]+=1
        msg=deserialize_message(data,PointCloud2)
        if msg.header.frame_id.lstrip("/")!="lidar_link":
            self.stats["skipped_wrong_frame"]+=1
            self.stats["rejected_clouds"]+=1
            return
        pose=self.tf_history.lookup(stamp_to_ns(msg.header.stamp))
        if pose is None:
            self.stats["skipped_no_map_tf"]+=1
            self.stats["rejected_clouds"]+=1
            return
        body,lidar=mapped_mounts(pose,mount_height_m=self.mount_height_m)
        try:
            xyz=xyz_transform_bytes(
                msg.data,msg.width,msg.height,msg.point_step,msg.row_step,
                [(f.name,f.offset,f.datatype,f.count) for f in msg.fields],
                msg.is_bigendian,lidar[0],lidar[1],self.max_points)
        except (TypeError,OverflowError,ValueError) as exc:
            self.stats["skipped_malformed"]+=1
            self.stats["rejected_clouds"]+=1
            self.get_logger().warning("Cloud skipped: %s" % exc)
            return
        world=PointCloud2()
        world.header.stamp=msg.header.stamp
        world.header.frame_id="map"
        world.height,world.width=msg.height,msg.width
        world.fields=msg.fields
        world.is_bigendian=msg.is_bigendian
        world.point_step,world.row_step=msg.point_step,msg.row_step
        world.data,world.is_dense=xyz,msg.is_dense
        # Only safe output interface; published on isolated domain 88.
        self.sensor_pub.publish(odom(msg.header.stamp,"lidar_link",lidar))
        self.body_pub.publish(odom(msg.header.stamp,"base_link",body))
        self.cloud_pub.publish(world)
        self.stats["output_clouds"]+=1
        if self.origin_pose is None:
            self.origin_pose=body
        elapsed=time.monotonic()-self.started
        if (self.stats["output_clouds"]>=10 and
            self.goal_seq<len(self.goal_times) and
            elapsed>=self.goal_times[self.goal_seq]):
            self.send_goal(msg.header.stamp)

    def send_goal(self,stamp):
        if self.should_block() or self.origin_pose is None:
            return
        p,q=self.origin_pose
        offset=rotate(q,(self.goal_forward,0.,0.))
        goal=PoseStamped()
        goal.header.stamp=stamp
        goal.header.frame_id="map"
        goal.pose.position.x=float(p[0]+offset[0])
        goal.pose.position.y=float(p[1]+offset[1])
        goal.pose.position.z=float(p[2])
        goal.pose.orientation.x=float(q[0])
        goal.pose.orientation.y=float(q[1])
        goal.pose.orientation.z=float(q[2])
        goal.pose.orientation.w=float(q[3])
        self.goal_pub.publish(goal)
        self.goal_seq+=1
        self.stats["goal_messages"]+=1
        self.get_logger().info("REPLAY-ONLY goal #%d relative to first recorded body pose" % self.goal_seq)

    def publish_status(self):
        self.should_block()
        msg=String()
        msg.data=json.dumps(self.stats,sort_keys=True,ensure_ascii=False)
        self.report_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node=RecordedBagReplay()
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
