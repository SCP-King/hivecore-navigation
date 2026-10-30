#!/usr/bin/env python3
"""Read-only live Hive2 + SLAM localization/TF evidence collection.

NO publishers, NO services, NO robot motion commands. Invoke while the
robot stays physically stationary, after vendor-authorized SLAM launch.
Checks what is observable, not the identities of individual TF publishers
inside each TFMessage (ROS2 message payload carries no publisher GID).
"""
import argparse
import json
import math
import time
from pathlib import Path


def classify(snapshot):
    edges=snapshot["mount_edges"]
    vals={round(e["height_m"],3) for e in edges}
    topics={e["topic"] for e in edges}
    status=snapshot.get("last_localization_status")
    status_ok=isinstance(status,dict) and all((
        status.get("outer")=="localization",
        status.get("inner")=="tracking",
        status.get("registration_tf_published") is True,
    ))
    cloud=snapshot.get("last_filtered_cloud")
    cloud_ok=(isinstance(cloud,dict) and cloud.get("frame")=="lidar_link"
              and cloud.get("stamp_ns",0)>0)
    mount_ok=(bool(edges) and vals=={.9} and
              not snapshot.get("tf_inspection_errors"))
    map_ok=snapshot["map_to_base_footprint_seen"]
    recent=bool(status and status.get("received_during_session"))
    status_time_check=status.get("age_at_receipt_s") if status else None
    fresh=(status_time_check is not None and
           0. <= status_time_check <= 2.0)
    if any(abs(v-.3)<.025 for v in vals) or len(vals)>1:
        result="BLOCKED_CONFLICTING_MOUNT_TF"
    elif not mount_ok:
        result="BLOCKED_MOUNT_TF_UNVERIFIED"
    elif not map_ok or not cloud_ok:
        result="BLOCKED_POSE_OR_SENSOR_NOT_OBSERVED"
    elif not status_ok or not recent or not fresh:
        result="BLOCKED_LOCALIZATION_STATUS_NOT_PROVEN_FRESH_TRACKING"
    elif snapshot.get("cmd_vel_publishers"):
        result="BLOCKED_UNEXPECTED_CMD_VEL_PUBLISHER"
    else:
        result="OBSERVATION_PRECHECK_PASSED_AUTHORSHIP_AND_MAP_ALIGNMENT_NOT_PROVEN"
    return {
        "result":result,
        "mount_one_value_0p90_observed":mount_ok,
        "mount_observed_on_topics":sorted(topics),
        "historical_0p30_seen_during_live_session":any(abs(v-.3)<.025 for v in vals),
        "map_to_base_footprint_seen":map_ok,
        "filtered_cloud_stamp_seen":bool(cloud_ok),
        "status_fields_match":bool(status_ok),
        "status_recent_ros_clock_checked":bool(fresh),
        "unique_mount_tf_publisher_attributed":False,
        "pcd_map_alignment_verified":False,
        "scan_time_tf_checked":False,
        "robot_motion_allowed":False,
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds",type=float,default=15.0)
    parser.add_argument("--output",type=Path,required=True)
    a=parser.parse_args()
    if not 5. <= a.seconds <= 40.:
        parser.error("duration must be 5..40 seconds")
    if a.output.exists():
        parser.error("refuses to overwrite an existing report")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import PointCloud2
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node=Node("n01_localization_read_only_evidence")
    state={
       "schema":"n01.live.localization_read_only_preflight.v1",
       "safety_authorization":"NEVER","actuator_output":"DISABLED",
       "node_creates_publishers":False,
       "mount_edges":[],"map_to_base_footprint_seen":False,
       "last_localization_status":None,"last_filtered_cloud":None,
       "tf_inspection_errors":[],"tf_publisher_endpoints":[],
       "cmd_vel_publishers":[],"status_subscription_error":None,
    }
    seen=set()
    def tf_cb(topic):
        def on_tf(msg):
            for t in msg.transforms:
                parent=t.header.frame_id.lstrip("/")
                child=t.child_frame_id.lstrip("/")
                if parent=="map" and child=="base_footprint":
                    state["map_to_base_footprint_seen"]=True
                if parent=="base_footprint" and child=="base_link":
                    z=float(t.transform.translation.z)
                    if not math.isfinite(z):
                        state["tf_inspection_errors"].append("NONFINITE_MOUNT")
                        continue
                    key=(topic,round(z,4))
                    if key not in seen:
                        seen.add(key)
                        state["mount_edges"].append({
                            "topic":topic,"height_m":round(z,5),
                            "first_seen_stamp_ns":int(t.header.stamp.sec)*10**9+
                                int(t.header.stamp.nanosec)})
        return on_tf
    static_qos=QoSProfile(depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs=[
        node.create_subscription(TFMessage,"/tf",tf_cb("/tf"),100),
        node.create_subscription(TFMessage,"/tf_static",tf_cb("/tf_static"),static_qos),
    ]
    def on_cloud(msg):
        state["last_filtered_cloud"]={
            "frame":msg.header.frame_id.lstrip("/"),
            "stamp_ns":msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
            "received_during_session":True}
    subs.append(node.create_subscription(PointCloud2,
        "/livox/lidar/geo_filtered",on_cloud,qos_profile_sensor_data))
    try:
        status_type=get_message("hivecore_slam3d/msg/LocalizationStatus")
        def on_status(msg):
            header=getattr(msg,"header",None)
            age=None
            if header is not None:
                st=int(header.stamp.sec)*10**9+int(header.stamp.nanosec)
                now=int(node.get_clock().now().nanoseconds)
                if st>0:
                    age=(now-st)/1e9
            state["last_localization_status"]={
                "outer":getattr(msg,"control_signal_outer_role",None),
                "inner":getattr(msg,"control_signal_session_mode",None),
                "registration_tf_published":getattr(
                    msg,"registration_output_tf_published",None),
                "localization_mode":getattr(
                    msg,"registration_output_localization_mode",None),
                "pose_source":getattr(msg,"registration_output_pose_source",None),
                "age_at_receipt_s":age,"received_during_session":True}
        subs.append(node.create_subscription(status_type,
            "/localization_status",on_status,qos_profile_sensor_data))
    except Exception as exc:
        state["status_subscription_error"]=repr(exc)
    try:
        end=time.monotonic()+a.seconds
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.2)
        for topic in ("/tf","/tf_static"):
            endpoints=node.get_publishers_info_by_topic(topic)
            state["tf_publisher_endpoints"].append({
                "topic":topic,
                "publishers":[{"node_name":p.node_name,
                               "node_namespace":p.node_namespace,
                               "endpoint_gid_hex":bytes(p.endpoint_gid).hex()}
                              for p in endpoints]})
        state["cmd_vel_publishers"]=[
            {"node_name":p.node_name,"node_namespace":p.node_namespace}
            for p in node.get_publishers_info_by_topic("/cmd_vel")]
        state["assessment"]=classify(state)
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(state,indent=2,ensure_ascii=False)+"\n",
                            encoding="utf-8")
        print("Read-only report:",a.output)
        print("Assessment:",state["assessment"]["result"])
        print("No motion command or TF publication was created.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
