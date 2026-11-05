#!/usr/bin/env python3
"""Read-only ROS 2 audit for N-01 live localization, map, planning and safety topics.

This tool subscribes only. It never publishes goals, velocity, wheel or actuator commands.
Run after sourcing ROS 2 Humble and the workspace install/setup.bash.
"""
import argparse
import json
import math
import time
from collections import Counter, defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)
from rosidl_runtime_py.utilities import get_message

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


EXPECTED_TYPES = {
    "/odom": "nav_msgs/msg/Odometry",
    "/robot_pose": "nav_msgs/msg/Odometry",
    "/body_pose": "nav_msgs/msg/Odometry",
    "/sensor_pose": "nav_msgs/msg/Odometry",
    "/n01/live_navigation/scan_body_pose": "nav_msgs/msg/Odometry",
    "/livox/lidar/geo_filtered": "sensor_msgs/msg/PointCloud2",
    "/cloud": "sensor_msgs/msg/PointCloud2",
    "/grid_map/static_occupancy": "sensor_msgs/msg/PointCloud2",
    "/localization_status": None,  # vendor message; resolve from the live graph
    "/n01/live_navigation/readiness": "std_msgs/msg/String",
    "/n01/live_navigation/system_health": "std_msgs/msg/String",
    "/n01/live_navigation/goal_gate_status": "std_msgs/msg/String",
    "/n01/live_navigation/hive2_feasibility": "std_msgs/msg/String",
    "/n01/live_navigation/dynamic_obstacle_status": "std_msgs/msg/String",
    "/n01/live_navigation/safety_status": "std_msgs/msg/String",
    "/grid_map/static_overlay_ready": "std_msgs/msg/Bool",
    "/n01/live_navigation/navigation_ready": "std_msgs/msg/Bool",
    "/joint_states": "sensor_msgs/msg/JointState",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}

SENSOR_TOPICS = {
    "/odom", "/robot_pose", "/body_pose", "/sensor_pose",
    "/n01/live_navigation/scan_body_pose",
    "/livox/lidar/geo_filtered", "/cloud", "/localization_status",
    "/joint_states",
}


def percentile(values, fraction):
    values = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not values:
        return None
    index = min(len(values) - 1, max(0, int(math.ceil(fraction * len(values)) - 1)))
    return values[index]


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def json_summary(topic, value):
    if not isinstance(value, dict):
        return {"value_type": type(value).__name__}
    if topic.endswith("/readiness"):
        keys = (
            "status", "localization_status", "motion_compensation_error",
            "robot_pose_stamp_age_s", "robot_pose_stamp_progress_age_s",
            "robot_pose_publisher_count", "robot_pose_multiple_publisher_endpoints",
            "robot_pose_publishers", "robot_pose_anchor_max_delta_s",
            "robot_pose_live_anchor_max_delta_s", "pose_duplicate",
            "pose_out_of_order", "drop_pose_duplicate_conflict",
            "pose_scan_before_delta_s", "pose_scan_anchor_available",
            "body_pose_live_fresh",
            "body_pose_live_stamp_age_s", "body_pose_live_anchor_delta_s",
            "body_pose_live_anchor_fresh", "body_pose_live_error",
            "latest_scan_lag_s", "cloud_output_age_s", "drop_tf_timeout",
            "drop_odom_timeout", "drop_pose_timeout", "drop_stale_after_wait",
        )
    elif topic.endswith("/system_health"):
        keys = (
            "schema", "status", "navigation_ready", "blockers",
            "static_overlay_ready", "body_pose_stamp_age_s",
            "body_pose_receive_age_s", "actuator_output", "safety_authorization",
        )
    elif topic.endswith("/goal_gate_status"):
        keys = ("accepted", "reason", "navigation_ready", "static_overlay_ready")
    elif topic.endswith("/hive2_feasibility"):
        keys = (
            "status", "candidate_review_status", "candidate_review_ready",
            "necessary_conditions_pass", "controller_model_verified",
            "wheel_contact_geometry_verified", "collision_envelope_verified",
        )
    elif topic.endswith("/dynamic_obstacle_status"):
        keys = (
            "status", "cluster_count", "unmatched_candidate_points",
            "visualization_only", "planner_input",
        )
    elif topic.endswith("/safety_status"):
        keys = (
            "status", "blockers", "operator_hold", "physical_acceptance_missing",
            "execution_authorized", "actuator_output", "safety_authorization",
        )
    else:
        keys = tuple(value.keys())[:16]
    return {key: value[key] for key in keys if key in value}


class LiveStackPreflight(Node):
    def __init__(self, duration):
        super().__init__("n01_live_stack_preflight")
        self.duration = duration
        self.started = time.monotonic()
        self.records = defaultdict(lambda: {
            "count": 0,
            "rx": [],
            "stamps": [],
            "stamp_ages": [],
            "frames": set(),
            "children": set(),
            "out_of_order": 0,
            "point_count": None,
            "point_fields": [],
            "latest_value": None,
            "latest_json": None,
            "joint_names": [],
            "tf_edges": Counter(),
        })
        self.topic_types = {}
        self.subscriptions = []
        self.subscribed_topics = set()
        self.topic_presence = {}
        self.topic_discovery_error = ""
        self.refresh_topic_subscriptions()
        # DDS discovery can complete after this observer starts. Re-scan the
        # graph so late transient-local topics are subscribed and reported.
        self.create_timer(0.5, self.refresh_topic_subscriptions)

    def refresh_topic_subscriptions(self):
        try:
            self.topic_types = dict(self.get_topic_names_and_types())
            self.topic_discovery_error = ""
        except Exception as exc:
            self.topic_discovery_error = str(exc)
            return

        for topic, expected_type in EXPECTED_TYPES.items():
            available = self.topic_types.get(topic, [])
            if not available:
                self.topic_presence[topic] = {
                    "present": False, "available_types": [], "expected_type": expected_type,
                }
                continue
            actual_type = expected_type or available[0]
            if actual_type not in available:
                self.topic_presence[topic] = {
                    "present": True, "available_types": available,
                    "expected_type": expected_type, "type_mismatch": True,
                }
                continue
            if topic not in self.subscribed_topics:
                try:
                    message_class = get_message(actual_type)
                    qos = self._qos_for(topic)
                    subscription = self.create_subscription(
                        message_class, topic, self._callback(topic), qos)
                    self.subscriptions.append(subscription)
                    self.subscribed_topics.add(topic)
                except Exception as exc:
                    self.topic_presence[topic] = {
                        "present": True, "available_types": available,
                        "expected_type": expected_type, "subscribe_error": str(exc),
                    }
                    continue
            self.topic_presence[topic] = {
                "present": True, "available_types": available,
                "expected_type": expected_type, "subscribed_type": actual_type,
            }

    @staticmethod
    def _qos_for(topic):
        if topic in ("/tf_static", "/grid_map/static_overlay_ready",
                     "/grid_map/static_occupancy"):
            return QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        if topic == "/tf":
            return QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        if topic in SENSOR_TOPICS:
            return qos_profile_sensor_data
        return 10

    def _callback(self, topic):
        def callback(msg):
            record = self.records[topic]
            now = time.monotonic()
            record["count"] += 1
            record["rx"].append(now)
            if len(record["rx"]) > 2000:
                del record["rx"][:-2000]

            header = getattr(msg, "header", None)
            if header is not None:
                current_stamp = stamp_ns(header.stamp)
                if current_stamp > 0:
                    if record["stamps"] and current_stamp < record["stamps"][-1]:
                        record["out_of_order"] += 1
                    record["stamps"].append(current_stamp)
                    if len(record["stamps"]) > 2000:
                        del record["stamps"][:-2000]
                    age = (self.get_clock().now().nanoseconds - current_stamp) / 1e9
                    record["stamp_ages"].append(age)
                    if len(record["stamp_ages"]) > 2000:
                        del record["stamp_ages"][:-2000]
                frame = str(getattr(header, "frame_id", "")).lstrip("/")
                if frame:
                    record["frames"].add(frame)

            child = str(getattr(msg, "child_frame_id", "")).lstrip("/")
            if child:
                record["children"].add(child)

            if isinstance(msg, PointCloud2):
                record["point_count"] = int(msg.width) * int(msg.height)
                record["point_fields"] = [str(field.name) for field in msg.fields]
            elif isinstance(msg, Bool):
                record["latest_value"] = bool(msg.data)
            elif isinstance(msg, String):
                try:
                    parsed = json.loads(msg.data)
                    record["latest_json"] = json_summary(topic, parsed)
                except (ValueError, TypeError):
                    record["latest_json"] = {"raw_prefix": str(msg.data)[:240]}
            elif isinstance(msg, JointState):
                record["joint_names"] = list(msg.name)
            elif isinstance(msg, TFMessage):
                for transform in msg.transforms:
                    parent = str(transform.header.frame_id).lstrip("/")
                    child_frame = str(transform.child_frame_id).lstrip("/")
                    if parent and child_frame:
                        record["tf_edges"][parent + " -> " + child_frame] += 1
        return callback

    @staticmethod
    def _endpoint_summary(info):
        try:
            gid = bytes(info.endpoint_gid).hex()
        except Exception:
            gid = str(getattr(info, "endpoint_gid", ""))
        return {
            "node": (str(info.node_namespace).rstrip("/") + "/" + str(info.node_name)),
            "type": str(info.topic_type),
            "gid": gid,
        }

    def _topic_summary(self, topic):
        record = self.records[topic]
        rx = record["rx"]
        stamps = record["stamps"]
        rx_gaps = [b - a for a, b in zip(rx, rx[1:]) if b >= a]
        stamp_gaps = [
            (b - a) / 1e9 for a, b in zip(stamps, stamps[1:]) if b > a
        ]
        rx_rate = (
            (len(rx) - 1) / (rx[-1] - rx[0])
            if len(rx) > 1 and rx[-1] > rx[0] else 0.0
        )
        now_ns = self.get_clock().now().nanoseconds
        now_mono = time.monotonic()
        latest_stamp_age = (
            (now_ns - stamps[-1]) / 1e9 if stamps else None
        )
        latest_rx_age = now_mono - rx[-1] if rx else None
        result = {
            "present": bool(self.topic_presence.get(topic, {}).get("present")),
            "type": self.topic_presence.get(topic, {}).get("subscribed_type"),
            "samples": record["count"],
            "receive_rate_hz": round(rx_rate, 3),
            "receive_gap_p95_s": round(percentile(rx_gaps, 0.95), 4)
                if rx_gaps else None,
            "receive_gap_max_s": round(max(rx_gaps), 4) if rx_gaps else None,
            "stamp_interval_p95_s": round(percentile(stamp_gaps, 0.95), 4)
                if stamp_gaps else None,
            "stamp_age_median_s": round(percentile(record["stamp_ages"], 0.50), 4)
                if record["stamp_ages"] else None,
            "stamp_age_p95_s": round(percentile(record["stamp_ages"], 0.95), 4)
                if record["stamp_ages"] else None,
            "latest_stamp_age_s": round(latest_stamp_age, 4)
                if latest_stamp_age is not None else None,
            "latest_receive_age_s": round(latest_rx_age, 4)
                if latest_rx_age is not None else None,
            "out_of_order_stamps": record["out_of_order"],
            "frames": sorted(record["frames"]),
            "child_frames": sorted(record["children"]),
        }
        if record["point_count"] is not None:
            result["latest_point_count"] = record["point_count"]
            result["point_fields"] = record["point_fields"]
        if record["latest_value"] is not None:
            result["latest_value"] = record["latest_value"]
        if record["latest_json"] is not None:
            result["latest_diagnostic"] = record["latest_json"]
        if record["joint_names"]:
            result["joint_name_count"] = len(record["joint_names"])
            result["joint_names"] = record["joint_names"]
        if record["tf_edges"]:
            result["observed_tf_edges"] = dict(record["tf_edges"].most_common(40))
        try:
            publishers = self.get_publishers_info_by_topic(topic)
            result["publishers"] = [
                self._endpoint_summary(item) for item in publishers
            ]
            result["publisher_count"] = len(publishers)
        except Exception as exc:
            result["publisher_query_error"] = str(exc)
        return result

    def report(self):
        self.refresh_topic_subscriptions()
        topics = {topic: self._topic_summary(topic) for topic in EXPECTED_TYPES}
        cloud_stamps = set(self.records["/cloud"]["stamps"])
        sensor_pose_stamps = set(self.records["/sensor_pose"]["stamps"])
        scan_body_stamps = set(self.records["/n01/live_navigation/scan_body_pose"]["stamps"])
        exact_pairs = {
            "cloud_sensor_pose_exact_matches": len(cloud_stamps & sensor_pose_stamps),
            "cloud_without_sensor_pose": len(cloud_stamps - sensor_pose_stamps),
            "cloud_scan_body_pose_exact_matches": len(cloud_stamps & scan_body_stamps),
            "cloud_without_scan_body_pose": len(cloud_stamps - scan_body_stamps),
        }

        node_pairs = self.get_node_names_and_namespaces()
        name_counts = Counter(
            (str(name), str(namespace).rstrip("/") or "/")
            for name, namespace in node_pairs
        )
        duplicate_names = {
            (namespace + "/" + name) if namespace != "/" else "/" + name: count
            for (name, namespace), count in name_counts.items() if count > 1
        }
        multiple_publisher_endpoints = {}
        same_node_multiple_publisher_endpoints = {}
        for topic, summary in topics.items():
            publishers = summary.get("publishers", [])
            if len(publishers) > 1:
                multiple_publisher_endpoints[topic] = {
                    "publisher_count": len(publishers),
                    "publishers": publishers,
                }
                endpoint_node_counts = Counter(
                    publisher.get("node", "UNKNOWN") for publisher in publishers)
                repeated_nodes = {
                    node: count for node, count in endpoint_node_counts.items()
                    if count > 1
                }
                if repeated_nodes:
                    same_node_multiple_publisher_endpoints[topic] = repeated_nodes
        tf_publishers = {}
        for topic in ("/tf", "/tf_static"):
            try:
                tf_publishers[topic] = [
                    self._endpoint_summary(item)
                    for item in self.get_publishers_info_by_topic(topic)
                ]
            except Exception as exc:
                tf_publishers[topic] = {"error": str(exc)}

        bridge_diag = topics["/n01/live_navigation/readiness"].get("latest_diagnostic", {})
        health_diag = topics["/n01/live_navigation/system_health"].get("latest_diagnostic", {})
        safety_diag = topics["/n01/live_navigation/safety_status"].get("latest_diagnostic", {})
        static_ready = topics["/grid_map/static_overlay_ready"].get("latest_value")
        navigation_ready = topics["/n01/live_navigation/navigation_ready"].get("latest_value")
        body_rate = topics["/body_pose"]["receive_rate_hz"]
        warnings = []
        if topics["/odom"]["receive_rate_hz"] < 20.0:
            warnings.append("odom_rate_below_20_hz")
        if topics["/robot_pose"]["receive_rate_hz"] < 1.0:
            warnings.append("robot_pose_rate_below_1_hz_or_missing")
        robot_pose_stamp_age = topics["/robot_pose"].get("latest_stamp_age_s")
        robot_pose_receive_age = topics["/robot_pose"].get("latest_receive_age_s")
        if robot_pose_stamp_age is not None and robot_pose_stamp_age > 0.5:
            warnings.append("robot_pose_stamp_lag_above_0_5s")
        if (
            robot_pose_stamp_age is not None and robot_pose_stamp_age > 0.5
            and robot_pose_receive_age is not None and robot_pose_receive_age <= 0.7
        ):
            warnings.append("robot_pose_stamp_stale_despite_recent_delivery")
        if robot_pose_stamp_age is not None and robot_pose_stamp_age > 1.5:
            warnings.append("robot_pose_stamp_age_exceeds_bridge_budget")
        for topic in multiple_publisher_endpoints:
            warnings.append("multiple_publisher_endpoints:" + topic)
        for topic in same_node_multiple_publisher_endpoints:
            warnings.append("multiple_publisher_endpoints_same_node:" + topic)
        if topics["/body_pose"]["samples"] < 2 or body_rate < 10.0:
            warnings.append("body_pose_rate_below_10_hz")
        if exact_pairs["cloud_sensor_pose_exact_matches"] == 0:
            warnings.append("no_exact_cloud_sensor_pose_pairs_observed")
        if static_ready is not True:
            warnings.append("static_overlay_not_ready_or_not_observed")
        if navigation_ready is not True:
            warnings.append("navigation_ready_false_or_not_observed")
        if duplicate_names:
            warnings.append("duplicate_ros_node_names_observed")
        if health_diag.get("blockers"):
            warnings.extend("health_blocker:" + str(item) for item in health_diag["blockers"])

        return {
            "schema": "n01.live_stack_preflight.v1",
            "duration_s": round(time.monotonic() - self.started, 3),
            "read_only": True,
            "actuator_output": "DISABLED",
            "execution_authorized": False,
            "safety_authorization": "NEVER",
            "overall": {
                "bridge_status": bridge_diag.get("status"),
                "navigation_ready": navigation_ready,
                "system_health_status": health_diag.get("status"),
                "system_health_blockers": health_diag.get("blockers", []),
                "static_overlay_ready": static_ready,
                "body_pose_receive_rate_hz": body_rate,
                "safety_status": safety_diag.get("status"),
                "multiple_publisher_endpoint_topics": sorted(multiple_publisher_endpoints),
            },
            "exact_stamp_pairs": exact_pairs,
            "warnings": warnings,
            "duplicate_node_names": duplicate_names,
            "multiple_publisher_endpoints": multiple_publisher_endpoints,
            "same_node_multiple_publisher_endpoints": same_node_multiple_publisher_endpoints,
            "topic_discovery_error": self.topic_discovery_error,
            "tf_publishers": tf_publishers,
            "topic_presence": self.topic_presence,
            "topics": topics,
        }


def main():
    parser = argparse.ArgumentParser(
        description="Read-only N-01 ROS 2 live stack preflight")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="sampling duration in seconds (default: 10)")
    args, _unknown = parser.parse_known_args()
    if not math.isfinite(args.duration) or not 3.0 <= args.duration <= 60.0:
        parser.error("--duration must be between 3 and 60 seconds")

    rclpy.init(args=None)
    node = LiveStackPreflight(args.duration)
    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(json.dumps(node.report(), indent=2, ensure_ascii=False, sort_keys=True))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
