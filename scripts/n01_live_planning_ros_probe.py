#!/usr/bin/env python3
"""Read-only ROS 2 live data probe for the N-01 planning readiness audit.

Subscribes only. It never publishes goals, commands, parameters, services or actions.
"""
import argparse
import bisect
import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)
from rosidl_runtime_py.utilities import get_message


SENSOR_TOPICS = {
    "/livox/lidar/geo_filtered": "sensor_msgs/msg/PointCloud2",
    "/cloud": "sensor_msgs/msg/PointCloud2",
    "/robot_pose": "nav_msgs/msg/Odometry",
    "/odom": "nav_msgs/msg/Odometry",
    "/sensor_pose": "nav_msgs/msg/Odometry",
    "/body_pose": "nav_msgs/msg/Odometry",
    "/n01/live_navigation/scan_body_pose": "nav_msgs/msg/Odometry",
    "/imu/data": "sensor_msgs/msg/Imu",
    "/joint_states": "sensor_msgs/msg/JointState",
}
RELIABLE_TOPICS = {
    "/planning/bspline": "scan_planner_msgs/msg/Bspline",
    "/n01/live_navigation/readiness": "std_msgs/msg/String",
    "/n01/live_navigation/system_health": "std_msgs/msg/String",
    "/n01/live_navigation/goal_gate_status": "std_msgs/msg/String",
    "/n01/live_navigation/dynamic_obstacle_status": "std_msgs/msg/String",
    "/n01/live_navigation/hive2_feasibility": "std_msgs/msg/String",
    "/n01/live_navigation/safety_status": "std_msgs/msg/String",
    "/n01/live_navigation/operator_hold": "std_msgs/msg/Bool",
    "/mapping_status": "hivecore_slam3d/msg/MappingStatus",
    "/localization_status": "hivecore_slam3d/msg/LocalizationStatus",
    "/n01/live_navigation/navigation_ready": "std_msgs/msg/Bool",
    "/planning/go2_execution_frozen": "std_msgs/msg/Bool",
    "/mapping_processing_timings": "geometry_msgs/msg/Vector3Stamped",
    "/grid_map/static_occupancy": "sensor_msgs/msg/PointCloud2",
    "/grid_map/static_overlay_ready": "std_msgs/msg/Bool",
    "/grid_map/static_overlay_status": "std_msgs/msg/String",
}


def stamp_ns(stamp):
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def primitive_fields(msg, limit=32):
    result = {}
    try:
        names = msg.get_fields_and_field_types()
    except Exception:
        return result
    for name in names:
        if name == "header":
            continue
        try:
            value = getattr(msg, name)
        except Exception:
            continue
        if isinstance(value, (bool, int, float, str)):
            if not isinstance(value, float) or math.isfinite(value):
                result[name] = value
        elif isinstance(value, (list, tuple)) and len(value) <= limit:
            if all(isinstance(v, (bool, int, float, str)) for v in value):
                result[name] = [
                    v if not isinstance(v, float) or math.isfinite(v) else None
                    for v in value
                ]
    return result


def pointcloud_summary(msg):
    fields = {
        f.name: f for f in msg.fields
    }
    summary = {
        "frame_id": msg.header.frame_id,
        "stamp_ns": stamp_ns(msg.header.stamp),
        "width": int(msg.width),
        "height": int(msg.height),
        "point_count": int(msg.width) * int(msg.height),
        "point_step": int(msg.point_step),
        "row_step": int(msg.row_step),
        "is_dense": bool(msg.is_dense),
        "fields": [
            {
                "name": f.name,
                "offset": int(f.offset),
                "datatype": int(f.datatype),
                "count": int(f.count),
            }
            for f in msg.fields
        ],
    }
    if np is None:
        summary["xyz_sample_error"] = "numpy_unavailable"
        return summary
    required = ("x", "y", "z")
    if any(name not in fields for name in required):
        summary["xyz_sample_error"] = "x_y_z_fields_missing"
        return summary
    selected = [fields[name] for name in required]
    if any(int(f.count) != 1 or int(f.datatype) not in (7, 8) for f in selected):
        summary["xyz_sample_error"] = "x_y_z_must_be_float32_or_float64"
        return summary
    point_step = int(msg.point_step)
    row_step = int(msg.row_step) or point_step * int(msg.width)
    height = int(msg.height)
    width = int(msg.width)
    if point_step <= 0 or width <= 0 or height <= 0:
        summary["xyz_sample_error"] = "empty_or_invalid_cloud_dimensions"
        return summary
    formats = []
    for field in selected:
        kind = "f4" if int(field.datatype) == 7 else "f8"
        endian = ">" if msg.is_bigendian else "<"
        formats.append(np.dtype(endian + kind))
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": formats,
        "offsets": [int(f.offset) for f in selected],
        "itemsize": point_step,
    })
    try:
        array = np.ndarray(
            shape=(height, width),
            dtype=dtype,
            buffer=memoryview(msg.data),
            strides=(row_step, point_step),
        ).reshape(-1)
        count = int(array.size)
        stride = max(1, count // 5000)
        sample = array[::stride]
        xyz = np.column_stack((sample["x"], sample["y"], sample["z"]))
        finite = np.isfinite(xyz).all(axis=1)
        finite_xyz = xyz[finite]
        summary["xyz_sample_stride"] = int(stride)
        summary["xyz_sample_count"] = int(len(xyz))
        summary["xyz_sample_finite_count"] = int(len(finite_xyz))
        if len(finite_xyz):
            summary["xyz_sample_min"] = finite_xyz.min(axis=0).astype(float).tolist()
            summary["xyz_sample_max"] = finite_xyz.max(axis=0).astype(float).tolist()
    except Exception as exc:
        summary["xyz_sample_error"] = str(exc)
    return summary


class PlanningReadinessProbe(Node):
    def __init__(self):
        super().__init__("n01_planning_readiness_probe")
        self.started = time.monotonic()
        self.stats = {}
        self.audit_subscriptions = []
        self.topic_types = {}
        self.stamp_caches = {}
        self._create_subscriptions(SENSOR_TOPICS, sensor=True)
        self._create_subscriptions(RELIABLE_TOPICS, sensor=False)

    def _create_subscriptions(self, mapping, sensor):
        for topic, type_name in mapping.items():
            self.topic_types[topic] = type_name
            self.stats[topic] = {
                "type": type_name,
                "count": 0,
                "first_stamp_ns": None,
                "last_stamp_ns": None,
                "stamp_samples_ns": deque(maxlen=400),
                "last_detail": None,
                "subscription_error": None,
            }
            try:
                msg_type = get_message(type_name)
                if topic in (
                    "/grid_map/static_occupancy",
                    "/grid_map/static_overlay_ready",
                    "/grid_map/static_overlay_status",
                ):
                    qos = QoSProfile(
                        depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    )
                elif sensor:
                    qos = qos_profile_sensor_data
                else:
                    qos = 10
                sub = self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, name=topic: self._on_message(name, msg),
                    qos,
                )
                self.audit_subscriptions.append(sub)
            except Exception as exc:
                self.stats[topic]["subscription_error"] = str(exc)

    def _on_message(self, topic, msg):
        stat = self.stats[topic]
        stat["count"] += 1
        now_mono = time.monotonic()
        detail = {"received_monotonic_s": now_mono}
        stamp = None
        if hasattr(msg, "header"):
            header = msg.header
            stamp = stamp_ns(header.stamp)
            detail["header"] = {
                "stamp_ns": stamp,
                "frame_id": str(header.frame_id),
            }
        elif hasattr(msg, "start_time"):
            stamp = stamp_ns(msg.start_time)
        if stamp is not None:
            if stat["first_stamp_ns"] is None:
                stat["first_stamp_ns"] = stamp
            stat["last_stamp_ns"] = stamp
            stat["stamp_samples_ns"].append(stamp)
        if topic in ("/livox/lidar/geo_filtered", "/cloud", "/grid_map/static_occupancy"):
            detail["pointcloud"] = pointcloud_summary(msg)
        elif topic in (
            "/robot_pose", "/odom", "/sensor_pose", "/body_pose",
            "/n01/live_navigation/scan_body_pose",
        ):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            v = msg.twist.twist.linear
            q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            detail["odometry"] = {
                "frame_id": str(msg.header.frame_id),
                "child_frame_id": str(msg.child_frame_id),
                "position_xyz": [float(p.x), float(p.y), float(p.z)],
                "orientation_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
                "orientation_norm": float(q_norm),
                "linear_velocity_xyz": [float(v.x), float(v.y), float(v.z)],
            }
        elif topic == "/imu/data":
            a = msg.linear_acceleration
            w = msg.angular_velocity
            detail["imu"] = {
                "frame_id": str(msg.header.frame_id),
                "linear_acceleration_xyz": [float(a.x), float(a.y), float(a.z)],
                "angular_velocity_xyz": [float(w.x), float(w.y), float(w.z)],
            }
        elif topic == "/joint_states":
            required = (
                "fl_leg_pitch1", "fr_leg_pitch1", "rl_leg_pitch1", "rr_leg_pitch1",
                "fl_leg_pitch2", "fr_leg_pitch2", "rl_leg_pitch2", "rr_leg_pitch2",
            )
            names = [str(x) for x in msg.name]
            positions = {
                str(name): float(value)
                for name, value in zip(msg.name, msg.position)
                if math.isfinite(float(value))
            }
            detail["joint_states"] = {
                "joint_count": len(names),
                "names": names,
                "position_count": len(msg.position),
                "velocity_count": len(msg.velocity),
                "effort_count": len(msg.effort),
                "required_pitch_joints_present": {
                    name: name in positions for name in required
                },
                "required_pitch_positions_rad": {
                    name: positions[name] for name in required if name in positions
                },
            }
        elif topic == "/planning/bspline":
            detail["bspline"] = {
                "order": int(msg.order),
                "trajectory_id": int(msg.traj_id),
                "start_time_ns": stamp_ns(msg.start_time),
                "control_point_count": len(msg.pos_pts),
                "knot_count": len(msg.knots),
                "yaw_point_count": len(msg.yaw_pts),
                "yaw_dt": float(msg.yaw_dt),
                "knots_finite": all(math.isfinite(float(x)) for x in msg.knots),
                "control_points_finite": all(
                    math.isfinite(float(v))
                    for p in msg.pos_pts for v in (p.x, p.y, p.z)
                ),
            }
        elif topic in (
            "/n01/live_navigation/readiness",
            "/n01/live_navigation/system_health",
            "/n01/live_navigation/goal_gate_status",
            "/n01/live_navigation/dynamic_obstacle_status",
            "/n01/live_navigation/hive2_feasibility",
            "/n01/live_navigation/safety_status",
            "/grid_map/static_overlay_status",
        ):
            try:
                detail["json"] = json.loads(msg.data)
            except Exception:
                detail["raw"] = str(msg.data)[:12000]
        elif topic in (
            "/n01/live_navigation/operator_hold",
            "/n01/live_navigation/navigation_ready",
            "/planning/go2_execution_frozen",
            "/grid_map/static_overlay_ready",
        ):
            detail["value"] = bool(msg.data)
        elif topic == "/mapping_processing_timings":
            detail["vendor_timings_xyz_raw"] = [
                float(msg.vector.x), float(msg.vector.y), float(msg.vector.z)
            ]
        elif topic == "/localization_status":
            detail["localization_status_fields"] = primitive_fields(msg)
        elif topic == "/mapping_status":
            detail["mapping_status_fields"] = primitive_fields(msg)
        stat["last_detail"] = detail

    def result(self, duration_s):
        now_ros_ns = self.get_clock().now().nanoseconds
        now_system_ns = time.time_ns()
        topics = {}
        for topic, stat in self.stats.items():
            samples = list(stat["stamp_samples_ns"])
            topics[topic] = {
                "type": stat["type"],
                "count": stat["count"],
                "rate_hz_over_probe_window": round(stat["count"] / max(duration_s, 1e-6), 3),
                "first_stamp_ns": stat["first_stamp_ns"],
                "last_stamp_ns": stat["last_stamp_ns"],
                "last_stamp_age_s": (
                    round((now_ros_ns - stat["last_stamp_ns"]) / 1e9, 6)
                    if stat["last_stamp_ns"] is not None else None
                ),
                "stamp_sample_count": len(samples),
                "last_detail": stat["last_detail"],
                "subscription_error": stat["subscription_error"],
            }
        alignment = self._alignment_report()
        return {
            "schema": "n01.planning_readiness_ros_probe.v1",
            "probe_duration_s": round(duration_s, 3),
            "ros_clock_ns": now_ros_ns,
            "system_clock_ns": now_system_ns,
            "ros_minus_system_clock_s": round((now_ros_ns - now_system_ns) / 1e9, 6),
            "topics": topics,
            "timestamp_alignment": alignment,
            "map_geometry": self._map_geometry_report(),
            "safety": {
                "probe_is_subscribe_only": True,
                "goal_published": False,
                "actuator_command_published": False,
                "parameter_written": False,
                "service_or_action_invoked": False,
            },
        }

    def _stamps(self, topic):
        return sorted(
            int(x) for x in self.stats.get(topic, {}).get("stamp_samples_ns", [])
            if x is not None and int(x) > 0
        )

    @staticmethod
    def _nearest_report(target_stamps, source_stamps, tolerance_s=None):
        if not target_stamps or not source_stamps:
            return {
                "target_samples": len(target_stamps),
                "source_samples": len(source_stamps),
                "matched_samples": 0,
                "nearest_abs_delta_s_min": None,
                "nearest_abs_delta_s_max": None,
                "within_tolerance_count": None,
                "tolerance_s": tolerance_s,
            }
        deltas = []
        for target in target_stamps:
            index = bisect.bisect_left(source_stamps, target)
            candidates = []
            if index < len(source_stamps):
                candidates.append(source_stamps[index])
            if index > 0:
                candidates.append(source_stamps[index - 1])
            nearest = min(candidates, key=lambda value: abs(value - target))
            deltas.append(abs(nearest - target) / 1e9)
        return {
            "target_samples": len(target_stamps),
            "source_samples": len(source_stamps),
            "matched_samples": len(deltas),
            "nearest_abs_delta_s_min": round(min(deltas), 6),
            "nearest_abs_delta_s_max": round(max(deltas), 6),
            "within_tolerance_count": (
                sum(value <= tolerance_s for value in deltas)
                if tolerance_s is not None else None
            ),
            "tolerance_s": tolerance_s,
        }

    @staticmethod
    def _bracket_report(target_stamps, source_stamps, max_gap_s):
        if not target_stamps or not source_stamps:
            return {
                "target_samples": len(target_stamps),
                "source_samples": len(source_stamps),
                "bracketed_samples": 0,
                "max_bracket_gap_s": None,
                "within_max_gap_count": None,
                "max_gap_s": max_gap_s,
            }
        gaps = []
        for target in target_stamps:
            index = bisect.bisect_left(source_stamps, target)
            if index == 0 or index >= len(source_stamps):
                continue
            gaps.append((source_stamps[index] - source_stamps[index - 1]) / 1e9)
        return {
            "target_samples": len(target_stamps),
            "source_samples": len(source_stamps),
            "bracketed_samples": len(gaps),
            "max_bracket_gap_s": round(max(gaps), 6) if gaps else None,
            "within_max_gap_count": (
                sum(value <= max_gap_s for value in gaps) if gaps else 0
            ),
            "max_gap_s": max_gap_s,
        }

    @staticmethod
    def _exact_match_report(left, right):
        if not left or not right:
            return {
                "left_samples": len(left),
                "right_samples": len(right),
                "exact_common_stamp_count": 0,
                "left_covered_by_right_pct": None,
            }
        right_set = set(right)
        common = set(left) & right_set
        return {
            "left_samples": len(left),
            "right_samples": len(right),
            "exact_common_stamp_count": len(common),
            "left_covered_by_right_pct": round(100.0 * len(set(left) & right_set) / len(set(left)), 2),
        }

    def _map_geometry_report(self):
        body_detail = (
            self.stats.get("/body_pose", {}).get("last_detail") or {}
        )
        body_odom = body_detail.get("odometry") or {}
        body_position = body_odom.get("position_xyz")
        body_frame = body_odom.get("frame_id")
        cloud_detail = (
            self.stats.get("/cloud", {}).get("last_detail") or {}
        )
        cloud = cloud_detail.get("pointcloud") or {}
        static_detail = (
            self.stats.get("/grid_map/static_occupancy", {}).get("last_detail") or {}
        )
        static_cloud = static_detail.get("pointcloud") or {}
        overlay_detail = (
            self.stats.get("/grid_map/static_overlay_status", {}).get("last_detail") or {}
        )
        overlay = overlay_detail.get("json") or {}

        def vector(name):
            value = overlay.get(name)
            if isinstance(value, list) and len(value) == 3:
                try:
                    return [float(x) for x in value]
                except (TypeError, ValueError):
                    return None
            return None

        source_min = vector("source_min_xyz")
        source_max = vector("source_max_xyz")
        window_min = vector("window_min_xyz")
        window_max = vector("window_max_xyz")

        def inside(point, low, high):
            return bool(
                point is not None and low is not None and high is not None
                and len(point) == 3
                and all(low[i] <= float(point[i]) <= high[i] for i in range(3))
            )

        cloud_min = cloud.get("xyz_sample_min")
        cloud_max = cloud.get("xyz_sample_max")
        cloud_intersects_window = bool(
            cloud_min is not None and cloud_max is not None
            and window_min is not None and window_max is not None
            and all(cloud_max[i] >= window_min[i] and cloud_min[i] <= window_max[i]
                    for i in range(3))
        )
        source_frame = overlay.get("source_frame")
        cloud_frame = cloud.get("frame_id")
        return {
            "body_pose_frame": body_frame,
            "body_pose_position_xyz": body_position,
            "body_pose_frame_is_map": body_frame is not None and body_frame.lstrip("/") == "map",
            "static_source_frame": source_frame,
            "static_source_frame_matches_body_pose": (
                source_frame is not None and body_frame is not None
                and source_frame.lstrip("/") == body_frame.lstrip("/")
            ),
            "static_source_min_xyz": source_min,
            "static_source_max_xyz": source_max,
            "active_scan_window_min_xyz": window_min,
            "active_scan_window_max_xyz": window_max,
            "robot_inside_static_source_bounds": inside(body_position, source_min, source_max),
            "robot_inside_active_scan_window": inside(body_position, window_min, window_max),
            "static_source_window_intersects": overlay.get("window_intersects_source_bounds"),
            "static_overlay_ready": overlay.get("ready"),
            "static_overlay_reason": overlay.get("reason"),
            "static_cloud_frame": static_cloud.get("frame_id"),
            "static_cloud_point_count": static_cloud.get("point_count"),
            "static_cloud_sample_min_xyz": static_cloud.get("xyz_sample_min"),
            "static_cloud_sample_max_xyz": static_cloud.get("xyz_sample_max"),
            "static_cloud_frame_is_map": (
                static_cloud.get("frame_id") is not None
                and static_cloud.get("frame_id").lstrip("/") == "map"
            ),
            "cloud_frame": cloud_frame,
            "cloud_frame_is_map": cloud_frame is not None and cloud_frame.lstrip("/") == "map",
            "cloud_sample_min_xyz": cloud_min,
            "cloud_sample_max_xyz": cloud_max,
            "cloud_sample_intersects_active_window": cloud_intersects_window,
            "registration_proven_by_pcd": False,
        }

    def _alignment_report(self):
        lidar = self._stamps("/livox/lidar/geo_filtered")
        cloud = self._stamps("/cloud")
        pose = self._stamps("/robot_pose")
        odom = self._stamps("/odom")
        imu = self._stamps("/imu/data")
        sensor_pose = self._stamps("/sensor_pose")
        body_pose = self._stamps("/body_pose")
        scan_body_pose = self._stamps("/n01/live_navigation/scan_body_pose")
        return {
            "livox_scan_vs_global_robot_pose": self._nearest_report(lidar, pose, 1.25),
            "livox_scan_vs_imu": self._nearest_report(lidar, imu, 0.10),
            "livox_scan_vs_odometry_bracket": self._bracket_report(lidar, odom, 0.08),
            "cloud_vs_original_livox_stamp": self._exact_match_report(cloud, lidar),
            "cloud_vs_sensor_pose_stamp": self._exact_match_report(cloud, sensor_pose),
            "cloud_vs_scan_body_pose_stamp": self._exact_match_report(cloud, scan_body_pose),
            "body_pose_vs_odometry_stamp": self._exact_match_report(body_pose, odom),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    if not 3.0 <= args.duration <= 30.0:
        parser.error("--duration must be in [3, 30] seconds")
    rclpy.init()
    node = PlanningReadinessProbe()
    started = time.monotonic()
    try:
        deadline = started + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.monotonic() - started
        result = node.result(elapsed)
        rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
        if args.output:
            with open(args.output, "w", encoding="utf-8") as stream:
                stream.write(rendered)
                stream.write("\n")
        print("===== N-01 ROS live-data probe summary =====")
        print("duration_s:", result["probe_duration_s"])
        print("ros_minus_system_clock_s:", result["ros_minus_system_clock_s"])
        for topic, data in result["topics"].items():
            detail = data.get("last_detail") or {}
            frame = detail.get("header", {}).get("frame_id")
            count = data["count"]
            rate = data["rate_hz_over_probe_window"]
            stamp_age = data["last_stamp_age_s"]
            print(
                f"{topic}: count={count}, rate={rate}Hz, "
                f"frame={frame}, stamp_age_s={stamp_age}"
            )
        print("===== Timestamp alignment =====")
        print(json.dumps(result["timestamp_alignment"], ensure_ascii=False, indent=2))
        print("===== Full JSON =====")
        print(rendered)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
