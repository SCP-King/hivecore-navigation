"""Read-only runtime supervisor for localization, static map and SCAN readiness."""
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3Stamped
from scan_planner_msgs.msg import Bspline

from .core import stamped_data_is_fresh


class LiveRuntimeSupervisor(Node):
    def __init__(self):
        super().__init__("n01_live_runtime_supervisor")
        self.declare_parameter("max_body_pose_age_s", 0.25)
        self.declare_parameter("max_body_pose_rx_age_s", 0.35)
        self.declare_parameter("status_topic", "/n01/live_navigation/readiness")
        self.declare_parameter("static_ready_topic", "/grid_map/static_overlay_ready")
        self.declare_parameter("body_pose_topic", "/body_pose")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("timings_topic", "/mapping_processing_timings")
        self.max_body_age = float(self.get_parameter("max_body_pose_age_s").value)
        self.max_body_rx_age = float(self.get_parameter("max_body_pose_rx_age_s").value)
        if not (0.1 <= self.max_body_age <= 2.0 and 0.1 <= self.max_body_rx_age <= 3.0):
            raise ValueError("runtime supervisor age bounds invalid")

        self.bridge = {}
        self.static_ready = False
        self.body_stamp_ns = None
        self.body_rx_monotonic = None
        self.map_tf_stamp_ns = None
        self.tf_intervals = deque(maxlen=20)
        self.last_tf_stamp_ns = None
        self.timings = None
        self.timings_stamp_ns = None
        self.timings_rx_monotonic = None
        self.robot_feasibility = {}
        self.robot_feasibility_rx_monotonic = None
        self.dynamic_obstacle = {}
        self.dynamic_obstacle_rx_monotonic = None
        self.planner_rx_monotonic = None
        self.planner_traj_id = None
        self.last_health = {}

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            String, self.get_parameter("status_topic").value, self.on_bridge, 10)
        self.create_subscription(
            Bool, self.get_parameter("static_ready_topic").value, self.on_static, latched)
        self.create_subscription(
            Odometry, self.get_parameter("body_pose_topic").value,
            self.on_body_pose, qos_profile_sensor_data)
        self.create_subscription(
            TFMessage, self.get_parameter("tf_topic").value, self.on_tf, tf_qos)
        self.create_subscription(
            Vector3Stamped, self.get_parameter("timings_topic").value,
            self.on_timings, 10)
        self.create_subscription(
            String, "/n01/live_navigation/hive2_feasibility", self.on_robot_feasibility, 10)
        self.create_subscription(
            String, "/n01/live_navigation/dynamic_obstacle_status",
            self.on_dynamic_obstacle_status, 10)
        self.create_subscription(
            Bspline, "/planning/bspline", self.on_planner_trajectory, 10)
        self.ready_pub = self.create_publisher(
            Bool, "/n01/live_navigation/navigation_ready", 10)
        self.health_pub = self.create_publisher(
            String, "/n01/live_navigation/system_health", 10)
        self.create_timer(0.25, self.publish_health)

    def on_bridge(self, msg):
        try:
            self.bridge = json.loads(msg.data)
        except (ValueError, TypeError):
            self.bridge = {"status": "INVALID_BRIDGE_STATUS"}

    def on_static(self, msg):
        self.static_ready = bool(msg.data)

    def on_body_pose(self, msg):
        self.body_stamp_ns = (
            int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec))
        self.body_rx_monotonic = time.monotonic()

    def on_tf(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if parent == "map" and child == "base_footprint":
                stamp_ns = (
                    int(transform.header.stamp.sec) * 1000000000
                    + int(transform.header.stamp.nanosec))
                if self.last_tf_stamp_ns is not None and stamp_ns > self.last_tf_stamp_ns:
                    self.tf_intervals.append((stamp_ns - self.last_tf_stamp_ns) / 1e9)
                self.last_tf_stamp_ns = stamp_ns
                self.map_tf_stamp_ns = stamp_ns

    def on_timings(self, msg):
        self.timings = [float(msg.vector.x), float(msg.vector.y), float(msg.vector.z)]
        self.timings_stamp_ns = (
            int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec))
        self.timings_rx_monotonic = time.monotonic()

    def on_robot_feasibility(self, msg):
        try:
            self.robot_feasibility = json.loads(msg.data)
        except (ValueError, TypeError):
            self.robot_feasibility = {"status": "INVALID_ROBOT_FEASIBILITY"}
        self.robot_feasibility_rx_monotonic = time.monotonic()

    def on_dynamic_obstacle_status(self, msg):
        try:
            self.dynamic_obstacle = json.loads(msg.data)
        except (ValueError, TypeError):
            self.dynamic_obstacle = {"status": "INVALID_DYNAMIC_OBSTACLE_STATUS"}
        self.dynamic_obstacle_rx_monotonic = time.monotonic()

    def on_planner_trajectory(self, msg):
        self.planner_rx_monotonic = time.monotonic()
        self.planner_traj_id = int(msg.traj_id)

    def publish_health(self):
        now_ns = self.get_clock().now().nanoseconds
        now_mono = time.monotonic()
        body_stamp_age = (
            (now_ns - self.body_stamp_ns) / 1e9 if self.body_stamp_ns is not None else None)
        body_rx_age = (
            now_mono - self.body_rx_monotonic if self.body_rx_monotonic is not None else None)
        tf_age = (
            (now_ns - self.map_tf_stamp_ns) / 1e9 if self.map_tf_stamp_ns is not None else None)
        tf_rate = (
            1.0 / (sum(self.tf_intervals) / len(self.tf_intervals))
            if self.tf_intervals and sum(self.tf_intervals) > 0 else None)
        timings_age = (
            (now_ns - self.timings_stamp_ns) / 1e9 if self.timings_stamp_ns is not None else None)

        bridge_ready = (
            self.bridge.get("status") == "LIVE_SCAN_INPUTS_READY"
            and self.bridge.get("localization_status_fresh") is True
            and self.bridge.get("robot_pose_receive_fresh") is True
            and self.bridge.get("odom_receive_fresh") is True
            and self.bridge.get("motion_compensation_ready") is True
            and self.bridge.get("sensor_extrinsic_ready") is True
            and self.bridge.get("cloud_output_fresh") is True
        )
        body_fresh = (
            body_stamp_age is not None and body_rx_age is not None
            and -0.2 <= body_stamp_age <= self.max_body_age
            and 0.0 <= body_rx_age <= self.max_body_rx_age
        )
        dynamic_age = (
            now_mono - self.dynamic_obstacle_rx_monotonic
            if self.dynamic_obstacle_rx_monotonic is not None else None)
        feasibility_age = (
            now_mono - self.robot_feasibility_rx_monotonic
            if self.robot_feasibility_rx_monotonic is not None else None)
        planner_age = (
            now_mono - self.planner_rx_monotonic
            if self.planner_rx_monotonic is not None else None)
        tf_seen = self.map_tf_stamp_ns is not None
        blockers = []
        if not bridge_ready:
            blockers.append("live_localization_or_scan_bridge_not_ready")
        if not self.static_ready:
            blockers.append("static_pcd_overlay_not_ready_in_current_window")
        if not body_fresh:
            blockers.append("body_pose_missing_or_stale")
        # A direct map->base_footprint TF edge is optional: the vendor's
        # /robot_pose Odometry is the global pose source. The bridge only marks
        # itself ready after odometry-assisted scan-time pose compensation and
        # body<-LiDAR extrinsic lookup both succeed.
        navigation_ready = not blockers
        stage_b_ready = bool(navigation_ready and self.static_ready)
        dynamic_receive_fresh = (
            dynamic_age is not None and 0.0 <= dynamic_age <= 1.0)
        dynamic_stamp_fresh, dynamic_stamp_age = stamped_data_is_fresh(
            self.dynamic_obstacle.get("cloud_stamp_ns"),
            now_ns,
            1.0,
        )
        dynamic_fresh = bool(dynamic_receive_fresh and dynamic_stamp_fresh)
        dynamic_status = self.dynamic_obstacle.get("status", "WAITING")
        dynamic_status_valid = dynamic_status in (
            "NO_UNMATCHED_CLUSTERS", "CANDIDATES_AVAILABLE")
        scene_diagnostics_ready = bool(dynamic_fresh and dynamic_status_valid)
        planner_fresh = planner_age is not None and 0.0 <= planner_age <= 1.0
        feasibility_fresh = feasibility_age is not None and 0.0 <= feasibility_age <= 1.0
        stage_c_ready = bool(
            feasibility_fresh
            and self.robot_feasibility.get("status") == "NECESSARY_CONDITIONS_PASS"
            and self.robot_feasibility.get("candidate_review_ready") is True
        )
        stages = {
            "A_localization_and_goal_gate": {
                "status": "READY" if navigation_ready else "BLOCKED",
                "ready": bool(navigation_ready),
                "navigation_ready": bool(navigation_ready),
                "blockers": list(blockers),
            },
            "B_static_dynamic_map_and_scan_planning": {
                "status": "READY_FOR_PLANNING" if stage_b_ready else "BLOCKED",
                "ready": stage_b_ready,
                "static_overlay_ready": bool(self.static_ready),
                "native_scan_cloud_input": "/cloud",
                "dynamic_tracker_status": dynamic_status,
                "dynamic_tracker_age_s": dynamic_age,
                "dynamic_tracker_fresh": bool(dynamic_fresh),
                "dynamic_tracker_receive_fresh": bool(dynamic_receive_fresh),
                "dynamic_tracker_stamp_age_s": dynamic_stamp_age,
                "dynamic_tracker_status_valid": bool(dynamic_status_valid),
                "dynamic_tracker_review_required": bool(
                    dynamic_status == "CANDIDATES_AVAILABLE"),
                "dynamic_tracker_role": "VISUALIZATION_ONLY",
                "scene_diagnostics_ready": scene_diagnostics_ready,
                "scene_diagnostics_blockers": (
                    [] if scene_diagnostics_ready else [
                        "dynamic_obstacle_status_missing_stale_or_invalid"]),
                "planner_candidate_received": bool(planner_fresh),
                "planner_candidate_status": (
                    "FRESH" if planner_fresh
                    else "WAITING_FOR_GOAL_OR_PLANNER_OUTPUT"
                    if planner_age is None else "STALE"),
                "planner_candidate_age_s": planner_age,
                "planner_trajectory_id": self.planner_traj_id,
                "planner_is_native_scan": True,
            },
            "C_hive2_candidate_kinematics": {
                "status": (
                    "CANDIDATE_REVIEW_READY" if stage_c_ready
                    else self.robot_feasibility.get("status", "WAITING_FOR_TRAJECTORY")
                ),
                "ready_for_review": stage_c_ready,
                "candidate_only": True,
                "feasibility_age_s": feasibility_age,
                "feasibility": self.robot_feasibility,
                "execution_authorized": False,
            },
            "D_physical_safety_acceptance": {
                "status": "NOT_AUTHORIZED",
                "execution_authorized": False,
                "actuator_output": "DISABLED",
                "safety_authorization": "NEVER",
                "required_physical_acceptance": [
                    "emergency_stop_chain_verified",
                    "communication_loss_stop_verified",
                    "localization_loss_stop_verified",
                    "dynamic_obstacle_stop_verified",
                    "manual_takeover_verified",
                    "wheel_controller_mapping_verified",
                    "wheel_contact_geometry_verified",
                    "collision_envelope_verified",
                    "braking_distance_verified",
                ],
            },
        }
        health = {
            "schema": "n01.live_system_health.v2",
            "status": "READY_FOR_GOAL_PLANNING" if navigation_ready else "BLOCKED",
            "navigation_ready": bool(navigation_ready),
            "blockers": blockers,
            "bridge": self.bridge,
            "static_overlay_ready": bool(self.static_ready),
            "body_pose_stamp_age_s": body_stamp_age,
            "body_pose_receive_age_s": body_rx_age,
            "map_base_tf_observed": bool(tf_seen),
            "map_base_tf_stamp_age_s": tf_age,
            "map_base_tf_rate_hz": tf_rate,
            "global_pose_source": self.bridge.get("robot_pose_topic"),
            "global_pose_interpolation_ready": (
                self.bridge.get("robot_pose_receive_fresh") is True),
            "global_pose_anchor_ready": (
                self.bridge.get("robot_pose_receive_fresh") is True),
            "odometry_motion_compensation_ready": (
                self.bridge.get("motion_compensation_ready") is True),
            "sensor_extrinsic_ready": (
                self.bridge.get("sensor_extrinsic_ready") is True),
            "vendor_processing_timings_raw_xyz": self.timings,
            "vendor_processing_timings_age_s": timings_age,
            "vendor_processing_timings_semantics": (
                "vendor-defined Vector3Stamped; raw x/y/z only, no guessed units or labels"),
            "robot_feasibility": self.robot_feasibility,
            "dynamic_obstacle_status": self.dynamic_obstacle,
            "dynamic_obstacle_status_fresh": bool(dynamic_fresh),
            "dynamic_obstacle_cloud_stamp_age_s": dynamic_stamp_age,
            "planner_candidate_fresh": bool(planner_fresh),
            "stages": stages,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
        self.ready_pub.publish(Bool(data=bool(navigation_ready)))
        self.health_pub.publish(String(data=json.dumps(health, sort_keys=True)))
        self.last_health = health


def main(args=None):
    rclpy.init(args=args)
    node = LiveRuntimeSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
