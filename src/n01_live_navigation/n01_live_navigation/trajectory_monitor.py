"""B-spline visualization and non-actuating Hive2 four-wheel steering screen."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from scan_planner_msgs.msg import Bspline

from n01_scan_dynacbf_bridge.curve import BSplineCurve, CurveError
from .core import four_wheel_steer_screen, trajectory_window_is_active


PITCH1_JOINTS = (
    "fl_leg_pitch1", "fr_leg_pitch1", "rl_leg_pitch1", "rr_leg_pitch1")
PITCH2_JOINTS = (
    "fl_leg_pitch2", "fr_leg_pitch2", "rl_leg_pitch2", "rr_leg_pitch2")


class Hive2TrajectoryMonitor(Node):
    def __init__(self):
        super().__init__("n01_hive2_trajectory_monitor")
        defaults = {
            "bspline_topic": "/planning/bspline",
            "body_pose_topic": "/body_pose",
            "joint_states_topic": "/joint_states",
            "map_frame": "map",
            "max_speed_mps": 0.25,
            "max_accel_mps2": 0.25,
            "wheel_base_x_m": 0.56,
            "track_y_m": 0.27,
            "wheel_radius_m": 0.095,
            "steering_limit_rad": 1.5708,
            "pitch1_home_reference_rad": 0.7853981633974483,
            "pitch2_home_reference_rad": -1.3089969389957472,
            "pitch_reference_tolerance_rad": 0.08,
            "max_bspline_receive_age_s": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.frame = str(get("map_frame"))
        self.max_speed = float(get("max_speed_mps"))
        self.max_accel = float(get("max_accel_mps2"))
        self.wheel_base_x = float(get("wheel_base_x_m"))
        self.track_y = float(get("track_y_m"))
        self.wheel_radius = float(get("wheel_radius_m"))
        self.steering_limit = float(get("steering_limit_rad"))
        self.pitch1_ref = float(get("pitch1_home_reference_rad"))
        self.pitch2_ref = float(get("pitch2_home_reference_rad"))
        self.pitch_tol = float(get("pitch_reference_tolerance_rad"))
        self.max_bspline_receive_age = float(get("max_bspline_receive_age_s"))
        if not 0.2 <= self.max_bspline_receive_age <= 5.0:
            raise ValueError("max_bspline_receive_age_s outside bounded limits")
        self.body_pose = None
        self.body_pose_rx = None
        self.body_pose_stamp_ns = None
        self.joint_positions = {}
        self.joint_rx = None
        self.joint_stamp_ns = None
        self.last_error = ""
        self.last_bspline_rx = None
        self.last_watchdog_status = None

        self.path_pub = self.create_publisher(
            Path, "/n01/live_navigation/hive2_candidate_path", 10)
        self.wheel_marker_pub = self.create_publisher(
            MarkerArray, "/n01/live_navigation/hive2_wheel_kinematics", 10)
        self.diag_pub = self.create_publisher(
            String, "/n01/live_navigation/hive2_feasibility", 10)
        self.create_subscription(
            Bspline, str(get("bspline_topic")), self.on_bspline, 10)
        self.create_subscription(
            Odometry, str(get("body_pose_topic")), self.on_body_pose, qos_profile_sensor_data)
        self.create_subscription(
            JointState, str(get("joint_states_topic")), self.on_joint_states, qos_profile_sensor_data)
        self.create_timer(0.25, self.publish_missing_or_stale_status)

    def _clear_wheel_markers(self, stamp):
        markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)
        self.wheel_marker_pub.publish(markers)

    def _clear_candidate_visualization(self):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = stamp
        self.path_pub.publish(path)
        self._clear_wheel_markers(stamp)

    def publish_missing_or_stale_status(self):
        now_mono = time.monotonic()
        if self.last_bspline_rx is not None:
            receive_age = max(0.0, now_mono - self.last_bspline_rx)
            if receive_age <= self.max_bspline_receive_age:
                return
            status = "STALE_BSPLINE"
            reason = "bspline_receive_timeout"
        else:
            receive_age = None
            status = "WAITING_FOR_BSPLINE"
            reason = "no_bspline_received"

        if self.last_watchdog_status != status:
            self._clear_candidate_visualization()
            self.last_watchdog_status = status
        self.diag_pub.publish(String(data=json.dumps({
            "schema": "n01.hive2_candidate_feasibility.v1",
            "status": status,
            "reason": reason,
            "trajectory_active": False,
            "trajectory_receive_age_s": receive_age,
            "max_bspline_receive_age_s": self.max_bspline_receive_age,
            "candidate_review_ready": False,
            "candidate_review_status": status,
            "candidate_only": True,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }, sort_keys=True)))

    def on_body_pose(self, msg):
        self.body_pose = msg
        self.body_pose_rx = time.monotonic()
        self.body_pose_stamp_ns = (
            int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec))

    def on_joint_states(self, msg):
        self.joint_positions = {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
            if math.isfinite(float(position))
        }
        self.joint_stamp_ns = (
            int(msg.header.stamp.sec) * 1000000000
            + int(msg.header.stamp.nanosec))
        self.joint_rx = time.monotonic()

    def _pitch_reference_match(self):
        if self.joint_rx is None or time.monotonic() - self.joint_rx > 0.5:
            return None
        if self.joint_stamp_ns is None or self.joint_stamp_ns <= 0:
            return None
        joint_stamp_age = (
            self.get_clock().now().nanoseconds - self.joint_stamp_ns) / 1e9
        if not -0.2 <= joint_stamp_age <= 0.5:
            return None
        required = list(PITCH1_JOINTS) + list(PITCH2_JOINTS)
        if any(name not in self.joint_positions for name in required):
            return None
        errors = []
        for name in PITCH1_JOINTS:
            errors.append(abs(self.joint_positions[name] - self.pitch1_ref))
        for name in PITCH2_JOINTS:
            errors.append(abs(self.joint_positions[name] - self.pitch2_ref))
        return bool(max(errors) <= self.pitch_tol)

    @staticmethod
    def _yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _publish_wheel_markers(self, stamp, screen):
        array = MarkerArray()
        if self.body_pose is None or self.body_pose.header.frame_id.lstrip("/") != self.frame:
            self._clear_wheel_markers(stamp)
            return
        pose = self.body_pose.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        wheel_positions = screen["wheel_positions_m"]
        steering = screen["wheel_steering_angles_rad"]
        for marker_id, name in enumerate(("fl", "fr", "rl", "rr")):
            local_x, local_y = wheel_positions[name]
            marker = Marker()
            marker.header.frame_id = self.frame
            marker.header.stamp = stamp
            marker.ns = "hive2_nominal_wheel_steering"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = pose.position.x + c * local_x - s * local_y
            marker.pose.position.y = pose.position.y + s * local_x + c * local_y
            marker.pose.position.z = pose.position.z
            arrow_yaw = yaw + steering[name]
            marker.pose.orientation.z = math.sin(arrow_yaw / 2.0)
            marker.pose.orientation.w = math.cos(arrow_yaw / 2.0)
            marker.scale.x, marker.scale.y, marker.scale.z = 0.35, 0.045, 0.045
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.8, 1.0, 0.9
            array.markers.append(marker)
        self.wheel_marker_pub.publish(array)

    def on_bspline(self, msg):
        self.last_bspline_rx = time.monotonic()
        try:
            # scan_planner_msgs/Bspline has no Header; its coordinate frame is
            # supplied by the live launch contract (map), not inferred from a field.
            curve = BSplineCurve(
                int(msg.order), list(msg.knots),
                [(p.x, p.y, p.z) for p in msg.pos_pts], table_samples=500)
            points = curve.table_xyz
            trajectory_start_ns = (
                int(msg.start_time.sec) * 1000000000
                + int(msg.start_time.nanosec))
            trajectory_duration_s = float(curve.end - curve.start)
            screen = four_wheel_steer_screen(
                points,
                max_speed=self.max_speed,
                max_accel=self.max_accel,
                wheel_base_x=self.wheel_base_x,
                track_y=self.track_y,
                wheel_radius=self.wheel_radius,
                steering_limit=self.steering_limit,
            )
        except (CurveError, ValueError, TypeError, OverflowError) as exc:
            self.last_error = str(exc)
            self._clear_candidate_visualization()
            self.diag_pub.publish(String(data=json.dumps({
                "schema": "n01.hive2_candidate_feasibility.v1",
                "status": "INVALID_OR_UNSUPPORTED_BSPLINE",
                "reason": self.last_error,
                "trajectory_active": False,
                "candidate_review_ready": False,
                "candidate_only": True,
                "actuator_output": "DISABLED",
                "safety_authorization": "NEVER",
            }, sort_keys=True)))
            return

        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = self.get_clock().now().to_msg()
        for i, xyz in enumerate(points):
            before = points[max(0, i - 1)]
            after = points[min(len(points) - 1, i + 1)]
            yaw = math.atan2(after[1] - before[1], after[0] - before[0])
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        pitch_match = self._pitch_reference_match()
        now_mono = time.monotonic()
        now_ns = self.get_clock().now().nanoseconds
        trajectory_active, trajectory_start_age_s = trajectory_window_is_active(
            trajectory_start_ns, now_ns, trajectory_duration_s)
        if trajectory_active:
            self.path_pub.publish(path)
            self._publish_wheel_markers(path.header.stamp, screen)
            self.last_watchdog_status = None
        else:
            self._clear_candidate_visualization()
            self.last_watchdog_status = "TRAJECTORY_NOT_ACTIVE_OR_STALE"
        body_stamp_age = (
            (now_ns - self.body_pose_stamp_ns) / 1e9
            if self.body_pose_stamp_ns is not None else None)
        body_rx_age = (
            now_mono - self.body_pose_rx if self.body_pose_rx is not None else None)
        joint_rx_age = (
            now_mono - self.joint_rx if self.joint_rx is not None else None)
        joint_stamp_age = (
            (now_ns - self.joint_stamp_ns) / 1e9
            if self.joint_stamp_ns is not None else None)
        body_pose_frame_match = bool(
            self.body_pose is not None
            and self.body_pose.header.frame_id.lstrip("/") == self.frame)
        body_feedback_fresh = (
            body_pose_frame_match
            and body_stamp_age is not None and body_rx_age is not None
            and -0.2 <= body_stamp_age <= 0.35 and 0.0 <= body_rx_age <= 0.4)
        joint_feedback_fresh = (
            joint_rx_age is not None and 0.0 <= joint_rx_age <= 0.5
            and joint_stamp_age is not None
            and -0.2 <= joint_stamp_age <= 0.5)
        candidate_review_ready = bool(
            trajectory_active and screen["necessary_conditions_pass"]
            and body_feedback_fresh and joint_feedback_fresh and pitch_match is True)
        data = {
            "schema": "n01.hive2_candidate_feasibility.v1",
            "status": (
                "TRAJECTORY_NOT_ACTIVE_OR_STALE" if not trajectory_active
                else "NECESSARY_CONDITIONS_PASS" if screen["necessary_conditions_pass"]
                else "NECESSARY_CONDITIONS_FAIL"
            ),
            "candidate_only": True,
            "path_frame": self.frame,
            "path_points": len(path.poses) if trajectory_active else 0,
            "trajectory_id": int(msg.traj_id),
            "trajectory_start_time": {
                "sec": int(msg.start_time.sec),
                "nanosec": int(msg.start_time.nanosec),
            },
            "trajectory_start_age_s": trajectory_start_age_s,
            "trajectory_duration_s": trajectory_duration_s,
            "trajectory_active": trajectory_active,
            "max_speed_limit_mps": self.max_speed,
            "max_accel_limit_mps2": self.max_accel,
            **screen,
            "pitch_feedback_matches_home_reference": pitch_match,
            "pitch_reference_is_home_target_only": True,
            "body_pose_frame_match": body_pose_frame_match,
            "body_pose_feedback_fresh": bool(body_feedback_fresh),
            "body_pose_stamp_age_s": body_stamp_age,
            "body_pose_receive_age_s": body_rx_age,
            "joint_feedback_fresh": bool(joint_feedback_fresh),
            "joint_feedback_stamp_age_s": joint_stamp_age,
            "joint_feedback_receive_age_s": joint_rx_age,
            "candidate_review_ready": candidate_review_ready,
            "candidate_review_status": (
                "TRAJECTORY_NOT_ACTIVE_OR_STALE" if not trajectory_active
                else "CANDIDATE_REVIEW_READY" if candidate_review_ready
                else "WAITING_FOR_FRESH_BODY_JOINT_FEEDBACK_OR_PITCH_MATCH"
            ),
            "pitch_hold_feedback_verified": False,
            "wheel_controller_mapping_verified": False,
            "wheel_contact_geometry_verified": False,
            "collision_envelope_verified": False,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
        self.diag_pub.publish(String(data=json.dumps(data, sort_keys=True)))
        self.last_error = ""

def main(args=None):
    rclpy.init(args=args)
    node = Hive2TrajectoryMonitor()
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
