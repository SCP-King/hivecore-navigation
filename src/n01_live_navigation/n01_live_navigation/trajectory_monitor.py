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
from .core import four_wheel_steer_screen


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
        self.body_pose = None
        self.body_pose_rx = None
        self.joint_positions = {}
        self.joint_rx = None
        self.last_error = ""

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

    def on_body_pose(self, msg):
        self.body_pose = msg
        self.body_pose_rx = time.monotonic()

    def on_joint_states(self, msg):
        self.joint_positions = {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
            if math.isfinite(float(position))
        }
        self.joint_rx = time.monotonic()

    def _pitch_reference_match(self):
        if self.joint_rx is None or time.monotonic() - self.joint_rx > 0.5:
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
            self.wheel_marker_pub.publish(array)
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
        try:
            # scan_planner_msgs/Bspline has no Header; its coordinate frame is
            # supplied by the live launch contract (map), not inferred from a field.
            curve = BSplineCurve(
                int(msg.order), list(msg.knots),
                [(p.x, p.y, p.z) for p in msg.pos_pts], table_samples=500)
            points = curve.table_xyz
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
            self.path_pub.publish(Path())
            self.wheel_marker_pub.publish(MarkerArray())
            self.diag_pub.publish(String(data=json.dumps({
                "schema": "n01.hive2_candidate_feasibility.v1",
                "status": "INVALID_OR_UNSUPPORTED_BSPLINE",
                "reason": self.last_error,
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
        self.path_pub.publish(path)
        self._publish_wheel_markers(path.header.stamp, screen)

        pitch_match = self._pitch_reference_match()
        data = {
            "schema": "n01.hive2_candidate_feasibility.v1",
            "status": (
                "NECESSARY_CONDITIONS_PASS" if screen["necessary_conditions_pass"]
                else "NECESSARY_CONDITIONS_FAIL"
            ),
            "candidate_only": True,
            "path_frame": self.frame,
            "path_points": len(path.poses),
            "trajectory_id": int(msg.traj_id),
            "trajectory_start_time": {
                "sec": int(msg.start_time.sec),
                "nanosec": int(msg.start_time.nanosec),
            },
            "max_speed_limit_mps": self.max_speed,
            "max_accel_limit_mps2": self.max_accel,
            **screen,
            "pitch_feedback_matches_home_reference": pitch_match,
            "pitch_reference_is_home_target_only": True,
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
