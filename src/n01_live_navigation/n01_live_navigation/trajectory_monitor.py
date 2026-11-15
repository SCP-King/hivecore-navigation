"""B-spline visualization and non-actuating Hive2 four-wheel steering screen."""
import json
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from scan_planner_msgs.msg import Bspline

from n01_scan_dynacbf_bridge.curve import BSplineCurve, CurveError
from .core import four_wheel_steer_screen, trajectory_window_is_active
from .navigation_path import execution_prefix
from .pose_policy import control_pose_fresh
from .heading_search import FootprintCollision, SearchResult, search, revalidate_cached, FOOTPRINT_RADIUS, FOOTPRINT_CENTERS, footprint_description, ENVIRONMENT_MARGIN
from .core import decode_pointcloud_xyz


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
            "delta_y_m": 0.085,
            "steering_limit_rad": 3.141592653589793,
            "execution_preview_m": 1.0,
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
        self.delta_y = float(get("delta_y_m"))
        self.steering_limit = float(get("steering_limit_rad"))
        self.execution_preview = float(get('execution_preview_m'))
        if not .3 <= self.execution_preview <= 2.:
            raise ValueError('execution_preview_m must be within [0.3, 2.0]')
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
        from .footprint_client import FootprintClient
        self.footprint_client=FootprintClient(self)
        self.heading_cache = None
        self.heading_search_at = 0.
        self.obstacles = self.ground_plane = self.map_window = None
        self.obstacles_rx = self.plane_rx = self.odom_rx = 0.
        self.obstacles_stamp_ns = 0
        self.measured_twist = None
        self.last_error = ""
        self.last_bspline_rx = None
        self.last_bspline = None
        self.accepted_goal_ns = 0
        self.accepted_goal_xy=None
        self.global_reference=None;self.global_reference_rx=0.
        self.readiness=None
        self.last_trajectory_window = None
        self.last_watchdog_status = None
        # Keep the last feasible candidate Path and re-publish it while its
        # B-spline is active. The GUI/RViz may subscribe after the one-shot
        # on_bspline callback; without replay they would see no path.
        self.last_path = None
        self.last_path_active = False
        from .runtime_chassis_model import RuntimeChassisModel
        self.native_model_sync=RuntimeChassisModel(self)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, '/grid_map/occupancy', self.on_obstacles, qos_profile_sensor_data)
        self.create_subscription(String, '/grid_map/static_overlay_status', self.on_window, latched)
        self.create_subscription(String, '/n01/ground_reference/status', self.on_plane, 10)
        self.create_subscription(Odometry, '/odom', self.on_odometry, qos_profile_sensor_data)

        self.path_pub = self.create_publisher(
            Path, "/n01/live_navigation/hive2_candidate_path", 10)
        self.wheel_marker_pub = self.create_publisher(
            MarkerArray, "/n01/live_navigation/hive2_wheel_kinematics", 10)
        self.diag_pub = self.create_publisher(
            String, "/n01/live_navigation/hive2_feasibility", 10)
        self.create_subscription(
            Bspline, str(get("bspline_topic")), self.on_bspline, 10)
        self.create_subscription(PoseStamped,'/n01/live_navigation/goal_gated',self.on_goal,10)
        self.create_subscription(
            Odometry, str(get("body_pose_topic")), self.on_body_pose, qos_profile_sensor_data)
        self.create_subscription(
            JointState, str(get("joint_states_topic")), self.on_joint_states, qos_profile_sensor_data)
        self.create_subscription(Path,'/planning/global_path',self.on_global_reference,latched)
        self.create_subscription(String,'/n01/live_navigation/system_health',self.on_health,10)
        from .rotation_task import setup as setup_turn_scene
        setup_turn_scene(self)
        self.create_timer(0.2, self.publish_missing_or_stale_status)

    def _clear_wheel_markers(self, stamp):
        markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)
        self.wheel_marker_pub.publish(markers)

    def _clear_candidate_visualization(self):
        self.last_path = None
        self.last_path_active = False
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = stamp
        self.path_pub.publish(path)
        self._clear_wheel_markers(stamp)

    def on_global_reference(self,msg):
        if msg.header.frame_id!=self.frame:return
        self.global_reference=msg;self.global_reference_rx=time.monotonic()

    def on_health(self,msg):
        try:self.readiness=json.loads(msg.data)
        except ValueError:self.readiness=None

    def publish_missing_or_stale_status(self):
        from .maneuver_review import publish_guide_maneuver
        if publish_guide_maneuver(self):return
        now_mono = time.monotonic()
        # Bspline is an event, not a heartbeat. Revalidate the original time
        # window and current feedback; never extend its start/duration on replay.
        if self.last_bspline is not None and self.last_trajectory_window:
            start_ns, duration_s = self.last_trajectory_window
            active, _ = trajectory_window_is_active(
                start_ns, self.get_clock().now().nanoseconds, duration_s)
            if active:
                self.on_bspline(self.last_bspline, replay=True)
                return
            self._clear_candidate_visualization()
        if self.last_bspline_rx is not None:
            receive_age = max(0.0, now_mono - self.last_bspline_rx)
            if receive_age <= self.max_bspline_receive_age:
                # Replay the active candidate path so a GUI/RViz subscriber
                # that joined after the B-spline callback still receives it.
                if self.last_path is not None and self.last_path_active:
                    self.path_pub.publish(self.last_path)
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
            "planning_footprint": self.footprint_client.description(),
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

    def on_goal(self,msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if msg.header.frame_id.lstrip('/')!=self.frame or stamp<=self.accepted_goal_ns:return
        self.accepted_goal_ns=stamp
        self.accepted_goal_xy=(msg.pose.position.x,msg.pose.position.y)
        self.global_reference=None
        self.maneuver_cache=None
        self.last_bspline=None;self.last_bspline_rx=None;self.last_trajectory_window=None
        self.heading_cache=None;self.heading_search_failure=None
        self._clear_candidate_visualization()

    def on_obstacles(self, msg):
        self.obstacles = None
        if msg.header.frame_id != self.frame:
            return
        try:
            self.obstacles = (np.empty((0, 3)) if msg.width == 0 and not msg.data
                              else decode_pointcloud_xyz(msg, 1000000, None))
            self.obstacles_rx = time.monotonic()
            self.obstacles_stamp_ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        except ValueError:
            pass

    def on_window(self, msg):
        self.map_window = None
        try:
            d = json.loads(msg.data)
            if d.get('ready') is True and d.get('source_frame') == self.frame:
                self.map_window = [d['window_min_xyz'][:2], d['window_max_xyz'][:2]]
        except (ValueError, KeyError, TypeError):
            pass

    def on_plane(self, msg):
        self.ground_plane = None
        try:
            d = json.loads(msg.data); p = d['plane']
            if (d['state'] == 'ground_projection_ready' and p['residual'] <= .04
                    and math.hypot(p['a'], p['b']) <= math.tan(math.radians(2))):
                self.ground_plane = [p['a'], p['b'], p['c']]
                self.plane_rx = time.monotonic()
        except (ValueError, KeyError, TypeError):
            pass

    def on_odometry(self, msg):
        self.measured_twist = None
        age = (self.get_clock().now().nanoseconds -
               (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        if (msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link'
                and -.1 <= age <= .3 and math.isfinite(v) and math.isfinite(w)):
            self.measured_twist = (v, w)
            self.odom_rx = time.monotonic()

    def heading_candidate(self, guide):
        now = time.monotonic()
        stamp_age = (self.get_clock().now().nanoseconds-self.obstacles_stamp_ns)/1e9
        if (self.obstacles is None or now-self.obstacles_rx > .6 or not -.1 <= stamp_age <= .6
                or self.ground_plane is None or now-self.plane_rx > 2.
                or self.map_window is None or self.measured_twist is None or now-self.odom_rx > .3):
            raise ValueError('heading_search_waiting_for_fresh_map_ground_or_odometry')
        body_age = (self.get_clock().now().nanoseconds-self.body_pose_stamp_ns)/1e9
        if (self.body_pose.header.frame_id != self.frame or not control_pose_fresh(body_age, now-self.body_pose_rx)):
            raise ValueError('heading_search_waiting_for_fresh_body_pose')
        pose = self.body_pose.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation)
        v, w = self.measured_twist
        if abs(v) < .01 and abs(w) > .02:
            raise ValueError('heading_search_waiting_for_rotation_to_stop')
        curvature = w/v if abs(v) >= .01 else 0.
        if v < -.01:
            raise ValueError('heading_search_waiting_for_forward_compatible_state')
        target = execution_prefix(guide, (pose.position.x, pose.position.y), 2.)
        end = target[-1]
        collision = FootprintCollision(self.obstacles, self.ground_plane,
                                       pose.position.z, self.map_window, footprint=self.footprint_client.get())
        # A prefix must rejoin the guide in its forward direction. At the final
        # point, station orientation is handled by the separate arrival turn.
        goal_yaw = (math.atan2(end[1]-target[-2][1], end[0]-target[-2][0])
                    if math.dist(end[:2], guide[-1][:2]) > .1 else None)
        start = (pose.position.x, pose.position.y, yaw, curvature)
        from .motion_capability import ForwardCapability
        capability=ForwardCapability(self.native_model_sync.geometry(),
            [self.joint_positions[f'{leg}_ankle_yaw']for leg in ('fl','fr','rl','rr')],
            max(self.max_speed,.012))
        failure=getattr(self,'heading_search_failure',None)
        if (failure and now-failure['at']<.4 and math.dist(start[:2],failure['start'][:2])<.02
                and abs(math.atan2(math.sin(yaw-failure['start'][2]),math.cos(yaw-failure['start'][2])))<math.radians(2)
                and math.dist(end[:2],failure['target'])<.03):
            raise ValueError(failure['reason'])
        cache = getattr(self, 'heading_cache', None)
        cached = None
        if (cache and now-cache['created'] <= 2.
                and math.dist(cache['guide_goal'], guide[-1][:2]) <= .03
                and math.dist(cache['target'], end[:2]) <= .03
                and cache['goal_yaw'] == goal_yaw):
            cached = revalidate_cached(cache['states'], start, collision)
        if cached and not capability.segment_allowed(cached):cached=None
        # Revalidate against every fresh scene; periodically seek a new path.
        # The original cache lifetime is never extended by replaying it.
        if cached and now-getattr(self, 'heading_search_at', 0.) < .8:
            result = SearchResult(cached, 'revalidated_previous_heading_path', 0, 0.)
        else:
            self.heading_search_at = now
            result = search(start, end[:2], collision, goal_yaw=goal_yaw, capability=capability)
            if result.states:
                self.heading_search_failure=None
                self.heading_cache = dict(states=result.states, created=now,
                    guide_goal=tuple(guide[-1][:2]), target=tuple(end[:2]), goal_yaw=goal_yaw)
            elif cached:
                result = SearchResult(cached, 'revalidated_after_'+result.reason,
                                      result.expanded, result.elapsed_ms)
            else:
                self.heading_cache = None
                self.heading_search_failure=dict(at=now,start=start,target=tuple(end[:2]),reason=result.reason)
                raise ValueError(result.reason)
        self.heading_collision_radius=collision.radius+(math.dist(result.states[0][:2],start[:2])
            if result.reason.startswith('revalidated') else 0.)
        n = len(result.states)
        points = [(s[0], s[1], pose.position.z+(end[2]-pose.position.z)*i/(n-1))
                  for i, s in enumerate(result.states)]
        return points, result

    def on_body_pose(self, msg):
        self.body_pose = msg
        self.body_pose_rx = time.monotonic()
        self.body_pose_stamp_ns = (
            int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec))

    def on_joint_states(self, msg):
        if not any(name in msg.name for name in (*PITCH1_JOINTS,*PITCH2_JOINTS)):
            return  # Disjoint wheel-spin updates are not leg-pose observations.
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

    def on_bspline(self, msg, replay=False):
        reference=getattr(self,'global_reference',None)
        goal=getattr(self,'accepted_goal_xy',None)
        if reference is not None and reference.poses and goal is not None:
            end=reference.poses[-1].pose.position
            if math.hypot(end.x-goal[0],end.y-goal[1])<=.12:return
        if not replay:
            self.last_bspline_rx = time.monotonic()
            self.last_bspline = msg
        try:
            # scan_planner_msgs/Bspline has no Header; its coordinate frame is
            # supplied by the live launch contract (map), not inferred from a field.
            curve = BSplineCurve(
                int(msg.order), list(msg.knots),
                [(p.x, p.y, p.z) for p in msg.pos_pts], table_samples=500)
            trajectory_start_ns = int(msg.start_time.sec)*10**9+int(msg.start_time.nanosec)
            trajectory_duration_s = float(curve.end-curve.start)
            if trajectory_start_ns<getattr(self,'accepted_goal_ns',0):
                raise ValueError('trajectory_from_previous_goal')
            self.last_trajectory_window = (trajectory_start_ns, trajectory_duration_s)
            active, _ = trajectory_window_is_active(trajectory_start_ns,
                self.get_clock().now().nanoseconds, trajectory_duration_s)
            if not active:
                raise ValueError('trajectory_window_expired')
            points = curve.table_xyz
            full_candidate_points = len(points)
            if self.body_pose is None:
                raise ValueError('missing_body_pose_for_execution_preview')
            body = self.body_pose.pose.pose.position
            start = min(range(len(points)), key=lambda i:
                        math.hypot(points[i][0]-body.x, points[i][1]-body.y))
            candidate_points, heading_result = self.heading_candidate(points[start:])
            points = execution_prefix(candidate_points, (body.x, body.y), self.execution_preview)
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
                delta_y=self.delta_y,
                initial_steering=[self.joint_positions[f'{leg}_ankle_yaw']
                                  for leg in ('fl','fr','rl','rr')],
                body_yaw=None,  # Forward + yaw follows path tangent, with no body lateral command.
                geometry_override=self.native_model_sync.geometry(),
                allow_retime=True,
            )
        except (CurveError, ValueError, TypeError, OverflowError, KeyError) as exc:
            self.last_error = str(exc)
            self._clear_candidate_visualization()
            self.diag_pub.publish(String(data=json.dumps({
                "schema": "n01.hive2_candidate_feasibility.v1",
            "planning_footprint": self.footprint_client.description(),
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
        for i, xyz in enumerate(candidate_points):
            yaw = heading_result.states[i][2]
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
        self.last_trajectory_window = (trajectory_start_ns, trajectory_duration_s)
        if trajectory_active:
            self.last_path = path
            self.last_path_active = True
            self.path_pub.publish(path)
            self._publish_wheel_markers(path.header.stamp, screen)
            self.last_watchdog_status = None
        else:
            self.last_path = None
            self.last_path_active = False
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
            and control_pose_fresh(body_stamp_age, body_rx_age))
        joint_feedback_fresh = (
            joint_rx_age is not None and 0.0 <= joint_rx_age <= 0.5
            and joint_stamp_age is not None
            and -0.2 <= joint_stamp_age <= 0.5)
        candidate_review_ready = bool(
            trajectory_active and screen["necessary_conditions_pass"]
            and body_feedback_fresh and joint_feedback_fresh and pitch_match is True)
        data = {
            "schema": "n01.hive2_candidate_feasibility.v1",
            "path_model": "forward_se2_curvature_lattice",
            "planning_footprint": self.footprint_client.description(),
            "candidate_stamp_ns": path.header.stamp.sec*10**9+path.header.stamp.nanosec,
            "heading_search_ms": heading_result.elapsed_ms,
            "heading_path_source": heading_result.reason,
            "heading_search_expanded": heading_result.expanded,
            "status": (
                "TRAJECTORY_NOT_ACTIVE_OR_STALE" if not trajectory_active
                else "NECESSARY_CONDITIONS_PASS" if screen["necessary_conditions_pass"]
                else "NECESSARY_CONDITIONS_FAIL"
            ),
            "candidate_only": True,
            "path_frame": self.frame,
            "path_points": len(path.poses) if trajectory_active else 0,
            "screened_prefix_only": True,
            "validated_prefix_points": len(points),
            "execution_preview_m": self.execution_preview,
            "full_candidate_points": full_candidate_points,
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
                else "KINEMATIC_CONSTRAINT_FAILED" if not screen['necessary_conditions_pass']
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
    from .runtime_chassis_model import configure_chassis_transport
    configure_chassis_transport()
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
