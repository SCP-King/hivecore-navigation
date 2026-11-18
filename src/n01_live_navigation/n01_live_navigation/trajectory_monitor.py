"""B-spline visualization and non-actuating Hive2 four-wheel steering screen."""
import json
import math
import time
import uuid
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray
from scan_planner_msgs.msg import Bspline, NavigationTrajectory
from geometry_msgs.msg import Point

from n01_scan_dynacbf_bridge.curve import BSplineCurve, CurveError
from .core import four_wheel_steer_screen, trajectory_window_is_active
from .pose_policy import control_pose_fresh
from .footprint_collision import FootprintCollision
from .scan_handoff import SplineEvents, sample_native_prefix
from .core import decode_pointcloud_xyz


PITCH1_JOINTS = (
    "fl_leg_pitch1", "fr_leg_pitch1", "rl_leg_pitch1", "rr_leg_pitch1")
PITCH2_JOINTS = (
    "fl_leg_pitch2", "fr_leg_pitch2", "rl_leg_pitch2", "rr_leg_pitch2")


def _live_param(obj,name,fallback):
    """Read a declared parameter at use time so GUI changes apply without restart."""
    try:
        return obj.get_parameter(name).value
    except Exception:  # noqa: BLE001
        return fallback


class Hive2TrajectoryMonitor(Node):
    def __init__(self):
        super().__init__("n01_hive2_trajectory_monitor")
        defaults = {
            "bspline_topic": "/planning/bspline",
            "body_pose_topic": "/body_pose",
            "joint_states_topic": "/joint_states",
            "map_frame": "map",
            "map_id": "",
            "max_speed_mps": 0.25,
            "max_accel_mps2": 0.25,
            "wheel_base_x_m": 0.56,
            "track_y_m": 0.27,
            "wheel_radius_m": 0.095,
            "delta_y_m": 0.085,
            "steering_limit_rad": 3.141592653589793,
            "execution_preview_m": 1.0,
            "require_observed_space": False,
            "pitch1_home_reference_rad": 0.7853981633974483,
            "pitch2_home_reference_rad": -1.3089969389957472,
            "pitch_reference_tolerance_rad": 0.08,
            "max_bspline_receive_age_s": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.frame = str(get("map_frame"))
        self.map_id = str(get("map_id")) or "navigation_session:"+uuid.uuid4().hex
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
        self.spline_events = SplineEvents()
        self.obstacles = self.ground_plane = self.map_window = None
        self.obstacles_rx = self.plane_rx = 0.
        self.obstacles_stamp_ns = 0
        self.last_error = ""
        self.last_bspline_rx = None
        self.last_bspline = None
        self.last_curve = None
        self.accepted_goal_ns = 0
        self.accepted_goal_xy=None
        self.accepted_goal_yaw=0.
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
        self.require_observed_space=bool(self.get_parameter('require_observed_space').value)
        self.observed_free=None;self.observed_free_rx=0.;self.observed_free_stamp=0
        self.create_subscription(PointCloud2,'/grid_map/observed_free',self.on_observed_free,qos_profile_sensor_data)
        from .scene_perception import setup as setup_scene_perception
        self.scene_perception=setup_scene_perception(self,None)
        from .dynamic_obstacles import ObstacleTracks
        self.dynamic_tracks=ObstacleTracks(self.declare_parameter("perception.prediction_horizons_s",[.25,.5,.75]).value)
        self.risk_speed_scale=float(self.declare_parameter('perception.risk_speed_scale',.5).value)
        # Scene freshness for trajectory review. These are our internal gates,
        # not a contractual number; the header stamp is the source scan time.
        self.scene_max_age_s=float(self.declare_parameter('perception.scene_max_age_s',3.0).value)
        self.ground_max_age_s=float(self.declare_parameter('perception.ground_max_age_s',3.0).value)
        self.observed_free_max_age_s=float(self.declare_parameter('perception.observed_free_max_age_s',3.0).value)
        if not .1<=self.observed_free_max_age_s<=3.:
            raise ValueError('perception.observed_free_max_age_s must be in [0.1, 3.0]')
        if not .1<=self.scene_max_age_s<=3.:
            raise ValueError('perception.scene_max_age_s must be in [0.1, 3.0]')
        if not .5<=self.ground_max_age_s<=10.:
            raise ValueError('perception.ground_max_age_s must be in [0.5, 10.0]')
        if not 0<self.risk_speed_scale<=1:raise ValueError('invalid_risk_speed_scale')
        self.unconfirmed_risk=np.empty((0,3))
        def risk(msg):
            try:self.unconfirmed_risk=decode_pointcloud_xyz(msg,120000,None)if msg.width else np.empty((0,3))
            except ValueError:pass
        self.create_subscription(PointCloud2,'/grid_map/unconfirmed_risk',risk,qos_profile_sensor_data)
        self.prediction_pub=self.create_publisher(PointCloud2,'/n01/live_navigation/predicted_obstacles',qos_profile_sensor_data)

        self.create_subscription(PointCloud2, '/grid_map/occupancy', self.on_obstacles, qos_profile_sensor_data)
        self.create_subscription(String, '/grid_map/static_overlay_status', self.on_window, latched)
        self.create_subscription(String, '/n01/ground_reference/status', self.on_plane, 10)

        self.trajectory_pub = self.create_publisher(NavigationTrajectory, "/n01/navigation/trajectory", 10)
        from .rotation_scene import setup
        setup(self)
        self.path_pub = self.create_publisher(
            Path, "/n01/live_navigation/hive2_candidate_path", 10)
        self.wheel_marker_pub = self.create_publisher(
            MarkerArray, "/n01/live_navigation/hive2_wheel_kinematics", 10)
        self.replan_pub=self.create_publisher(Bool,'/n01/navigation/replan',10)
        self.last_prefix_replan_key=None
        self.diag_pub = self.create_publisher(
            String, "/n01/live_navigation/hive2_feasibility", 10)
        self.create_subscription(
            Bspline, str(get("bspline_topic")), self.on_bspline, 10)
        self.create_subscription(PoseStamped,'/n01/live_navigation/goal_gated',self.on_goal,10)
        self.create_subscription(
            Odometry, str(get("body_pose_topic")), self.on_body_pose, qos_profile_sensor_data)
        self.create_subscription(
            JointState, str(get("joint_states_topic")), self.on_joint_states, qos_profile_sensor_data)
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

    def publish_missing_or_stale_status(self):
        now_mono = time.monotonic()
        if self.accepted_goal_xy is not None and self.body_pose is not None:
            p=self.body_pose.pose.pose.position
            if math.hypot(p.x-self.accepted_goal_xy[0],p.y-self.accepted_goal_xy[1])<=.08:
                try:
                    q=self.body_pose.pose.pose.orientation;yaw=self._yaw_from_quaternion(q)
                    states=[(p.x,p.y,p.z,yaw)]
                    self.check_native_scene(states)
                    age=(self.get_clock().now().nanoseconds-self.body_pose_stamp_ns)/1e9
                    ready=control_pose_fresh(age,now_mono-self.body_pose_rx) and self._pitch_reference_match()is True
                    self.publish_trajectory(None,None,states,{},ready)
                except (ValueError,TypeError,KeyError):pass
                return
        # Bspline is an event, not a heartbeat. Revalidate current feedback and
        # scene while preserving the original mathematical time domain.
        if self.last_bspline is not None and self.last_trajectory_window:
            start_ns, duration_s = self.last_trajectory_window
            active, _ = trajectory_window_is_active(
                start_ns, self.get_clock().now().nanoseconds, duration_s)
            # Once accepted, execution progress owns the parameter clock.
            # Each replay still requires fresh scene/model/state admission.
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
            "goal_id": self.accepted_goal_ns,
            "issues": Hive2TrajectoryMonitor.scene_issues(self),
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
        self.accepted_goal_yaw=self._yaw_from_quaternion(msg.pose.orientation)
        self.spline_events = SplineEvents()
        self.last_bspline=None;self.last_curve=None;self.last_bspline_rx=None;self.last_trajectory_window=None
        self._clear_candidate_visualization()

    def stage_scene(self,kind,stamp,points):
        if stamp<=0:return
        if not getattr(self,'require_observed_space',False):
            if kind=='occupied' and stamp>self.obstacles_stamp_ns:
                self.obstacles=points;self.obstacles_stamp_ns=stamp;self.obstacles_rx=time.monotonic()
            return
        if stamp<=self.obstacles_stamp_ns:return
        pending=getattr(self,'pending_scenes',{})
        pending.setdefault(stamp,{})[kind]=(points,time.monotonic())
        pair=pending[stamp]
        if 'occupied'in pair and 'free'in pair:
            self.obstacles,self.obstacles_rx=pair['occupied'];self.obstacles_stamp_ns=stamp
            self.observed_free,self.observed_free_rx=pair['free'];self.observed_free_stamp=stamp
            pending={k:v for k,v in pending.items()if k>stamp}
        self.pending_scenes=dict(sorted(pending.items())[-3:])

    def on_observed_free(self,msg):
        if msg.header.frame_id!=self.frame:return
        try:
            points=decode_pointcloud_xyz(msg,200000,None) if msg.width else np.empty((0,3))
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            Hive2TrajectoryMonitor.stage_scene(self,'free',stamp,points)
        except ValueError:self.last_error='invalid_observation_coverage'

    def observed_ready(self):
        if not getattr(self,'require_observed_space',False):return True
        return (self.observed_free is not None and self.observed_free_stamp==self.obstacles_stamp_ns and
                time.monotonic()-self.observed_free_rx<=_live_param(self,'perception.observed_free_max_age_s',getattr(self,'observed_free_max_age_s',.6)) and
                -.1<=(self.get_clock().now().nanoseconds-self.observed_free_stamp)/1e9<=_live_param(self,'perception.observed_free_max_age_s',getattr(self,'observed_free_max_age_s',.6)))

    def on_obstacles(self,msg):
        if msg.header.frame_id!=self.frame:return
        try:
            points=(np.empty((0,3)) if msg.width==0 and not msg.data else decode_pointcloud_xyz(msg,1000000,None))
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            Hive2TrajectoryMonitor.stage_scene(self,'occupied',stamp,points)
        except ValueError:self.last_error='invalid_obstacle_cloud'

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

    def scene_issues(self):
        now=time.monotonic();ns=self.get_clock().now().nanoseconds;issues=[]
        def issue(code,**facts):issues.append(dict(reason=code,**facts))
        if not Hive2TrajectoryMonitor.observed_ready(self):
            issue('waiting_for_observed_local_space',occupied_stamp_ns=self.obstacles_stamp_ns,
                  coverage_stamp_ns=getattr(self,'observed_free_stamp',0),pending_versions=list(getattr(self,'pending_scenes',{})))
        if self.body_pose is None or self.body_pose_stamp_ns is None or self.body_pose_rx is None:issue('body_pose_missing')
        else:
            age=(ns-self.body_pose_stamp_ns)/1e9;rx=now-self.body_pose_rx
            if self.body_pose.header.frame_id!=self.frame:issue('body_pose_frame_mismatch',actual=self.body_pose.header.frame_id,expected=self.frame)
            if not control_pose_fresh(age,rx):issue('control_pose_stale',stamp_age_s=age,receive_age_s=rx)
        if self.obstacles is None:issue('occupancy_missing')
        else:
            age=(ns-self.obstacles_stamp_ns)/1e9;rx=now-self.obstacles_rx
            lim=float(_live_param(self, 'perception.scene_max_age_s',getattr(self,'scene_max_age_s',.6)))
            if rx>lim or not -.1<=age<=lim:issue('occupancy_stale',stamp_age_s=age,receive_age_s=rx,limit_s=lim)
        if self.ground_plane is None:issue('ground_plane_unavailable')
        elif now-self.plane_rx>float(_live_param(self, 'perception.ground_max_age_s',getattr(self,'ground_max_age_s',2.))):issue('ground_plane_stale',receive_age_s=now-self.plane_rx,limit_s=float(_live_param(self, 'perception.ground_max_age_s',getattr(self,'ground_max_age_s',2.))))
        if self.map_window is None:issue('local_window_missing')
        if self.joint_rx is None or self.joint_stamp_ns is None:issue('wheel_feedback_missing')
        else:
            age=(ns-self.joint_stamp_ns)/1e9;rx=now-self.joint_rx
            if rx>.5 or not -.2<=age<=.5:issue('wheel_feedback_stale',stamp_age_s=age,receive_age_s=rx,limit_s=.5)
        if self._pitch_reference_match()is not True:issue('leg_feedback_reference_mismatch')
        return issues

    def scene_inputs_ready(self):
        return not Hive2TrajectoryMonitor.scene_issues(self)

    def check_native_scene(self, states, allow_prefix=False):
        """Review the original SCAN samples without searching for another path."""
        now = time.monotonic()
        stamp_age = (self.get_clock().now().nanoseconds-self.obstacles_stamp_ns)/1e9
        scene_limit=float(_live_param(self, 'perception.scene_max_age_s',getattr(self,'scene_max_age_s',.6)))
        ground_limit=float(_live_param(self, 'perception.ground_max_age_s',getattr(self,'ground_max_age_s',2.)))
        if (self.obstacles is None or now-self.obstacles_rx > scene_limit
                or not -.1 <= stamp_age <= scene_limit
                or self.ground_plane is None or now-self.plane_rx > ground_limit
                or self.map_window is None):
            # Name the missing input instead of one generic code.
            missing=[]
            if self.obstacles is None:missing.append('occupancy_missing')
            elif now-self.obstacles_rx > .6 or not -.1 <= stamp_age <= .6:missing.append('occupancy_stale')
            if self.ground_plane is None:missing.append('ground_plane_unavailable')
            elif now-self.plane_rx > ground_limit:missing.append('ground_plane_stale')
            if self.map_window is None:missing.append('local_window_missing')
            raise ValueError('scan_review_waiting_for_fresh_map_or_ground; '+','.join(missing or ['scene_unknown']))
        if not Hive2TrajectoryMonitor.observed_ready(self):raise ValueError('waiting_for_observed_local_space')
        footprint = self.footprint_client.get()
        if footprint is None:
            raise ValueError('scan_review_waiting_for_urdf_footprint')
        from .observed_space import observed_states,executable_prefix
        states=np.asarray(states,float).reshape(-1,4)
        observed=np.ones(len(states),bool)
        if getattr(self,'require_observed_space',False):
            observed=observed_states(states,footprint,self.observed_free)
        collision=FootprintCollision(self.obstacles,self.ground_plane,
            self.body_pose.pose.pose.position.z,self.map_window,footprint=footprint)
        geometry=collision.free_batch(states[:,[0,1,3]][:,None,:])
        valid=observed & geometry
        bad=np.flatnonzero(~valid)
        self.current_prefix_speed=self.max_speed
        self.prefix_limited=False
        if len(bad):
            i=int(bad[0]);self.failure_context={'position':states[i,:3].tolist(),'yaw':float(states[i,3]),
                'scene_stamp_ns':self.obstacles_stamp_ns,'coverage_stamp_ns':getattr(self,'observed_free_stamp',0),
                'reason':'footprint_coverage_missing'if not observed[i]else'collision_or_window_boundary'}
            if allow_prefix:
                prefix,speed=executable_prefix(states,valid,self.max_speed,self.max_accel,.6)
                self.current_prefix_speed=speed;self.prefix_limited=True
                key=(self.accepted_goal_ns,getattr(getattr(self,'last_bspline',None),'traj_id',-1),tuple(np.round(states[i,:2]/.05).astype(int)))
                if getattr(self,'last_prefix_replan_key',None)!=key:
                    if getattr(self,'replan_pub',None) is not None:self.replan_pub.publish(Bool(data=True))
                    self.last_prefix_replan_key=key
                return prefix
            raise ValueError('scan_footprint_enters_unknown_space; frontier_required'if not observed[i]
                else'scan_prefix_collision_or_outside_window; replan_required')
        self.failure_context={}
        return states.tolist()

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
        # Bind the producer goal ID and reject reordered/conflicting revisions
        # before replacing the event. Duplicate delivery never resets progress.
        try:
            event = self.spline_events.classify(msg, self.accepted_goal_ns)
        except (ValueError, TypeError, OverflowError) as exc:
            reason = str(exc)
            if reason != getattr(self, 'last_rejected_event', None):
                self.get_logger().warning('Ignoring SCAN event: '+reason)
                self.last_rejected_event = reason
            return  # A rejected event must not revoke a newer valid trajectory.
        committed = False
        try:
            # scan_planner_msgs/Bspline has no Header; its coordinate frame is
            # supplied by the live launch contract (map), not inferred from a field.
            curve = (self.last_curve if event == 'duplicate' and self.last_curve is not None
                     else BSplineCurve(int(msg.order), list(msg.knots),
                         [(p.x, p.y, p.z) for p in msg.pos_pts], table_samples=500))
            trajectory_start_ns = int(msg.start_time.sec)*10**9+int(msg.start_time.nanosec)
            trajectory_duration_s = float(curve.end-curve.start)
            if trajectory_start_ns<getattr(self,'accepted_goal_ns',0):
                raise ValueError('trajectory_from_previous_goal')
            active, _ = trajectory_window_is_active(trajectory_start_ns,
                self.get_clock().now().nanoseconds, trajectory_duration_s)
            if not active and event == 'new':
                raise ValueError('trajectory_window_expired')
            # Commit only structurally valid, active events. Perception/feedback
            # may recover on replay, but the original curve clock stays fixed.
            if not replay and event == 'new':
                self.spline_events.commit(msg)
                self.last_bspline_rx = time.monotonic()
                self.last_bspline = msg
                self.last_curve = curve
                committed = True
                self.last_trajectory_window = (trajectory_start_ns, trajectory_duration_s)
            if self.body_pose is None:
                raise ValueError('missing_body_pose_for_execution_preview')
            body = self.body_pose.pose.pose.position
            states = sample_native_prefix(curve, body.x, body.y, self.execution_preview)
            states=self.check_native_scene(states,allow_prefix=True)
            candidate_points = [p[:3] for p in states]
            points = candidate_points
            full_candidate_points = len(curve.table_xyz)
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
                runtime_limits=self.native_model_sync.limits(),
            )
        except (CurveError, ValueError, TypeError, OverflowError, KeyError) as exc:
            self.last_error = str(exc)
            if event == 'new' and not committed and self.last_bspline is not None:
                self.get_logger().warning('Ignoring malformed/inactive SCAN replacement: '+str(exc))
                return
            self._clear_candidate_visualization()
            if self.last_bspline is not None:
                try:self.publish_trajectory(self.last_bspline,self.last_curve,[],{},False)
                except (ValueError,TypeError,KeyError):pass
            self.diag_pub.publish(String(data=json.dumps({
                "schema": "n01.hive2_candidate_feasibility.v1",
            "goal_id": self.accepted_goal_ns,
            "issues": Hive2TrajectoryMonitor.scene_issues(self),
            "planning_footprint": self.footprint_client.description(),
                "status": "INVALID_OR_UNSUPPORTED_BSPLINE",
                "reason": self.last_error,
                "failure_context": getattr(self,"failure_context",{}),
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
            yaw = states[i][3]
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
        trajectory_active = True  # Source accepted once; fresh admission governs continued execution.
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
            "goal_id": self.accepted_goal_ns,
            "issues": Hive2TrajectoryMonitor.scene_issues(self),
            "path_model": "scan_bspline",
            "execution_stage": "forward",
            "goal_stamp_ns": self.accepted_goal_ns,
            "goal_xy": self.accepted_goal_xy,
            "trajectory_source": "/planning/bspline",
            "geometry_preserved": True,
            "source_fingerprint": self.spline_events.fingerprint(msg),
            "planning_footprint": self.footprint_client.description(),
            "candidate_stamp_ns": path.header.stamp.sec*10**9+path.header.stamp.nanosec,
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
            "collision_prefix_checked": True,
            "prefix_limited": getattr(self,'prefix_limited',False),
            "failure_context": getattr(self,'failure_context',{}),
            "reason": ('verified_prefix_retained; '+str(getattr(self,'failure_context',{}).get('reason',''))
                       if getattr(self,'prefix_limited',False) else ('admitted' if candidate_review_ready else 'feedback_or_native_model_not_ready')),
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
        self.publish_trajectory(msg, curve, states, screen, candidate_review_ready)
        data['risk_speed_advice_active']=getattr(self,'risk_advice_active',False)
        data['dynamic_predictions']=getattr(getattr(self,'dynamic_tracks',None),'stats',{})
        self.diag_pub.publish(String(data=json.dumps(data, sort_keys=True)))
        self.last_error = ""

    def publish_trajectory(self, source, curve, states, screen, ready):
        """One atomic execution carrier: original source plus current scene/model."""
        out=NavigationTrajectory()
        out.header.frame_id=self.frame;out.header.stamp=self.get_clock().now().to_msg()
        out.scene_valid=self.scene_inputs_ready()
        out.map_id=self.map_id
        out.goal_id=self.accepted_goal_ns
        out.revision=source.traj_id if source is not None else -1
        out.spline=source if source is not None else Bspline(goal_id=self.accepted_goal_ns,traj_id=-1)
        out.hold_position=source is None
        out.goal=Point(x=float(self.accepted_goal_xy[0]),y=float(self.accepted_goal_xy[1]))
        out.goal_yaw=float(self.accepted_goal_yaw)
        out.max_speed=float(screen.get('recommended_speed_mps',self.max_speed))
        out.max_speed=min(out.max_speed,self.max_speed,getattr(self,"current_prefix_speed",self.max_speed))
        out.max_accel=self.max_accel;out.environment_margin=.06
        out.chassis_geometry=self.native_model_sync.geometry().array().tolist()
        limits=self.native_model_sync.limits()
        out.max_steering_rate=float(limits['chassis.steer.max_speed'])
        out.max_wheel_rate=float(limits['chassis.drive.max_speed'])
        out.footprint=[Point(x=float(x),y=float(y)) for x,y in self.footprint_client.get()]
        out.ground_plane=list(map(float,self.ground_plane))
        lo,hi=self.map_window;out.window_bounds=[float(lo[0]),float(lo[1]),float(hi[0]),float(hi[1])]
        out.scene_stamp.sec=self.obstacles_stamp_ns//10**9;out.scene_stamp.nanosec=self.obstacles_stamp_ns%10**9
        out.require_observed_space=getattr(self,'require_observed_space',False)
        out.observed_resolution=.1
        if out.require_observed_space and Hive2TrajectoryMonitor.observed_ready(self):
            free=self.observed_free
            out.observed_free=PointCloud2()
            out.observed_free.header.frame_id='map';out.observed_free.header.stamp.sec=self.observed_free_stamp//10**9;out.observed_free.header.stamp.nanosec=self.observed_free_stamp%10**9
            out.observed_free.height=1;out.observed_free.width=len(free);out.observed_free.point_step=12;out.observed_free.row_step=len(free)*12;out.observed_free.is_dense=True
            out.observed_free.fields=[PointField(name=name,offset=offset,datatype=PointField.FLOAT32,count=1)for name,offset in [('x',0),('y',4),('z',8)]]
            out.observed_free.data=np.asarray(free,dtype='<f4').tobytes()

        self.risk_advice_active=False
        historical=getattr(self,'unconfirmed_risk',np.empty((0,3)))
        if len(historical) and len(states):
            risk=FootprintCollision(historical,self.ground_plane,self.body_pose.pose.pose.position.z,
                self.map_window,footprint=self.footprint_client.get())
            if not risk.free([(x,y,yaw)for x,y,z,yaw in states]):
                out.max_speed=min(out.max_speed,self.max_speed*_live_param(self, 'perception.risk_speed_scale',self.risk_speed_scale));self.risk_advice_active=True
        scene=self.obstacles
        try:
            from .rotation_scene import live_points
            raw,raw_stamp=live_points(self,with_stamp=True)
            out.rotation_stamp.sec=raw_stamp//10**9;out.rotation_stamp.nanosec=raw_stamp%10**9
            scene=np.concatenate((scene,raw));out.rotation_observed=True
            if getattr(self,'dynamic_tracks',None) is not None:
                predicted=self.dynamic_tracks.update(raw,raw_stamp/1e9)
                forecast=PointCloud2();forecast.header.frame_id='map';forecast.header.stamp=out.rotation_stamp
                forecast.height=1;forecast.width=len(predicted);forecast.point_step=12;forecast.row_step=len(predicted)*12;forecast.is_dense=True
                forecast.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)for i,n in enumerate(('x','y','z'))]
                forecast.data=np.asarray(predicted,dtype='<f4').tobytes();self.prediction_pub.publish(forecast)
                if len(predicted) and len(states):
                    risk=FootprintCollision(predicted,self.ground_plane,self.body_pose.pose.pose.position.z,
                        self.map_window,footprint=self.footprint_client.get())
                    if not risk.free([(x,y,yaw)for x,y,z,yaw in states]):
                        out.max_speed=min(out.max_speed,self.max_speed*_live_param(self, 'perception.risk_speed_scale',self.risk_speed_scale));self.risk_advice_active=True
        except ValueError:out.rotation_observed=False
        p=self.body_pose.pose.pose.position
        floor=scene[:,:2]@np.asarray(self.ground_plane)[:2]+self.ground_plane[2]
        mask=(scene[:,2]>floor+.03)&(scene[:,2]>=p.z-.90)&(scene[:,2]<=p.z+.25)
        mask&=(np.hypot(scene[:,0]-p.x,scene[:,1]-p.y)<=4.)
        scene=scene[mask]
        if len(scene):
            _,ids=np.unique(scene[:,:2],axis=0,return_index=True);scene=scene[ids]
        out.scene_center=Point(x=p.x,y=p.y,z=p.z);out.scene_radius=4.
        if len(scene)>120000:
            out.valid=False;out.reason='scene_certificate_point_budget_exceeded'
        else:
            out.obstacles=PointCloud2()
            out.obstacles.header.frame_id=self.frame;out.obstacles.header.stamp=out.scene_stamp
            out.obstacles.height=1;out.obstacles.width=len(scene)
            out.obstacles.fields=[PointField(name=name,offset=offset,datatype=PointField.FLOAT32,count=1)
                for name,offset in [('x',0),('y',4),('z',8)]]
            out.obstacles.point_step=12;out.obstacles.row_step=len(scene)*12;out.obstacles.is_dense=True
            out.obstacles.data=np.asarray(scene,dtype='<f4').tobytes()
            out.valid=bool(ready);out.reason='admitted'if ready else (self.last_error or '; '.join(i['reason']for i in Hive2TrajectoryMonitor.scene_issues(self)) or 'native_model_not_ready')
        out.samples=[]
        for x,y,z,yaw in states:
            from geometry_msgs.msg import Pose
            p=Pose();p.position=Point(x=float(x),y=float(y),z=float(z))
            p.orientation.z=math.sin(yaw/2);p.orientation.w=math.cos(yaw/2);out.samples.append(p)
        self.trajectory_pub.publish(out)


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
