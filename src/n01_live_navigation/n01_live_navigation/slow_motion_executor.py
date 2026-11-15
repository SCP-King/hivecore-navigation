"""Opt-in, very slow path executor for the N-01 chassis.

This is the only node in the N-01 stack that may publish /cmd_vel.  It is
disabled by default, requires an explicitly enabled process, a fresh local
path, a fresh body pose and a READY runtime supervisor.  It publishes zero on
every unsafe transition and on shutdown, except while yielding to an external
manual command source, when it stops publishing entirely.
"""
from __future__ import annotations

import json
import math
import time
import os
import subprocess
from ament_index_python.packages import get_package_prefix
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8
from sensor_msgs.msg import PointCloud2, JointState
from .core import decode_pointcloud_xyz
from .cbf_velocity import project_speed, point_obstacles
from .reduced_tracking import reduced_yaw_rate, translation_velocity, prefix_requires_extension, braking_speed_limit
from .hive2_kinematics import inverse, MODEL_ID, controller_geometry
from .path_select import select_source
from .feedback_continuity import SteeringRateGuard
from .pose_policy import tracking_mode, PREDICTION_SPEED, control_pose_fresh
from .arrival_confirmation import measurement_confirms, prediction_metadata

PATH_TOPIC = "/n01/navigation/path"
POSE_TOPIC = "/body_pose"
READINESS_TOPIC = "/n01/live_navigation/system_health"
ESTOP_TOPIC = "/estop"
DEC_STOP_STATE_TOPIC = "/chassis_dec_stop/state"
CMD_VEL_TOPIC = "/cmd_vel"
STATUS_TOPIC = "/n01/live_navigation/slow_motion/status"


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class SlowMotionExecutor(Node):
    def __init__(self) -> None:
        super().__init__("n01_slow_motion_executor")
        self.declare_parameter('motion_mode','translate')
        self.declare_parameter('target_yaw_rad',0.)
        self.motion_mode=str(self.get_parameter('motion_mode').value)
        self.target_yaw=float(self.get_parameter('target_yaw_rad').value)
        if self.motion_mode not in ('translate','rotate') or not math.isfinite(self.target_yaw):
            raise ValueError('invalid motion task')
        self.face_path=bool(self.declare_parameter("face_path",False).value)
        self.require_maneuver_review=True
        self.path_turn_active=False
        self.turn_recovery_until=0.;self.turn_replan_at=0.;self.turn_replan_count=0
        self.declare_parameter("enabled", False)
        self.declare_parameter("task_id", "")
        self.task_id=str(self.get_parameter("task_id").value)
        self.declare_parameter("cbf_enabled", False)
        self.declare_parameter("tracking_model", "reduced_v_omega_curvature")
        self.declare_parameter("max_speed_mps", 0.05)
        self.declare_parameter("max_yaw_rate_radps", 0.15)
        self.declare_parameter("lookahead_m", 0.55)
        self.declare_parameter("control_period_s", 0.1)
        self.declare_parameter("path_timeout_s", 300.0)
        self.declare_parameter("pose_timeout_s", 0.6)
        self.declare_parameter("readiness_timeout_s", 2.0)
        self.declare_parameter("max_accel_mps2", 0.05)
        self.declare_parameter("max_session_distance_m", 0.0)
        self.declare_parameter("max_session_duration_s", 0.0)
        # auto: use the 4WIS steering guard when it passes, otherwise fall back
        # to a speed-scaled differential-drive /cmd_vel command.  four_wis keeps
        # the strict guard; diff_drive always uses the fallback.
        self.declare_parameter("control_mode", "four_wis")
        self.declare_parameter("dry_run", False)
        # unified: follow /n01/navigation/path (validated local prefix).
        # global:  emergency fallback requested by the operator; follow the
        #          planner's /planning/global_path directly with diff-drive
        #          assist.  estop/CBF/pose/health checks remain active, but the
        #          unified prefix and local candidate review are bypassed.
        # auto:    unified first; hold the last validated prefix for
        #          unified_grace_s across a replanning gap, then fall back to
        #          the fresh global path.  This is the default: a replan must
        #          not zero /cmd_vel while a validated path is still followable.
        self.declare_parameter("path_source", "unified")
        self.declare_parameter("global_path_timeout_s", 30.0)
        self.declare_parameter("unified_grace_s", 1.0)
        get = lambda name: self.get_parameter(name).value
        self.enabled = bool(get("enabled"))
        self.cbf_enabled = bool(get("cbf_enabled"))
        self.tracking_model = str(get("tracking_model"))
        if self.tracking_model not in ('fixed_heading_translation','reduced_v_omega_curvature'):
            raise ValueError('unsupported_tracking_model')
        self.native_spin_threshold = controller_geometry().spin_threshold
        self.control_mode = str(get("control_mode")).lower()
        if self.control_mode not in ("auto", "four_wis", "diff_drive"):
            raise ValueError("control_mode must be auto, four_wis or diff_drive")
        self.dry_run = bool(get("dry_run"))
        self.path_source = str(get("path_source")).lower()
        if self.path_source not in ("unified", "global", "auto"):
            raise ValueError("path_source must be unified, global or auto")
        if self.path_source == 'global':
            raise ValueError('global reference is not an executable local collision-checked path')
        # Legacy auto callers retain compatibility, without the unsafe global
        # fallback that bypassed current local obstacle/feasibility review.
        self.path_source='unified'
        self.global_path_timeout = max(0.5, min(120.0, float(get("global_path_timeout_s"))))
        self.unified_grace = max(0.2, min(5.0, float(get("unified_grace_s"))))
        self.max_speed = float(get('max_speed_mps'))
        if not math.isfinite(self.max_speed) or not .01<=self.max_speed<=.8:
            raise ValueError('max_speed_mps must be finite and between 0.01 and 0.8')
        self.max_yaw = min(0.30, max(0.02, float(get("max_yaw_rate_radps"))))
        self.lookahead = min(1.0, max(0.25, float(get("lookahead_m"))))
        self.period = min(0.2, max(0.05, float(get("control_period_s"))))
        self.path_timeout = float(get("path_timeout_s"))
        self.pose_timeout = float(get("pose_timeout_s"))
        self.readiness_timeout = float(get("readiness_timeout_s"))
        self.max_accel = max(0.01, float(get("max_accel_mps2")))
        self.distance_limit = float(get("max_session_distance_m"))
        self.session_duration = float(get("max_session_duration_s"))
        if not (math.isfinite(self.distance_limit) and 0 <= self.distance_limit <= 10
                and math.isfinite(self.session_duration) and 0 <= self.session_duration <= 3600):
            raise ValueError("Invalid session travel/time limits")
        self.session_start = time.monotonic()
        self.odom_xy = None
        self.odom_rx = 0.0
        self.travelled_m = 0.0
        self.commanded_m = 0.0
        self.last_control = self.session_start
        from .motion_state import MotionStateMachine
        self.motion_fsm=MotionStateMachine()
        self.finished = False
        self.feasibility = None
        self.feasibility_rx = 0.0
        self.path_status = None
        self.path_status_rx = 0.0
        self.pending_paths = {}
        self.pending_path_status = {}
        self.obstacle_cloud = None
        self.cloud_rx = self.tracks_rx = 0.0
        self.tracks = None
        self.cbf_state = 'WAITING_FOR_SENSORS'
        self.final_goal = None
        self.arrival_check_started = None
        self.goal_distance = None
        self.declare_parameter('require_expected_goal',False)
        self.declare_parameter('expected_goal_x',0.)
        self.declare_parameter('expected_goal_y',0.)
        self.expected_goal=(float(get('expected_goal_x')),float(get('expected_goal_y'))) if get('require_expected_goal') else None
        if self.expected_goal and not all(math.isfinite(v)for v in self.expected_goal):
            raise ValueError('invalid expected route target')
        self.requested_goal_xy = self.expected_goal
        self.steering_feedback = None
        self.steering_feedback_rx = 0.0
        self.fallback_active = False
        self.fallback_reason = ""
        self.cmd_linear_x = 0.0
        self.cmd_linear_y = 0.0
        self.cmd_angular_z = 0.0
        self.command_owner_checked_at = 0.0
        self.competing_command_nodes = []
        self.manual_override = False
        self.manual_source = ''
        self.observer_buffer = b''
        self.observer_ready = False

        q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)
        self.path = None
        self.path_rx = 0.0
        self.path_valid_at = 0.0        # last time a (path, valid status) pair landed
        self.path_source_used = None    # unified | unified_grace | global
        self.global_path = None
        self.global_path_rx = 0.0
        self.pose = None
        self.pose_z = float('nan')
        self.pose_rx = 0.0
        self.readiness = None
        self.readiness_rx = 0.0
        self.estop = False
        self.dec_stop = None
        self.speed = 0.0
        self.state = "DISABLED"
        from .runtime_chassis_model import RuntimeChassisModel
        self.native_model_sync=RuntimeChassisModel(self)

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        own = [p for p in self.get_publishers_info_by_topic(CMD_VEL_TOPIC)
               if (p.node_name, p.node_namespace) == (self.get_name(), self.get_namespace())]
        if len(own) != 1:
            raise RuntimeError('cannot uniquely identify executor command publisher GID')
        env = os.environ.copy()
        env['N01_EXECUTOR_PUBLISHER_GID'] = bytes(own[0].endpoint_gid).hex()
        observer = os.path.join(get_package_prefix('n01_command_observer'),
                                'lib/n01_command_observer/command_observer')
        self.command_observer = subprocess.Popen([observer], stdout=subprocess.PIPE,
                                                  stderr=subprocess.DEVNULL, env=env)
        os.set_blocking(self.command_observer.stdout.fileno(), False)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.replan_goal_pub=self.create_publisher(PoseStamped,"/move_base_simple/goal",10)
        self.goal_reached_pub = self.create_publisher(PointStamped,'/n01/navigation/goal_reached',10)
        self.create_subscription(Path, PATH_TOPIC, self._on_path, q)
        global_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/planning/global_path',
                                 self._on_global_path, global_qos)
        self.create_subscription(Odometry, POSE_TOPIC, self._on_pose, q)
        self.create_subscription(String, READINESS_TOPIC, self._on_readiness, q)
        stop_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, ESTOP_TOPIC, self._on_estop, stop_qos)
        self.create_subscription(UInt8, DEC_STOP_STATE_TOPIC, self._on_dec_stop, stop_qos)
        self.create_subscription(Bool, '/n01/slow_motion/enable', self._on_enable, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, q)
        self.create_subscription(String, '/n01/live_navigation/hive2_feasibility', self._on_feasibility, q)
        self.create_subscription(String, '/n01/navigation/path_status', self._on_path_status, q)
        if self.cbf_enabled:
            self.create_subscription(PointCloud2, '/cloud', self._on_obstacle_cloud, q)
            self.create_subscription(String, '/n01/live_navigation/dynamic_obstacle_status', self._on_tracks, q)
        self.create_subscription(JointState, '/joint_states', self._on_steering, q)
        self.create_subscription(PoseStamped,'/n01/live_navigation/goal_gated',self._on_accepted_goal,10)
        if self.motion_mode=='rotate' or self.face_path:
            from .rotation_task import setup
            setup(self)
        self.create_timer(self.period, self._control)
        self.get_logger().warn(
            "SLOW MOTION EXECUTOR: %s; max_speed=%.3f m/s; path_source=%s; control_mode=%s; cbf_enabled=%s; "
            "publishes /cmd_vel only when enabled"
            % ("ENABLED" if self.enabled else "DISABLED", self.max_speed,
               self.path_source, self.control_mode, self.cbf_enabled))

    def _on_command_observed(self, source) -> None:
        """Yield on actual external traffic, including a manual stop command.

        Endpoint presence alone is normal: vendor teleop/follow publishers stay
        registered while idle. Never send periodic zeros against a manual owner.
        """
        self.manual_override = True
        self.manual_source = source
        self.speed = self.cmd_linear_x = self.cmd_linear_y = self.cmd_angular_z = 0.0

    def _poll_command_observer(self):
        try:
            chunk = os.read(self.command_observer.stdout.fileno(), 65536)
        except BlockingIOError:
            chunk = b''
        self.observer_buffer += chunk
        while b'\n' in self.observer_buffer:
            line, self.observer_buffer = self.observer_buffer.split(b'\n', 1)
            text = line.decode('utf-8', errors='replace')
            if text == 'READY':
                self.observer_ready = True
            elif text.startswith('SOURCE '):
                self._on_command_observed(text[7:])
        if self.command_observer.poll() is not None:
            self._on_command_observed('command observer exited; cannot confirm control ownership')

    def _on_global_path(self, msg: Path) -> None:
        self.global_path = msg
        self.global_path_rx = time.monotonic()

    def _on_path(self, msg: Path) -> None:
        stamp_age = (self.get_clock().now().nanoseconds -
                     (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)) / 1e9
        if msg.header.frame_id.lstrip('/') != 'map' or not -0.2 <= stamp_age <= 0.6:
            return
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if len(msg.poses)<2:
            # 清空信号（路径节点/轨迹监视器在清除时发一条空 Path）：不能覆盖
            # 正在跟随的最后有效路径，否则重规划窗口一开始就 NO_PATH。
            # 何时放弃由 select_source 的 grace 窗口 / path_timeout 决定。
            return
        self.pending_paths[stamp]=(msg,time.monotonic())
        self.pending_paths=dict(list(self.pending_paths.items())[-6:])
        self._pair_path(stamp)

    def _pair_path(self, stamp):
        if stamp not in self.pending_paths or stamp not in self.pending_path_status:
            return
        path,path_rx=self.pending_paths[stamp]
        status,status_rx=self.pending_path_status[stamp]
        current=(self.path.header.stamp.sec*10**9+self.path.header.stamp.nanosec) if self.path else 0
        if stamp < current or time.monotonic()-min(path_rx,status_rx)>.6:
            return
        self.path,self.path_rx=path,path_rx
        self.path_status,self.path_status_rx=status,status_rx
        self.path_valid_at=time.monotonic()

    def _on_steering(self, msg):
        names=[f'{leg}_ankle_yaw' for leg in ('fl','fr','rl','rr')]
        # JointState permits independent publishers for disjoint joint sets.
        # Wheel-spin-only updates must not erase ankle feedback or refresh its age.
        if not any(name in msg.name for name in names):
            return
        self.steering_feedback = None
        age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if not -.2 <= age <= .5:
            return
        try:
            indexes=[list(msg.name).index(name) for name in names]
            angles=[float(msg.position[i]) for i in indexes]
            rates=[float(msg.velocity[i]) for i in indexes]
            if not all(math.isfinite(v) for v in angles+rates):
                return
            self.steering_feedback=(angles,rates)
            self.steering_feedback_rx=time.monotonic()
        except (ValueError,IndexError,TypeError):
            return

    def _on_accepted_goal(self,msg):
        if getattr(self,'motion_mode','translate')=='rotate':
            self.finished=True;self.completion_state='GOAL_CHANGED';return
        p=msg.pose.position
        if self.expected_goal and math.hypot(p.x-self.expected_goal[0],p.y-self.expected_goal[1])>.05:
            self.finished=True;self.completion_state='GOAL_CHANGED';return
        if msg.header.frame_id.lstrip('/')=='map' and all(math.isfinite(v) for v in (p.x,p.y)):
            self.requested_goal_xy=(p.x,p.y)
            self.final_goal=None
            self.arrival_check_started=None

    def _handle_arrival(self,now):
        if self.final_goal is None:return False
        x,y,_=self.pose;g=self.final_goal
        self.goal_distance=math.hypot(g.x-x,g.y-y)
        # Stop on entering 8cm. A 2cm exit hysteresis avoids repeated starts
        # from millimetre localization noise while verifying at zero velocity.
        limit=.10 if self.arrival_check_started is not None else .08
        if self.goal_distance>=limit:
            self.arrival_check_started=None
            return False
        if self.arrival_check_started is None:
            self.arrival_check_started=now
            self.arrival_source_ns=(self.get_clock().now().nanoseconds
                if prediction_metadata(getattr(self,'readiness',None)) is not None else 0)
        confirmed=measurement_confirms(getattr(self,'readiness',None),
            self.arrival_source_ns,xy=(g.x,g.y))
        if now-self.arrival_check_started>=.5 and confirmed:
            self.finished=True;self.completion_state='ARRIVED'
            event=PointStamped();event.header.frame_id='map'
            event.header.stamp=self.get_clock().now().to_msg()
            event.point.x=float(g.x);event.point.y=float(g.y);event.point.z=float(g.z)
            self.goal_reached_pub.publish(event)
            self._zero('ARRIVED',f'settled_goal_dist={self.goal_distance:.3f}')
        else:self._zero('VERIFYING_ARRIVAL',f'goal_dist={self.goal_distance:.3f}; holding zero')
        return True

    def _chassis_command_valid(self, speed, yaw_rate, now, lateral=0.0):
        if self.steering_feedback is None or now-self.steering_feedback_rx>.5:
            return False,'fresh steering positions AND velocities required'
        angles,rates=self.steering_feedback
        try:
            sync=getattr(self,'native_model_sync',None)
            geometry=sync.geometry(now) if sync is not None else None
            targets,drive=inverse((speed,lateral,yaw_rate),angles,rates,geometry=geometry)
        except ValueError as exc:
            return False,str(exc)
        limit=min(math.pi,geometry.controller_steering_limit) if geometry is not None else math.pi
        if max(abs(v) for v in angles+list(targets))>limit+1e-6:
            return False,'confirmed steering range or live controller limit exceeded'
        if max(abs(v) for v in drive)>20.+1e-6:
            return False,'native wheel speed exceeds 20rad/s'
        if not hasattr(self,'steering_rate_guard'):
            self.steering_rate_guard=SteeringRateGuard()
        if not self.steering_rate_guard.update(rates,now):
            return False,'steering feedback sustained overspeed or >3.14rad/s; waiting for stable recovery'
        return True,'native geometric checks passed'

    def _on_path_status(self, msg):
        try:
            status=json.loads(msg.data)
            if not isinstance(status,dict):
                raise ValueError('invalid_status')
            if status.get('valid') is not True:
                # 无效状态（例如重规划中的 waiting_for_validated_local_segment）
                # 只作废待配对的候选，不覆盖最近一次有效 (path, status) 配对：
                # auto 模式在 grace 窗口内继续跟随最后验证前缀。
                self.pending_paths.clear()
                self.pending_path_status.clear()
                return
            stamp=int(status['path_stamp_ns'])
            self.pending_path_status[stamp]=(status,time.monotonic())
            self.pending_path_status=dict(list(self.pending_path_status.items())[-6:])
            self._pair_path(stamp)
        except (ValueError,KeyError,TypeError):
            self.pending_paths.clear()
            self.pending_path_status.clear()

    def _on_obstacle_cloud(self, msg):
        age = (self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        self.obstacle_cloud = None
        if msg.header.frame_id != 'map' or not -.2<=age<=.6:
            return
        try:
            self.obstacle_cloud = decode_pointcloud_xyz(msg, max_points=100000)
            self.cloud_rx = time.monotonic()
        except (ValueError, TypeError):
            pass

    def _on_tracks(self, msg):
        try:
            data = json.loads(msg.data)
            age = (self.get_clock().now().nanoseconds-int(data['cloud_stamp_ns']))/1e9
            if data.get('cloud_frame') != 'map' or not -.2<=age<=.6 or data.get('status') not in ('CANDIDATES_AVAILABLE','NO_UNMATCHED_CLUSTERS'):
                raise ValueError('invalid_dynamic_snapshot')
            self.tracks = data.get('tracks', [])
            self.tracks_rx = time.monotonic()
        except (ValueError, KeyError, TypeError):
            self.tracks = None

    def _on_pose(self, msg: Odometry) -> None:
        stamp_age = (self.get_clock().now().nanoseconds -
                     (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)) / 1e9
        if (msg.header.frame_id.lstrip('/') != 'map' or
                not control_pose_fresh(stamp_age,0.)):
            self.pose = None
            return
        p = msg.pose.pose
        if not all(math.isfinite(v) for v in (p.position.x,p.position.y,p.position.z,
                                               p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)):
            self.pose = None
            return
        yaw = math.atan2(2.0 * (p.orientation.w * p.orientation.z +
                                p.orientation.x * p.orientation.y),
                         1.0 - 2.0 * (p.orientation.y * p.orientation.y +
                                      p.orientation.z * p.orientation.z))
        self.pose = (float(p.position.x), float(p.position.y), yaw)
        self.pose_z = float(p.position.z)
        self.pose_stamp_age_at_rx = stamp_age
        self.pose_rx = time.monotonic()

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        stamp_age = (self.get_clock().now().nanoseconds -
                     (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)) / 1e9
        if (msg.header.frame_id.lstrip('/') != 'odom' or not -0.2 <= stamp_age <= 0.3
                or not all(math.isfinite(v) for v in (p.x, p.y))):
            return
        if self.odom_xy is not None:
            self.travelled_m += math.hypot(p.x-self.odom_xy[0], p.y-self.odom_xy[1])
        self.odom_xy = (p.x, p.y)
        self.odom_rx = time.monotonic()

    def _on_feasibility(self, msg):
        try:
            self.feasibility = json.loads(msg.data)
        except (ValueError, TypeError):
            self.feasibility = None
        self.feasibility_rx = time.monotonic()

    def _on_readiness(self, msg: String) -> None:
        try:
            self.readiness = json.loads(msg.data)
        except Exception:
            self.readiness = None
        self.readiness_rx = time.monotonic()

    def _on_estop(self, msg: Bool) -> None:
        self.estop = self.estop or bool(msg.data)

    def _on_enable(self, msg: Bool) -> None:
        requested = bool(msg.data)
        if requested and self.finished:
            return  # A completed bounded session requires a fresh process.
        if requested and not self.enabled:
            self.session_start = self.last_control = time.monotonic()
            self.travelled_m = self.commanded_m = 0.0
        self.enabled = requested
        if not requested and not self.manual_override:
            self._zero('DISABLED', 'operator disabled')

    def _on_dec_stop(self, msg: UInt8) -> None:
        self.dec_stop = int(msg.data)

    def _zero(self, state: str, reason: str = "") -> None:
        self.speed = 0.0
        self.cmd_linear_x = 0.0
        self.cmd_linear_y = 0.0
        self.cmd_angular_z = 0.0
        self.state = state
        if not self.manual_override:
            self.cmd_pub.publish(Twist())
        self._status(state, reason)

    def _status(self, state: str, reason: str = "") -> None:
        now = time.monotonic()
        from .motion_state import MotionStateMachine
        if not hasattr(self,'motion_fsm'):self.motion_fsm=MotionStateMachine()
        phase=self.motion_fsm.observe(state,now)
        payload = {
            "phase":phase.value,
            "phase_elapsed_s":round(now-self.motion_fsm.entered,3) if self.motion_fsm.entered is not None else 0.,
            "phase_transitions":self.motion_fsm.transitions,
            "state": state,
            "task_id": self.task_id,
            "motion_mode": self.motion_mode,
            "yaw_error_rad": getattr(self,'rotation_error',None),
            "manual_override": self.manual_override,
            "manual_source": self.manual_source,
            "reason": reason,
            "enabled": self.enabled,
            "speed_mps": round(self.speed, 4),
            "max_speed_mps": self.max_speed,
            "path_age_s": (None if not self.path_rx else round(now - self.path_rx, 3)),
            "pose_age_s": (None if not self.pose_rx else round(now - self.pose_rx, 3)),
            "tracking_continuity": tracking_mode((self.readiness or {}).get('bridge',{})),
            "estop": self.estop,
            "dec_stop": self.dec_stop,
            "travelled_m": round(self.travelled_m, 4),
            "commanded_m": round(self.commanded_m, 4),
            "distance_limit_m": self.distance_limit,
            "cbf_status": self.cbf_state,
            "cbf_enabled": self.cbf_enabled,
            "path_source": self.path_source,
            "path_source_used": self.path_source_used,
            "global_path_age_s": (None if not self.global_path_rx else round(now - self.global_path_rx, 3)),
            "control_mode": self.control_mode,
            "diff_drive_fallback": self.fallback_active,
            "fallback_reason": self.fallback_reason,
            "dry_run": self.dry_run,
            "cmd_linear_x": round(self.cmd_linear_x, 4),
            "cmd_angular_z": round(self.cmd_angular_z, 4),
            "kinematics_model": MODEL_ID,
            "tracking_model": self.tracking_model,
            "goal_distance_m": self.goal_distance,
            "cmd_linear_y": round(self.cmd_linear_y,4),
        }
        self.status_pub.publish(String(data=json.dumps(payload)))

    def _request_turn_replan(self, now):
        """Replan same goal through normal gate; never use global path as drive permission."""
        goal=getattr(self,'requested_goal_xy',None) or getattr(self,'expected_goal',None)
        if goal is None:
            self.finished=True;self.completion_state='RECOVERY_FAILED'
            self._zero('RECOVERY_FAILED','rotation blocked and no current goal available')
            return
        if self.turn_replan_count>=3 or (self.turn_replan_at and now-self.turn_replan_at<8.):
            return
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x=float(goal[0]);msg.pose.position.y=float(goal[1]);msg.pose.orientation.w=1.
        self.replan_goal_pub.publish(msg)
        self.turn_replan_at=now;self.turn_replan_count+=1
        self._zero('REPLANNING_AFTER_ROTATION_BLOCKED',
                   f'rotation swept space blocked; replan {self.turn_replan_count}/3; waiting for validated translation')

    def _face_global_path(self, now):
        """A global tangent may request checked rotation, never authorize translation."""
        from .rotation_task import control
        if self.path_turn_active:
            control(self,now)
            stalled = self.state in ('ROTATION_POSE_MISMATCH','ROTATION_INPUT_STALE','STOPPING_FOR_TURN',
                                     'CHASSIS_KINEMATICS_BLOCKED')
            if stalled:
                if getattr(self,'turn_stalled_since',None) is None:
                    self.turn_stalled_since=now
            else:
                self.turn_stalled_since=None
            recover = stalled and now-self.turn_stalled_since >= 2.
            if self.state=='ROTATION_BLOCKED' or recover:
                if recover:
                    self.turn_measurement_after_ns=self.get_clock().now().nanoseconds
                self.path_turn_active=False
                self.turn_recovery_until=now+30.
                self.turn_suppressed_until=now+30.
                self.turn_replan_at=0.;self.turn_replan_count=0
                self.path=None;self.path_rx=0.;self.path_valid_at=0.
                self._request_turn_replan(now)
                return True
            if getattr(self,'rotation_complete',False):
                self.rotation_complete=False
                self.path_turn_active=False
                self.turn_measurement_after_ns=self.get_clock().now().nanoseconds
                self.path=None;self.path_rx=0.;self.path_valid_at=0.
                self.turn_replan_count=0;self.turn_replan_at=0.
                self._request_turn_replan(now)
                self._zero('WAITING_FOR_REPLAN','heading aligned; waiting for newly validated local path')
            return True
        status=getattr(self,'path_status',None)
        review=status.get('execution_review')if isinstance(status,dict)else None
        if (isinstance(review,dict)and review.get('execution_stage')=='rotation'
                and review.get('candidate_review_ready')is True and status.get('valid')is True
                and now-getattr(self,'path_status_rx',0.)<=.6 and self.path is not None
                and status.get('path_stamp_ns')==self.path.header.stamp.sec*10**9+self.path.header.stamp.nanosec):
            stage=review['stages'][0];goal=review.get('goal_xy');expected=getattr(self,'requested_goal_xy',None)
            if expected is not None and (goal is None or math.dist(goal,expected)>.12):return False
            if math.dist(stage['origin_xy'],self.pose[:2])>.04:
                self._zero('WAITING_FOR_REPLAN','turn origin moved; waiting for current-pose maneuver');return True
            target=float(stage['target_yaw_rad'])
            if abs(_wrap(target-self.pose[2]))<math.radians(3):
                self._zero('WAITING_FOR_REPLAN','planned turn complete; waiting for forward review');return True
            self._zero('ALIGN_PATH','executing the collision-checked rotation stage')
            self.target_yaw=target;self.rotation_anchor=None;self.rotation_settled=None;self.rotation_began=None
            self.rotation_stop_since=None;self.rotation_odom_anchor=None;self.rotation_complete=False
            self.turn_stalled_since=None;self.path_turn_active=True
            return True
        if getattr(self,'require_maneuver_review',False):return False
        if getattr(self,'turn_recovery_until',0.) or now<getattr(self,'turn_suppressed_until',0.):
            return False
        # Prefer a fresh, reviewed forward curve over spinning toward a distant
        # global tangent. This is what lets the local planner drive around bends.
        feasibility = getattr(self, 'feasibility', None)
        if (isinstance(feasibility, dict)
                and feasibility.get('path_model') == 'forward_se2_curvature_lattice'
                and feasibility.get('candidate_review_ready') is True
                and now-getattr(self, 'feasibility_rx', 0.) <= .6
                and getattr(self, 'path', None) is not None
                and now-getattr(self, 'path_rx', 0.) <= .6):
            return False
        if self.global_path is None or now-self.global_path_rx>1. or not self.global_path.poses:
            return False
        poses=self.global_path.poses
        expected=getattr(self,'expected_goal',None)
        end=poses[-1].pose.position
        if expected is not None and math.hypot(end.x-expected[0],end.y-expected[1])>.12:
            return False
        x,y,yaw=self.pose
        closest=min(range(len(poses)),key=lambda i:math.hypot(poses[i].pose.position.x-x,poses[i].pose.position.y-y))
        target=None
        for p in poses[closest:]:
            if math.hypot(p.pose.position.x-x,p.pose.position.y-y)>=.35:
                target=p.pose.position;break
        if target is None:return False
        desired=math.atan2(target.y-y,target.x-x)
        if abs(_wrap(desired-yaw))<=math.radians(35):return False
        self._zero('ALIGN_PATH','path turns: stopping before swept-space rotation check')
        self.target_yaw=desired
        self.rotation_anchor=None;self.rotation_settled=None;self.rotation_began=None
        self.rotation_stop_since=None
        self.turn_stalled_since=None
        self.rotation_odom_anchor=None
        self.rotation_complete=False
        self.path_turn_active=True
        return True

    def _control(self) -> None:
        now = time.monotonic()
        self._poll_command_observer()
        if not self.observer_ready and not self.manual_override:
            self._status('WAITING_COMMAND_OBSERVER', 'waiting for command source observer')
            return
        if self.manual_override:
            self.state = 'MANUAL_OVERRIDE'
            self.last_control = now
            self._status('MANUAL_OVERRIDE', self.manual_source +
                         '; navigation yielded; click start again to resume')
            return
        self.commanded_m += abs(self.speed) * max(0.0, now-self.last_control)
        self.last_control = now
        if (self.finished or (self.enabled and self.session_duration>0 and now-self.session_start >= self.session_duration) or
                (self.distance_limit > 0 and max(self.travelled_m, self.commanded_m) >= self.distance_limit)):
            self.finished = True
            self._zero(getattr(self, 'completion_state', 'SESSION_COMPLETE'),
                       "session complete; restart required")
            return
        if not self.enabled:
            self._zero("DISABLED", "explicit enable required")
            return
        if self.estop:
            self._zero("ESTOP", "/estop")
            return
        if self.dec_stop is None:
            self._zero('WAITING_STOP_STATE','waiting for latched chassis stop state')
            return
        if self.dec_stop:
            self._zero("ESTOP", "/chassis_dec_stop/state")
            return
        receive_age=now-self.pose_rx
        estimate_age=(getattr(self,'pose_stamp_age_at_rx',0.) or 0.)+receive_age
        if self.pose is None or not control_pose_fresh(estimate_age,receive_age):
            self._zero("NO_POSE", "/body_pose stale/missing")
            return
        if self.distance_limit > 0 and (not self.odom_rx or now-self.odom_rx > 0.3):
            self._zero("NO_ODOM", "bounded test requires fresh local odometry")
            return
        if (self.readiness is None or
                now - self.readiness_rx > self.readiness_timeout):
            self._zero("NOT_READY", "readiness stale/missing")
            return
        blockers = self.readiness.get("blockers") if isinstance(self.readiness, dict) else None
        if not isinstance(self.readiness, dict) or self.readiness.get("navigation_ready") is not True:
            self._zero("NOT_READY", "runtime navigation_ready is not true")
            return
        if isinstance(blockers, list) and blockers:
            self._zero("BLOCKED", ",".join(str(x) for x in blockers[:4]))
            return
        if getattr(self,'motion_mode','translate')=='rotate':
            from .rotation_task import control
            control(self,now)
            return
        # Arrival is a zero-output decision based on fresh localization, not
        # permission to move. The planner may already stop sending trajectories
        # on measured arrival; that must not prevent settled arrival verification.
        if self._handle_arrival(now):return
        recovery=getattr(self,'turn_recovery_until',0.)
        if recovery and now>=recovery:
            self.finished=True;self.completion_state='RECOVERY_FAILED'
            self._zero('RECOVERY_FAILED','rotation blocked; no executable translation within 30 seconds')
            return
        if getattr(self,'face_path',False) and self._face_global_path(now):
            return
        sel = select_source(
            path_source=self.path_source, now=now,
            path=self.path, path_rx=self.path_rx, path_timeout=self.path_timeout,
            path_status=self.path_status, path_status_rx=self.path_status_rx,
            path_valid_at=self.path_valid_at, grace_s=self.unified_grace,
            global_path=self.global_path, global_path_rx=self.global_path_rx,
            global_timeout_s=self.global_path_timeout, pose=self.pose)
        if sel is None:
            recovery=getattr(self,'turn_recovery_until',0.)
            if recovery:
                if now>=recovery:
                    self.finished=True;self.completion_state='RECOVERY_FAILED'
                    self._zero('RECOVERY_FAILED','rotation blocked; no validated translation after 3 replans/30 seconds')
                    return
                self._request_turn_replan(now)
            global_age = (f"{now - self.global_path_rx:.1f}s"
                          if self.global_path is not None else "missing")
            self._zero("NO_PATH",
                       f"no followable path (source={self.path_source}, "
                       f"global_age={global_age}); " + str((self.path_status or {}).get("reason","waiting local validation")))
            return
        source, poses, prefix, _path_stamp = sel
        self.path_source_used = source
        # Path selection already binds status to this path's publication stamp.
        # Execute its attached review, not whichever new review arrived last.
        review=(self.path_status or {}).get('execution_review') if source=='unified' else None
        if not isinstance(review,dict):review=self.feasibility
        # 局部候选复核只在“当前有效 unified 路径”上强制：它是 local SCAN/CBF
        # 轨迹的可行性证据。grace / global 回退是操作员显式接受的低速应急模式，
        # CBF、pose、readiness、estop 等硬门仍全部生效（见下方）。
        if source == "unified" and (
                not isinstance(review, dict) or now-self.path_status_rx > 0.6 or
                review.get('candidate_review_ready') is not True or
                review.get('trajectory_active') is not True):
            self._zero("NO_VALID_TRAJECTORY", "candidate/feedback not ready")
            return
        if (source == 'unified' and review.get('path_model') == 'forward_se2_curvature_lattice'
                and review.get('candidate_stamp_ns') != (self.path_status or {}).get('candidate_stamp_ns')):
            self._zero('NO_VALID_TRAJECTORY','waiting for the review bound to this unified path')
            return
        if getattr(self,'turn_measurement_after_ns',0):
            if not measurement_confirms(self.readiness,self.turn_measurement_after_ns,
                                        xy=self.pose[:2],yaw=self.pose[2]):
                self._zero('WAITING_FOR_REPLAN','waiting for post-stop measured localization confirmation')
                return
            self.turn_measurement_after_ns=0
        if self.max_speed>float(review.get('max_speed_limit_mps',0.))+1e-6:
            self._zero('SPEED_PROFILE_MISMATCH','executor speed exceeds trajectory review profile; restart navigation with matching speed')
            return
        if review.get('execution_stage')=='rotation':
            self._zero('WAITING_FOR_REPLAN','rotation prefix never authorizes translation');return
        full_len = len(poses)
        if full_len < 2:
            self._zero("NO_PATH", "path has <2 poses")
            return
        g=poses[-1].pose.position
        if (self.requested_goal_xy is not None and
                math.hypot(g.x-self.requested_goal_xy[0],g.y-self.requested_goal_xy[1])>.12):
            self._zero('WAITING_FOR_NEW_GOAL_PATH','cached path belongs to a previous goal')
            return
        self.final_goal = poses[-1].pose.position
        poses = poses[:prefix]

        x, y, yaw = self.pose
        closest = 0
        best = float("inf")
        for i, ps in enumerate(poses):
            dx = ps.pose.position.x - x
            dy = ps.pose.position.y - y
            d2 = dx * dx + dy * dy
            if d2 < best:
                best = d2
                closest = i
        target = poses[-1]
        travelled = 0.0
        prev = poses[closest].pose.position
        for ps in poses[closest + 1:]:
            p = ps.pose.position
            travelled += math.hypot(p.x - prev.x, p.y - prev.y)
            prev = p
            if travelled >= self.lookahead:
                target = ps
                break
        tx = float(target.pose.position.x)
        ty = float(target.pose.position.y)
        dist = math.hypot(tx - x, ty - y)
        goal = self.final_goal
        goal_dist = math.hypot(goal.x - x, goal.y - y)
        self.goal_distance = goal_dist
        if self._handle_arrival(now):return
        prefix_end = poses[-1].pose.position
        if (source != "global" and prefix_requires_extension((x,y),(prefix_end.x,prefix_end.y),
                (goal.x,goal.y),prefix<full_len)):
            self._zero('WAITING_FOR_PATH_EXTENSION', 'validated prefix exhausted')
            return
        err = _wrap(math.atan2(ty - y, tx - x) - yaw)
        translation=self.tracking_model=='fixed_heading_translation'
        # Fixed-heading translation can also reverse its drive direction; the
        # native module optimizer decides the bounded steering branch.
        if not translation and abs(err) > 1.0:
            self._zero("ALIGN", f"heading_err={err:.2f}")
            return
        reviewed_speed=min(self.max_speed,float(review.get('recommended_speed_mps',self.max_speed)))
        if tracking_mode((self.readiness or {}).get('bridge',{}))=='PREDICTED_TRACKING':
            reviewed_speed=min(reviewed_speed,PREDICTION_SPEED)
        speed_target = reviewed_speed if translation else reviewed_speed * max(0.0, 1.0 - abs(err) / 0.8)
        moving_floor=min(reviewed_speed,self.native_spin_threshold+self.max_accel*self.period)
        speed_target = min(speed_target, max(moving_floor,reviewed_speed*goal_dist/0.5))
        available = math.hypot(poses[closest].pose.position.x-x,poses[closest].pose.position.y-y)
        for a,b in zip(poses[closest:],poses[closest+1:]):
            available += math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y)
        speed_target=min(speed_target,braking_speed_limit(min(available,goal_dist),self.speed,self.max_accel))
        accel_step = self.max_accel * self.period
        self.speed += max(-accel_step, min(accel_step, speed_target - self.speed))
        # Reduced (v, omega), vy=0 tracking. Curvature, not independent yaw
        # feedback, keeps the requested turn radius finite during acceleration.
        yaw_rate = reduced_yaw_rate(self.speed,err,dist,self.max_yaw,self.native_spin_threshold)
        if self.cbf_enabled:
            if self.obstacle_cloud is None or now-self.cloud_rx>.6 or self.tracks is None or now-self.tracks_rx>.6:
                self._zero('CBF_SENSOR_STALE', 'fresh cloud and dynamic snapshot required')
                return
            xyz = self.obstacle_cloud
            body_z = self.pose_z
            ground = body_z-.778
            mask = ((xyz[:,2]>ground+.08)&(xyz[:,2]<ground+1.4)&
                    (np.hypot(xyz[:,0]-x,xyz[:,1]-y)<3.0))
            try:
                obstacles=point_obstacles(xyz[mask],self.tracks)
            except (ValueError,TypeError):
                self._zero('CBF_INVALID_TRACK','invalid measured obstacle support')
                return
            self.speed, self.cbf_state = project_speed((x,y),yaw+err if translation else yaw,self.speed,obstacles)
            if self.speed<=1e-6:
                self._zero(self.cbf_state, 'CBF constrained to stop')
                return
        else:
            self.cbf_state = 'DISABLED_SCAN_ONLY_TEST'
        # CBF may reduce v after reference generation; recompute omega so a
        # reduced speed cannot accidentally turn this into a spin command.
        yaw_rate = reduced_yaw_rate(self.speed,err,dist,self.max_yaw,self.native_spin_threshold)
        vx,vy=translation_velocity(self.speed,err) if translation else (self.speed,0.0)
        if translation:yaw_rate=0.0
        yaw_cmd = float(yaw_rate)
        fallback = False
        self.fallback_reason = ""
        # Even reduced/diff_drive mode must satisfy the actual four-steering
        # mechanism. It must never bypass wheel feedback or native limits.
        valid, reason = self._chassis_command_valid(vx, yaw_rate, now, vy)
        if not valid:
            self._zero('CHASSIS_KINEMATICS_BLOCKED', reason)
            return
        self.fallback_active = fallback
        self.cmd_linear_x = float(vx)
        self.cmd_linear_y = float(vy)
        self.turn_recovery_until=0.
        self.cmd_angular_z = float(yaw_cmd)
        if self.dry_run:
            self.state = "DRY_RUN"
            self._status("DRY_RUN", f"would cmd v={self.speed:.3f} w={yaw_cmd:.3f}")
            return
        cmd = Twist()
        cmd.linear.x = self.cmd_linear_x
        cmd.linear.y = self.cmd_linear_y
        cmd.angular.z = self.cmd_angular_z
        self.cmd_pub.publish(cmd)
        self.state = "MOVING"
        self._status("MOVING", f"dist={dist:.2f} goal={goal_dist:.2f}")


def main(args=None) -> None:
    from .runtime_chassis_model import configure_chassis_transport
    configure_chassis_transport()
    rclpy.init(args=args)
    node = SlowMotionExecutor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble may raise a pybind subscription conversion error while SIGINT
        # tears down the context. Real runtime faults still propagate after zero.
        if rclpy.ok():
            raise
    finally:
        try:
            if not node.manual_override:
                node.cmd_pub.publish(Twist())
        except Exception:
            pass
        if node.command_observer.poll() is None:
            node.command_observer.terminate()
            try:
                node.command_observer.wait(timeout=2)
            except subprocess.TimeoutExpired:
                node.command_observer.kill()
                node.command_observer.wait()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
