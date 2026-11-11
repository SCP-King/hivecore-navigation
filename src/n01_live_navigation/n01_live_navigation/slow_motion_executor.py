"""Opt-in, very slow path executor for the N-01 chassis.

This is the only node in the N-01 stack that may publish /cmd_vel.  It is
disabled by default, requires an explicitly enabled process, a fresh local
path, a fresh body pose and a READY runtime supervisor.  It publishes zero on
every unsafe transition and on shutdown.
"""
from __future__ import annotations

import json
import math
import time
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8
from sensor_msgs.msg import PointCloud2, JointState
from .core import decode_pointcloud_xyz
from .cbf_velocity import project_speed
from .hive2_kinematics import inverse, MODEL_ID
from .path_select import select_source

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
        self.declare_parameter("enabled", False)
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
        self.declare_parameter("control_mode", "auto")
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
        self.declare_parameter("path_source", "auto")
        self.declare_parameter("global_path_timeout_s", 30.0)
        self.declare_parameter("unified_grace_s", 1.0)
        get = lambda name: self.get_parameter(name).value
        self.enabled = bool(get("enabled"))
        self.control_mode = str(get("control_mode")).lower()
        if self.control_mode not in ("auto", "four_wis", "diff_drive"):
            raise ValueError("control_mode must be auto, four_wis or diff_drive")
        self.dry_run = bool(get("dry_run"))
        self.path_source = str(get("path_source")).lower()
        if self.path_source not in ("unified", "global", "auto"):
            raise ValueError("path_source must be unified, global or auto")
        self.global_path_timeout = max(0.5, min(120.0, float(get("global_path_timeout_s"))))
        self.unified_grace = max(0.2, min(5.0, float(get("unified_grace_s"))))
        self.max_speed = min(0.08, max(0.01, float(get("max_speed_mps"))))
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
        self.steering_feedback = None
        self.steering_feedback_rx = 0.0
        self.fallback_active = False
        self.fallback_reason = ""
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

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
        self.dec_stop = 0
        self.speed = 0.0
        self.state = "DISABLED"

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(Path, PATH_TOPIC, self._on_path, q)
        global_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/planning/global_path',
                                 self._on_global_path, global_qos)
        self.create_subscription(Odometry, POSE_TOPIC, self._on_pose, q)
        self.create_subscription(String, READINESS_TOPIC, self._on_readiness, q)
        self.create_subscription(Bool, ESTOP_TOPIC, self._on_estop, q)
        self.create_subscription(UInt8, DEC_STOP_STATE_TOPIC, self._on_dec_stop, q)
        self.create_subscription(Bool, '/n01/slow_motion/enable', self._on_enable, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, q)
        self.create_subscription(String, '/n01/live_navigation/hive2_feasibility', self._on_feasibility, q)
        self.create_subscription(String, '/n01/navigation/path_status', self._on_path_status, q)
        self.create_subscription(PointCloud2, '/cloud', self._on_obstacle_cloud, q)
        self.create_subscription(String, '/n01/live_navigation/dynamic_obstacle_status', self._on_tracks, q)
        self.create_subscription(JointState, '/joint_states', self._on_steering, q)
        self.create_timer(self.period, self._control)
        self.get_logger().warn(
            "SLOW MOTION EXECUTOR: %s; max_speed=%.3f m/s; path_source=%s; control_mode=%s; "
            "publishes /cmd_vel only when enabled"
            % ("ENABLED" if self.enabled else "DISABLED", self.max_speed,
               self.path_source, self.control_mode))

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
        self.steering_feedback = None
        age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if not -.2 <= age <= .5:
            return
        names=[f'{leg}_ankle_yaw' for leg in ('fl','fr','rl','rr')]
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

    def _chassis_command_valid(self, speed, yaw_rate, now):
        if self.steering_feedback is None or now-self.steering_feedback_rx>.5:
            return False,'fresh steering positions AND velocities required'
        angles,rates=self.steering_feedback
        try:
            targets,drive=inverse((speed,0.,yaw_rate),angles,rates)
        except ValueError as exc:
            return False,str(exc)
        if max(abs(v) for v in angles+list(targets))>1.5708+1e-6:
            return False,'URDF +/-90deg conservative steering bound exceeded'
        if max(abs(v) for v in drive)>20.+1e-6:
            return False,'native wheel speed exceeds 20rad/s'
        if max(abs(v) for v in rates)>1.57+1e-6:
            return False,'steering feedback exceeds 1.57rad/s'
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
                not -0.2 <= stamp_age <= self.pose_timeout):
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
        self.estop = bool(msg.data)

    def _on_enable(self, msg: Bool) -> None:
        requested = bool(msg.data)
        if requested and self.finished:
            return  # A completed bounded session requires a fresh process.
        if requested and not self.enabled:
            self.session_start = self.last_control = time.monotonic()
            self.travelled_m = self.commanded_m = 0.0
        self.enabled = requested
        if not requested:
            self._zero('DISABLED', 'operator disabled')

    def _on_dec_stop(self, msg: UInt8) -> None:
        self.dec_stop = int(msg.data)

    def _zero(self, state: str, reason: str = "") -> None:
        self.speed = 0.0
        self.state = state
        self.cmd_pub.publish(Twist())
        self._status(state, reason)

    def _status(self, state: str, reason: str = "") -> None:
        now = time.monotonic()
        payload = {
            "state": state,
            "reason": reason,
            "enabled": self.enabled,
            "speed_mps": round(self.speed, 4),
            "max_speed_mps": self.max_speed,
            "path_age_s": (None if not self.path_rx else round(now - self.path_rx, 3)),
            "pose_age_s": (None if not self.pose_rx else round(now - self.pose_rx, 3)),
            "estop": self.estop,
            "dec_stop": self.dec_stop,
            "travelled_m": round(self.travelled_m, 4),
            "commanded_m": round(self.commanded_m, 4),
            "distance_limit_m": self.distance_limit,
            "cbf_status": self.cbf_state,
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
        }
        self.status_pub.publish(String(data=json.dumps(payload)))

    def _control(self) -> None:
        now = time.monotonic()
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
        if self.dec_stop:
            self._zero("ESTOP", "/chassis_dec_stop/state")
            return
        if self.pose is None or now - self.pose_rx > self.pose_timeout:
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
        sel = select_source(
            path_source=self.path_source, now=now,
            path=self.path, path_rx=self.path_rx, path_timeout=self.path_timeout,
            path_status=self.path_status, path_status_rx=self.path_status_rx,
            path_valid_at=self.path_valid_at, grace_s=self.unified_grace,
            global_path=self.global_path, global_path_rx=self.global_path_rx,
            global_timeout_s=self.global_path_timeout, pose=self.pose)
        if sel is None:
            global_age = (f"{now - self.global_path_rx:.1f}s"
                          if self.global_path is not None else "missing")
            self._zero("NO_PATH",
                       f"no followable path (source={self.path_source}, "
                       f"global_age={global_age})")
            return
        source, poses, prefix, _path_stamp = sel
        self.path_source_used = source
        # 局部候选复核只在“当前有效 unified 路径”上强制：它是 local SCAN/CBF
        # 轨迹的可行性证据。grace / global 回退是操作员显式接受的低速应急模式，
        # CBF、pose、readiness、estop 等硬门仍全部生效（见下方）。
        if source == "unified" and (
                not isinstance(self.feasibility, dict) or now-self.feasibility_rx > 0.6 or
                self.feasibility.get('candidate_review_ready') is not True or
                self.feasibility.get('trajectory_active') is not True):
            self._zero("NO_VALID_TRAJECTORY", "candidate/feedback not ready")
            return
        full_len = len(poses)
        if full_len < 2:
            self._zero("NO_PATH", "path has <2 poses")
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
        if goal_dist < 0.10:
            self.finished = True
            self.completion_state = 'ARRIVED'
            self._zero("ARRIVED", f"goal_dist={goal_dist:.3f}")
            return
        prefix_end = poses[-1].pose.position
        if (source != "global" and math.hypot(prefix_end.x-x,prefix_end.y-y)<.20
                and prefix<full_len):
            self._zero('WAITING_FOR_PATH_EXTENSION', 'validated prefix exhausted')
            return
        err = _wrap(math.atan2(ty - y, tx - x) - yaw)
        if abs(err) > 1.0:
            self._zero("ALIGN", f"heading_err={err:.2f}")
            return
        speed_target = self.max_speed * max(0.0, 1.0 - abs(err) / 0.8)
        speed_target = min(speed_target, self.max_speed * max(0.0, goal_dist / 0.5))
        accel_step = self.max_accel * self.period
        self.speed += max(-accel_step, min(accel_step, speed_target - self.speed))
        yaw_rate = max(-self.max_yaw, min(self.max_yaw, 1.2 * err))
        if self.obstacle_cloud is None or now-self.cloud_rx>.6 or self.tracks is None or now-self.tracks_rx>.6:
            self._zero('CBF_SENSOR_STALE', 'fresh cloud and dynamic snapshot required')
            return
        xyz = self.obstacle_cloud
        body_z = self.pose_z
        ground = body_z-.778
        mask = ((xyz[:,2]>ground+.08)&(xyz[:,2]<ground+1.4)&
                (np.hypot(xyz[:,0]-x,xyz[:,1]-y)<3.0))
        obstacles = [(float(p[0]),float(p[1]),0.,0.,0.) for p in xyz[mask]]
        for track in self.tracks:
            center = track.get('centroid_map', [])
            velocity = track.get('velocity_map_mps', [])
            if len(center)<2 or len(velocity)<2:
                self._zero('CBF_INVALID_TRACK', 'invalid track geometry'); return
            valid_velocity = track.get('velocity_valid') is True
            vx,vy = velocity[:2] if valid_velocity else (0.,0.)
            obstacles.append((center[0],center[1],vx,vy,.5 if valid_velocity else .75))
        self.speed, self.cbf_state = project_speed((x,y),yaw,self.speed,obstacles)
        if self.speed<=1e-6:
            self._zero(self.cbf_state, 'CBF constrained to stop')
            return
        yaw_cmd = float(yaw_rate)
        fallback = False
        self.fallback_reason = ""
        if self.control_mode == "diff_drive":
            fallback = True
            self.fallback_reason = "diff_drive mode"
        else:
            valid, reason = self._chassis_command_valid(self.speed, yaw_rate, now)
            if not valid:
                if self.control_mode == "four_wis":
                    self._zero('CHASSIS_KINEMATICS_BLOCKED', reason)
                    return
                # auto fallback: the 4WIS IK branch from the current wheel
                # state can exceed the +/-90 deg bound at very low speed.  Fall
                # back to a differential-drive twist whose yaw command scales
                # with speed, so the chassis is not asked for a tight turn
                # before it has started moving.
                fallback = True
                self.fallback_reason = reason
        if fallback:
            yaw_cmd = yaw_rate * min(1.0, self.speed / max(self.max_speed, 1e-3))
        self.fallback_active = fallback
        self.cmd_linear_x = float(self.speed)
        self.cmd_angular_z = float(yaw_cmd)
        if self.dry_run:
            self.state = "DRY_RUN"
            self._status("DRY_RUN", f"would cmd v={self.speed:.3f} w={yaw_cmd:.3f}")
            return
        cmd = Twist()
        cmd.linear.x = self.cmd_linear_x
        cmd.angular.z = self.cmd_angular_z
        self.cmd_pub.publish(cmd)
        self.state = "MOVING"
        self._status("MOVING", f"dist={dist:.2f} goal={goal_dist:.2f}")


def main(args=None) -> None:
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
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
