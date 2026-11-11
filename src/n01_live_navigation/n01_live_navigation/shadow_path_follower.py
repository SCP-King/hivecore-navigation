"""Shadow path follower: candidate speed/steer only, never actuator output.

It consumes the live local candidate path, body pose and dynamic obstacle
candidates. It computes a pure-pursuit lookahead command and a conservative
CBF-style speed screen, then publishes a JSON status. It deliberately has no
/cmd_vel publisher and no chassis command path.
"""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from .core import decode_pointcloud_xyz

SCHEMA = "n01.shadow_path_follower.v1"
PATH_TOPIC = "/n01/live_navigation/hive2_candidate_path"
CANDIDATE_TOPIC = "/n01/live_navigation/dynamic_obstacle_candidates"
HEALTH_TOPIC = "/n01/live_navigation/system_health"
STATUS_TOPIC = "/n01/live_navigation/shadow_path_follower/status"


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _path_points(msg: Path):
    return [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]


def lookahead_target(points, pose, lookahead_m):
    """Return (target_xy, distance_to_target) along a polyline."""
    if not points:
        return None, 0.0
    if len(points) == 1:
        return points[0], math.hypot(points[0][0] - pose[0], points[0][1] - pose[1])
    # nearest vertex, then walk forward along the polyline
    dists = [math.hypot(x - pose[0], y - pose[1]) for x, y in points]
    i0 = int(min(range(len(points)), key=lambda i: dists[i]))
    accumulated = 0.0
    for i in range(i0, len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg <= 1e-6:
            continue
        if accumulated + seg >= lookahead_m:
            ratio = (lookahead_m - accumulated) / seg
            target = (x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0))
            return target, math.hypot(target[0] - pose[0], target[1] - pose[1])
        accumulated += seg
    target = points[-1]
    return target, math.hypot(target[0] - pose[0], target[1] - pose[1])


def cbf_speed_limit(distance_m, safe_radius_m, decel_mps2, desired_speed_mps):
    """Conservative 1-D CBF speed barrier: v^2 <= 2 a (d - safe)."""
    margin = float(distance_m) - float(safe_radius_m)
    if margin <= 0.0:
        return 0.0, margin
    limit = math.sqrt(max(0.0, 2.0 * float(decel_mps2) * margin))
    return min(float(desired_speed_mps), limit), margin


class ShadowPathFollower(Node):
    def __init__(self):
        super().__init__("n01_shadow_path_follower")
        self.declare_parameter("lookahead_m", 0.8)
        self.declare_parameter("max_speed_mps", 0.15)
        self.declare_parameter("wheel_base_m", 0.56)
        self.declare_parameter("path_timeout_s", 0.8)
        self.declare_parameter("cbf_safe_radius_m", 0.55)
        self.declare_parameter("cbf_decel_mps2", 0.20)
        self.declare_parameter("max_dynamic_points", 20000)
        self.lookahead = float(self.get_parameter("lookahead_m").value)
        self.max_speed = float(self.get_parameter("max_speed_mps").value)
        self.wheel_base = float(self.get_parameter("wheel_base_m").value)
        self.path_timeout = float(self.get_parameter("path_timeout_s").value)
        self.safe_radius = float(self.get_parameter("cbf_safe_radius_m").value)
        self.decel = float(self.get_parameter("cbf_decel_mps2").value)
        self.max_dynamic = int(self.get_parameter("max_dynamic_points").value)

        self.path_msg = None
        self.path_rx = 0.0
        self.body = None
        self.dynamic_points = np.empty((0, 3), dtype=float)
        self.health = {}

        self.create_subscription(Path, PATH_TOPIC, self.on_path, 10)
        self.create_subscription(Odometry, "/body_pose", self.on_body, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, CANDIDATE_TOPIC, self.on_candidates,
                                 qos_profile_sensor_data)
        self.create_subscription(String, HEALTH_TOPIC, self.on_health, 10)
        self.pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_timer(0.1, self.publish_status)

    def on_path(self, msg: Path):
        self.path_msg = msg
        self.path_rx = time.monotonic()

    def on_body(self, msg: Odometry):
        self.body = msg

    def on_candidates(self, msg: PointCloud2):
        try:
            pts = decode_pointcloud_xyz(msg, max_points=self.max_dynamic)
            self.dynamic_points = pts
        except Exception:
            self.dynamic_points = np.empty((0, 3), dtype=float)

    def on_health(self, msg: String):
        try:
            self.health = json.loads(msg.data)
        except Exception:
            self.health = {"raw": msg.data[:300]}

    def _path_age(self):
        if self.path_msg is None:
            return None
        stamp = self.path_msg.header.stamp
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if stamp_s <= 0.0:
            return None
        return max(0.0, time.time() - stamp_s)

    def _nearest_dynamic(self, pose_xy, yaw):
        if self.dynamic_points.size == 0:
            return None, None
        pts = np.asarray(self.dynamic_points)
        if pts.shape[1] < 2:
            return None, None
        dx = pts[:, 0] - pose_xy[0]
        dy = pts[:, 1] - pose_xy[1]
        dist = np.hypot(dx, dy)
        # Prefer obstacles in the forward corridor; fall back to nearest.
        forward = dx * math.cos(yaw) + dy * math.sin(yaw)
        lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
        mask = (forward > -0.2) & (np.abs(lateral) < 0.8)
        if np.any(mask):
            idx = int(np.argmin(np.where(mask, dist, np.inf)))
        else:
            idx = int(np.argmin(dist))
        return float(dist[idx]), float(forward[idx])

    def publish_status(self):
        now = time.monotonic()
        status = "WAITING_FOR_PATH"
        speed = 0.0
        yaw_rate = 0.0
        lookahead_target_pt = None
        nearest_d = None
        margin = None
        path_age = self._path_age()
        if self.path_msg is None or self.body is None:
            status = "WAITING_FOR_PATH_OR_POSE"
        elif path_age is None or path_age > self.path_timeout:
            status = "PATH_STALE"
        else:
            points = _path_points(self.path_msg)
            p = self.body.pose.pose.position
            yaw = yaw_from_quaternion(self.body.pose.pose.orientation)
            target, _ = lookahead_target(points, (p.x, p.y), self.lookahead)
            if target is not None:
                lookahead_target_pt = target
                alpha = math.atan2(target[1] - p.y, target[0] - p.x) - yaw
                alpha = math.atan2(math.sin(alpha), math.cos(alpha))
                speed = self.max_speed
                if abs(speed) > 1e-6:
                    yaw_rate = 2.0 * speed * math.sin(alpha) / max(self.wheel_base, 1e-3)
                nearest_d, _ = self._nearest_dynamic((p.x, p.y), yaw)
                if nearest_d is not None:
                    speed, margin = cbf_speed_limit(
                        nearest_d, self.safe_radius, self.decel, speed)
                    if margin <= 0.0:
                        status = "CBF_STOP"
                    elif speed < self.max_speed - 1e-6:
                        status = "CBF_LIMIT"
                    else:
                        status = "SHADOW_READY"
                else:
                    status = "SHADOW_READY"
        payload = {
            "schema": SCHEMA,
            "mode": "SHADOW",
            "status": status,
            "path_age_s": path_age,
            "candidate_speed_mps": round(float(speed), 4),
            "candidate_yaw_rate_radps": round(float(yaw_rate), 4),
            "lookahead_m": self.lookahead,
            "lookahead_target": None if lookahead_target_pt is None else [
                round(float(lookahead_target_pt[0]), 3),
                round(float(lookahead_target_pt[1]), 3)],
            "nearest_dynamic_obstacle_m": None if nearest_d is None else round(float(nearest_d), 3),
            "cbf_margin_m": None if margin is None else round(float(margin), 3),
            "cbf_safe_radius_m": self.safe_radius,
            "cbf_decel_mps2": self.decel,
            "actuator_output": "DISABLED",
            "execution_authorized": False,
            "safety_authorization": "NEVER",
            "health_status": self.health.get("status"),
            "recv_path": self.path_rx > 0.0,
            "source": PATH_TOPIC,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShadowPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
