"""FAST-LIO2 -> N-01 localization contract adapter.

FAST-LIO2 publishes its own map->imu_link odometry and does not publish TF.
This adapter composes that pose with the existing static TF chain
(imu_link -> base_footprint) and publishes the N-01 localization contract:
  /robot_pose (map -> base_footprint)
  /loc_status (synthetic tracking status while FAST-LIO is running)
  map -> base_footprint TF
It never broadcasts map->imu_link, so the existing TF tree keeps one parent.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

try:
    from hivecore_slam3d_lite_localization.msg import LocStatus
except Exception:  # pragma: no cover - build environment without vendor msgs
    LocStatus = None


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _qrot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def _qnorm(q):
    n = math.sqrt(sum(v * v for v in q))
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / n for v in q)


class FastLioPoseAdapter(Node):
    def __init__(self) -> None:
        super().__init__("n01_fast_lio_pose_adapter")
        self.declare_parameter("input_topic", "/n01/fast_lio/odometry")
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("status_topic", "/loc_status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("fastlio_map_frame", "n01_fast_lio_map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("alignment_topic", "/n01/fast_lio/map_alignment")
        self.declare_parameter("require_alignment", True)
        self.declare_parameter("publish_tf", True)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.fastlio_map_frame = str(self.get_parameter("fastlio_map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.require_alignment = bool(self.get_parameter("require_alignment").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.alignment = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pose_pub = self.create_publisher(
            Odometry, str(self.get_parameter("pose_topic").value), 10)
        self.status_pub = None
        if LocStatus is not None:
            self.status_pub = self.create_publisher(
                LocStatus, str(self.get_parameter("status_topic").value), 10)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        self.sub = self.create_subscription(
            Odometry, str(self.get_parameter("input_topic").value),
            self._on_odometry, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("alignment_topic").value),
            self._on_alignment, 10)
        self.received = 0
        self.published = 0
        self.tf_misses = 0
        self.get_logger().warn(
            "FAST-LIO2 N-01 adapter: map->base_footprint composition; "
            "synthetic tracking status; no relocalization")

    def _compose(self, msg):
        if msg.header.frame_id.lstrip("/") != self.fastlio_map_frame:
            self.get_logger().warn(
                f"FAST-LIO odometry frame is {msg.header.frame_id!r}, expected {self.fastlio_map_frame!r}",
                throttle_duration_sec=5.0)
            return None
        child = msg.child_frame_id.lstrip("/")
        if child != self.imu_frame:
            self.get_logger().warn(
                f"FAST-LIO child frame is {child!r}, expected {self.imu_frame!r}",
                throttle_duration_sec=5.0)
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                self.imu_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            self.tf_misses += 1
            self.get_logger().warn(
                f"waiting for TF {self.base_frame}->{self.imu_frame}: {type(exc).__name__}: {exc}",
                throttle_duration_sec=2.0)
            return None
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        q_map_imu = _qnorm((q.x, q.y, q.z, q.w))
        p_map_imu = (p.x, p.y, p.z)
        tr = t.transform.translation
        rq = t.transform.rotation
        p_imu_base = (tr.x, tr.y, tr.z)
        q_imu_base = _qnorm((rq.x, rq.y, rq.z, rq.w))
        rotated = _qrot(q_map_imu, p_imu_base)
        p_base = (p_map_imu[0] + rotated[0],
                  p_map_imu[1] + rotated[1],
                  p_map_imu[2] + rotated[2])
        q_base = _qnorm(_qmul(q_map_imu, q_imu_base))
        return p_base, q_base, msg

    def _on_alignment(self, msg: PoseStamped) -> None:
        if msg.header.frame_id.lstrip("/") != self.map_frame:
            self.get_logger().warn(
                f"alignment frame is {msg.header.frame_id!r}, expected {self.map_frame!r}",
                throttle_duration_sec=5.0)
            return
        p = msg.pose.position
        q = msg.pose.orientation
        self.alignment = ((p.x, p.y, p.z), _qnorm((q.x, q.y, q.z, q.w)))

    def _publish_status(self, session: str, reloc_ok: bool, fitness: float) -> None:
        if self.status_pub is None:
            return
        status = LocStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.map_frame
        status.session = session
        status.map_loaded = True
        status.reloc_ok = reloc_ok
        status.reloc_fitness = float(fitness)
        self.status_pub.publish(status)

    def _on_odometry(self, msg: Odometry) -> None:
        self.received += 1
        composed = self._compose(msg)
        if composed is None:
            self._publish_status("relocalization", False, 0.0)
            return
        p_fast, q_fast, src = composed
        if self.require_alignment and self.alignment is None:
            self._publish_status("relocalization", False, 0.0)
            return
        if self.alignment is not None:
            p_align, q_align = self.alignment
            rotated = _qrot(q_align, p_fast)
            p_base = (p_align[0] + rotated[0],
                      p_align[1] + rotated[1],
                      p_align[2] + rotated[2])
            q_base = _qnorm(_qmul(q_align, q_fast))
        else:
            p_base, q_base = p_fast, q_fast
        out = Odometry()
        out.header.stamp = src.header.stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = p_base[0]
        out.pose.pose.position.y = p_base[1]
        out.pose.pose.position.z = p_base[2]
        out.pose.pose.orientation.x = q_base[0]
        out.pose.pose.orientation.y = q_base[1]
        out.pose.pose.orientation.z = q_base[2]
        out.pose.pose.orientation.w = q_base[3]
        out.pose.covariance = src.pose.covariance
        out.twist = src.twist
        self.pose_pub.publish(out)
        self.published += 1
        if self.tf_pub is not None:
            tf = TransformStamped()
            tf.header.stamp = src.header.stamp
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = p_base[0]
            tf.transform.translation.y = p_base[1]
            tf.transform.translation.z = p_base[2]
            tf.transform.rotation.x = q_base[0]
            tf.transform.rotation.y = q_base[1]
            tf.transform.rotation.z = q_base[2]
            tf.transform.rotation.w = q_base[3]
            self.tf_pub.sendTransform(tf)
        self._publish_status("tracking", True, 0.0)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastLioPoseAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
