"""Saved-map relocalization for FAST-LIO2 using Open3D ICP.

The operator supplies a global initial pose (/initialpose, map frame).  The
node aligns the live FAST-LIO body cloud to the saved PCD map and publishes
the map -> n01_fast_lio_map alignment consumed by n01_fast_lio_pose_adapter.
It never publishes TF or /robot_pose itself.
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _qnorm(q):
    n = math.sqrt(sum(v * v for v in q))
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / n for v in q)


def _qmat(q):
    x, y, z, w = _qnorm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def _mat4(p, q):
    t = np.eye(4)
    t[:3, :3] = _qmat(q)
    t[:3, 3] = p
    return t


def _pose_mat(msg):
    pose = getattr(msg, "pose", msg)
    if hasattr(pose, "pose"):
        pose = pose.pose
    p = pose.position
    q = pose.orientation
    return _mat4((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def _transform_msg(t):
    tr = t.transform.translation
    q = t.transform.rotation
    return _mat4((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))


def _mat_pose(m):
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in m[:3, 3])
    # quaternion from rotation matrix
    r = m[:3, :3]
    tr = np.trace(r)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = _qnorm((x, y, z, w))
    return pose


class FastLioRelocalizer(Node):
    def __init__(self) -> None:
        super().__init__("n01_fast_lio_relocalizer")
        self.declare_parameter("map_file_path", "/home/demo/lyy/slam_data/maps/mapping_lite_20261109_140536_087.pcd")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("fastlio_map_frame", "n01_fast_lio_map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("cloud_topic", "/n01/fast_lio/cloud_registered_body")
        self.declare_parameter("odom_topic", "/n01/fast_lio/odometry")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("alignment_topic", "/n01/fast_lio/map_alignment")
        self.declare_parameter("status_topic", "/n01/fast_lio/relocalization_status")
        self.declare_parameter("reference_map_topic", "/n01/fast_lio/reference_map")
        self.declare_parameter("voxel_m", 0.2)
        self.declare_parameter("max_correspondence_m", 0.6)
        self.declare_parameter("max_iterations", 30)
        self.declare_parameter("min_fitness", 0.30)
        self.declare_parameter("icp_period_s", 1.0)
        self.declare_parameter("max_points", 120000)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.fastlio_map_frame = str(self.get_parameter("fastlio_map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.voxel = float(self.get_parameter("voxel_m").value)
        self.max_corr = float(self.get_parameter("max_correspondence_m").value)
        self.max_iter = int(self.get_parameter("max_iterations").value)
        self.min_fitness = float(self.get_parameter("min_fitness").value)
        self.icp_period = float(self.get_parameter("icp_period_s").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.map_cloud = o3d.geometry.PointCloud()
        self.map_loaded = self._load_map(str(self.get_parameter("map_file_path").value))
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.odom = None
        self.odom_rx = 0.0
        self.cloud = None
        self.cloud_rx = 0.0
        self.has_initial = False
        self.T_map_fastlio = None
        self.last_icp = 0.0
        self.valid = False
        self.fitness = 0.0
        self.rmse = float("inf")
        self.alignment_pub = self.create_publisher(PoseStamped, str(self.get_parameter("alignment_topic").value), 10)
        self.status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference_map_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("reference_map_topic").value), map_qos)
        if self.map_loaded:
            header = Header()
            header.frame_id = self.map_frame
            reference = point_cloud2.create_cloud_xyz32(
                header, np.asarray(self.map_cloud.points, dtype=np.float32).tolist())
            self.reference_map_pub.publish(reference)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10)
        self.create_subscription(PointCloud2, str(self.get_parameter("cloud_topic").value), self._on_cloud, 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 str(self.get_parameter("initialpose_topic").value), self._on_initialpose, 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().warn(
            "FAST-LIO relocalizer: saved PCD + /initialpose + Open3D ICP; "
            f"map_loaded={self.map_loaded}")

    def _load_map(self, path):
        try:
            pcd = o3d.io.read_point_cloud(path)
            if not pcd.has_points():
                self.get_logger().error(f"map PCD has no points: {path}")
                return False
            pcd = pcd.voxel_down_sample(self.voxel)
            pts = np.asarray(pcd.points)
            self.map_cloud.points = o3d.utility.Vector3dVector(pts)
            self.get_logger().info(f"loaded saved map {path}: {len(pts)} points after voxel {self.voxel}")
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"cannot load map {path}: {type(exc).__name__}: {exc}")
            return False

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        self.odom_rx = time.monotonic()

    def _on_cloud(self, msg: PointCloud2) -> None:
        self.cloud = msg
        self.cloud_rx = time.monotonic()

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        if msg.header.frame_id.lstrip("/") != self.map_frame:
            self.get_logger().warn(f"initialpose frame {msg.header.frame_id!r} != {self.map_frame!r}")
            return
        if self.odom is None:
            self.get_logger().warn("initialpose received before FAST-LIO odometry")
            return
        try:
            t_imu_base = self.tf_buffer.lookup_transform(
                self.imu_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"initialpose: missing TF imu<-base: {exc}", throttle_duration_sec=2.0)
            return
        T_map_base_init = _pose_mat(msg.pose)
        T_fastlio_imu = _pose_mat(self.odom.pose)
        T_imu_base = _transform_msg(t_imu_base)
        T_fastlio_base = T_fastlio_imu @ T_imu_base
        self.T_map_fastlio = T_map_base_init @ np.linalg.inv(T_fastlio_base)
        self.has_initial = True
        self.valid = False
        self._status("ICP_PENDING", "initialpose accepted; waiting for ICP")
        self._try_icp(force=True)

    def _publish_alignment(self) -> None:
        if self.T_map_fastlio is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose = _mat_pose(self.T_map_fastlio)
        self.alignment_pub.publish(msg)

    def _status(self, state, reason="") -> None:
        payload = {
            "state": state,
            "reason": reason,
            "map_loaded": self.map_loaded,
            "has_initialpose": self.has_initial,
            "valid": self.valid,
            "fitness": round(self.fitness, 4),
            "inlier_rmse": (None if not math.isfinite(self.rmse) else round(self.rmse, 4)),
        }
        self.status_pub.publish(String(data=json.dumps(payload)))

    def _try_icp(self, force=False) -> None:
        now = time.monotonic()
        if not force and now - self.last_icp < self.icp_period:
            return
        self.last_icp = now
        if not self.map_loaded or not self.has_initial:
            self._status("WAITING_MAP_OR_INITIALPOSE")
            return
        if self.cloud is None or now - self.cloud_rx > 1.5:
            self._status("WAITING_CLOUD")
            return
        if self.odom is None or now - self.odom_rx > 1.0:
            self._status("WAITING_ODOM")
            return
        try:
            t_imu_base = self.tf_buffer.lookup_transform(
                self.imu_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            self._status("WAITING_TF", str(exc))
            return
        T_fastlio_imu = _pose_mat(self.odom.pose)
        T_imu_base = _transform_msg(t_imu_base)
        T_map_imu = self.T_map_fastlio @ T_fastlio_imu
        pts = []
        for p in point_cloud2.read_points(self.cloud, field_names=("x", "y", "z"), skip_nans=True):
            pts.append((float(p[0]), float(p[1]), float(p[2])))
        if len(pts) < 50:
            self._status("WAITING_CLOUD_POINTS", f"points={len(pts)}")
            return
        pts = np.asarray(pts, dtype=float)
        if len(pts) > self.max_points:
            pts = pts[::max(1, len(pts) // self.max_points)]
        r = T_map_imu[:3, :3]
        t = T_map_imu[:3, 3]
        pts_map = pts @ r.T + t
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(pts_map)
        source = source.voxel_down_sample(self.voxel)
        reg = o3d.pipelines.registration.registration_icp(
            source, self.map_cloud, self.max_corr, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.max_iter))
        self.fitness = float(reg.fitness)
        self.rmse = float(reg.inlier_rmse)
        if self.fitness < self.min_fitness:
            self.valid = False
            self._status("ICP_REJECTED", f"fitness={self.fitness:.3f} < {self.min_fitness:.3f}")
            return
        T_map_imu_corrected = reg.transformation @ T_map_imu
        self.T_map_fastlio = T_map_imu_corrected @ np.linalg.inv(T_fastlio_imu)
        self.valid = True
        self._publish_alignment()
        self._status("ICP_OK", f"fitness={self.fitness:.3f}")

    def _tick(self) -> None:
        self._try_icp()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastLioRelocalizer()
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
