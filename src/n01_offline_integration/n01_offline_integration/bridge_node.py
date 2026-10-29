"""Gated read-only offline input bridge to the REAL SCAN planner.

Only isolated ROS_DOMAIN_ID=88 / ROS_LOCALHOST_ONLY=1 is accepted.
Never emits commands, uses sensor timestamps rather than latest-time TF.
"""
import json
import math
import os
import threading

if os.environ.get("ROS_DOMAIN_ID") != "88" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise RuntimeError("Offline bridge refuses non-isolated ROS domain")

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException
from rosidl_runtime_py.utilities import get_message

from .core import ReadinessGate, xyz_transform_bytes


def as_pose(tf):
    q, p = tf.rotation, tf.translation
    return ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def odom_from_tf(stamp, target_frame, child_frame, transform):
    position, quat = as_pose(transform)
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = target_frame
    message.child_frame_id = child_frame
    message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = position
    (message.pose.pose.orientation.x, message.pose.pose.orientation.y,
     message.pose.pose.orientation.z, message.pose.pose.orientation.w) = quat
    # Twist intentionally zero: bag TF does not determine actual velocity.
    # NEVER consume this fake-zero twist as real robot motion state.
    return message


class OfflineBridge(Node):
    def __init__(self):
        super().__init__("n01_offline_sensor_bridge")
        def p(key, default):
            self.declare_parameter(key, default)
            return self.get_parameter(key).value
        self.fixture = bool(p("fixture_mode", False))
        if self.fixture and os.environ.get("N01_OFFLINE_FIXTURE") != "1":
            raise RuntimeError("Fixture readiness needs explicit N01_OFFLINE_FIXTURE=1")
        self.cloud_topic = str(p("input_cloud_topic", "/livox/lidar/geo_filtered"))
        self.input_frame = str(p("input_frame", "lidar_link"))
        self.target_frame = str(p("target_frame", "map"))
        self.body_frame = str(p("body_frame", "base_link"))
        self.status_topic = str(p("status_topic", "/localization_status"))
        self.status_type = str(p("status_type", ""))
        self.slam_rsp_disabled = bool(p("slam_rsp_disabled_confirmed", False))
        self.max_tf_wait_s = float(p("tf_timeout_s", 0.05))
        self.max_cloud_points = int(p("max_cloud_points", 300000))
        self.gate = ReadinessGate(
            timeout_s=float(p("status_timeout_s", 1.0)),
            outer_path=str(p("outer_path", "")),
            inner_path=str(p("inner_path", "")),
            tf_flag_path=str(p("tf_flag_path", "registration_output_tf_published")),
            outer_value=str(p("outer_value", "Localization")),
            inner_value=str(p("inner_value", "tracking")),
            fixture=self.fixture)
        if not (0.001 <= self.max_tf_wait_s <= .3 and
                100 <= self.max_cloud_points <= 1000000):
            raise ValueError("unacceptable TF timeout / cloud cap")
        self.lock = threading.RLock()
        self.tf_conflict = False
        self.motion_tf_seen = False
        self.cmd_seen = False
        self.status_sub = None
        self.stats = {"input_clouds":0, "output_clouds":0, "rejected":0,
                      "tf_failed":0, "malformed":0, "status_received":0,
                      "rejected_reasons":{}, "control_commands_published":0}
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf_static", self.observe_tf, static_qos)
        self.create_subscription(TFMessage, "/tf", self.observe_tf, 100)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud,
                                 qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, "/cloud", qos_profile_sensor_data)
        self.body_pub = self.create_publisher(Odometry, "/body_pose", qos_profile_sensor_data)
        self.sensor_pub = self.create_publisher(Odometry, "/sensor_pose", qos_profile_sensor_data)
        self.health_pub = self.create_publisher(String, "/n01/offline/readiness", 10)
        if self.fixture:
            self.status_sub = self.create_subscription(
                Bool, "/n01/offline/test/localization_ready", self.observe_status,
                qos_profile_sensor_data)
        elif self.status_type:
            self.create_status_sub(self.status_type)
        self.create_timer(1.0, self.tick)
        self.get_logger().warning("OFFLINE BRIDGE only, no /cmd_vel or chassis interface")

    def create_status_sub(self, name):
        try:
            cls = get_message(name)
            self.status_sub = self.create_subscription(
                cls, self.status_topic, self.observe_status, qos_profile_sensor_data)
            self.status_type = name
        except Exception as exc:
            self.get_logger().error("Unknown /localization_status type: %s" % exc)

    def observe_status(self, msg):
        with self.lock:
            self.stats["status_received"] += 1
            self.gate.observe(msg)

    def observe_tf(self, msg):
        with self.lock:
            for t in msg.transforms:
                if (t.header.frame_id.lstrip("/") == "base_footprint" and
                        t.child_frame_id.lstrip("/") == "base_link"):
                    z = t.transform.translation.z
                    if math.isfinite(z) and abs(z - 0.9) <= 0.02:
                        self.motion_tf_seen = True
                    else:
                        self.tf_conflict = True
                        self.get_logger().error("Wrong/duplicate base_link height %.4f" % z)

    def reject(self, reason):
        with self.lock:
            self.stats["rejected"] += 1
            reasons = self.stats["rejected_reasons"]
            reasons[reason] = reasons.get(reason, 0) + 1

    def on_cloud(self, msg):
        with self.lock:
            self.stats["input_clouds"] += 1
            ready, reason = self.gate.check()
            if not ready:
                self.reject(reason)
                return
            if self.cmd_seen or self.tf_conflict:
                self.reject("cmd_vel_or_height_conflict")
                return
            if not self.motion_tf_seen:
                self.reject("no_motion_tf_0_90")
                return
            if not self.fixture and not self.slam_rsp_disabled:
                self.reject("slam_rsp_disable_unverified")
                return
        if msg.header.frame_id.lstrip("/") != self.input_frame:
            self.reject("incorrect_cloud_frame")
            return
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            self.reject("missing_cloud_timestamp")
            return
        stamp = Time.from_msg(msg.header.stamp)
        try:
            tf_sensor = self.tf_buffer.lookup_transform(
                self.target_frame, self.input_frame, stamp,
                timeout=Duration(seconds=self.max_tf_wait_s))
            tf_body = self.tf_buffer.lookup_transform(
                self.target_frame, self.body_frame, stamp,
                timeout=Duration(seconds=self.max_tf_wait_s))
        except (TransformException, ValueError) as exc:
            with self.lock:
                self.stats["tf_failed"] += 1
            self.reject("sensor_stamp_tf_missing:" + type(exc).__name__)
            return
        try:
            translation, rotation = as_pose(tf_sensor.transform)
            fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
            converted = xyz_transform_bytes(
                msg.data, msg.width, msg.height, msg.point_step, msg.row_step,
                fields, msg.is_bigendian, translation, rotation, self.max_cloud_points)
        except (ValueError, OverflowError, TypeError) as exc:
            with self.lock:
                self.stats["malformed"] += 1
            self.reject("invalid_xyz_or_quaternion:" + type(exc).__name__)
            return
        output = PointCloud2()
        output.header = msg.header
        output.header.frame_id = self.target_frame
        output.height, output.width, output.fields = msg.height, msg.width, msg.fields
        output.is_bigendian, output.point_step, output.row_step = (
            msg.is_bigendian, msg.point_step, msg.row_step)
        output.data, output.is_dense = converted, msg.is_dense
        with self.lock:
            ready, _ = self.gate.check()
            if not ready or self.cmd_seen or self.tf_conflict:
                self.reject("gate_changed_during_transform")
                return
            self.sensor_pub.publish(odom_from_tf(
                msg.header.stamp, self.target_frame, self.input_frame, tf_sensor.transform))
            self.body_pub.publish(odom_from_tf(
                msg.header.stamp, self.target_frame, self.body_frame, tf_body.transform))
            self.cloud_pub.publish(output)
            self.stats["output_clouds"] += 1

    def tick(self):
        if not self.fixture and self.status_sub is None:
            try:
                for topic, types in self.get_topic_names_and_types():
                    if topic == self.status_topic and len(types) == 1:
                        self.create_status_sub(types[0])
                        break
            except Exception:
                pass
        try:
            nodes = self.get_publishers_info_by_topic("/cmd_vel")
            self.cmd_seen = self.cmd_seen or bool(nodes)
        except Exception:
            self.cmd_seen = True  # Fail closed
        with self.lock:
            ready, reason = self.gate.check()
            passable = (ready and self.motion_tf_seen and not self.tf_conflict and
                        not self.cmd_seen and (self.fixture or self.slam_rsp_disabled))
            result = dict(self.stats)
            result.update({"readiness": ("OFFLINE_FIXTURE_ONLY" if self.fixture and passable
                                        else "READY_FOR_OFFLINE_DATA" if passable else "NOT_READY"),
                           "reason": reason, "tf_conflict":self.tf_conflict,
                           "motion_tf_seen":self.motion_tf_seen, "cmd_vel_seen":self.cmd_seen,
                           "status_type":self.status_type, "cloud_in":self.cloud_topic,
                           "cloud_out_frame":self.target_frame})
        out=String()
        out.data=json.dumps(result, ensure_ascii=False, sort_keys=True)
        self.health_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OfflineBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
