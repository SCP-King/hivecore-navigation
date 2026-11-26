"""Backend-neutral N-01 localization adapter.

Discovers the external status message type at runtime and maps only explicitly
configured field paths. Unknown or unmapped status fields always fail closed.
"""
import json
import math
import time
import copy

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from .core import invert_transform, mapping_navigation_status, localization_is_tracking
from .pose_frame import change_pose_child, change_twist_child
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def resolve_path(message, path):
    if not path:
        raise ValueError("field_path_not_configured")
    value = message
    for part in path.split("."):
        if not hasattr(value, part):
            raise ValueError("field_path_missing:" + path)
        value = getattr(value, part)
    return value


def strict_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "valid", "ok"):
            return True
        if normalized in ("false", "0", "no", "invalid", "error"):
            return False
    raise ValueError("boolean_value_unrecognized")

 
def pose_contract_issues(message, expected_frame, expected_child_frame):
    """Diagnose passthrough data, or validate source semantics before conversion."""
    issues = []
    stamp_ns = int(message.header.stamp.sec) * 1000000000 + int(message.header.stamp.nanosec)
    frame = str(message.header.frame_id).lstrip("/")
    child = str(message.child_frame_id).lstrip("/")
    if stamp_ns <= 0:
        issues.append("zero_or_invalid_stamp")
    if frame != expected_frame:
        issues.append("unexpected_frame_id:" + frame)
    if child != expected_child_frame:
        issues.append("unexpected_child_frame_id:" + child)
    return issues


class N01LocalizationAdapter(Node):
    def __init__(self):
        super().__init__("n01_localization_adapter")
        defaults = {
            "status_forwarding_enabled": True,
            "status_mode": "localization",
            "status_input_topic": "/loc_status",
            "status_output_topic": "/n01/localization/status",
            "status_header_path": "header",
            "status_stamp_path": "header.stamp",
            "status_field_paths_json": '{"session":"session","map_loaded":"map_loaded","reloc_ok":"reloc_ok","reloc_fitness":"reloc_fitness"}',
            "status_expected_values_json": '{"session":"tracking"}',
            "status_discovery_period_s": 1.0,
            "pose_input_topic": "/robot_pose",
            "pose_output_topic": "/n01/localization/pose",
            "expected_pose_frame": "map",
            "expected_pose_child_frame": "base_link",
            "output_pose_child_frame": "",
            "pose_mirror_topic": "",
            "broadcast_pose_tf": False,
            "broadcast_input_pose_tf": False,
            "initial_pose_input_topic": "",
            "initial_pose_output_topic": "",
            "initial_pose_preserve_height": False,
            "pose_qos_depth": 50,
            "pose_stats_period_s": 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.status_forwarding_enabled = bool(get('status_forwarding_enabled'))
        self.status_mode = str(get('status_mode'))
        if self.status_mode not in ('localization', 'mapping'):
            raise ValueError('unknown_status_mode')
        self.status_input_topic = str(get("status_input_topic"))
        self.status_output_topic = str(get("status_output_topic"))
        self.status_header_path = str(get("status_header_path"))
        self.status_stamp_path = str(get("status_stamp_path"))
        self.field_paths = json.loads(str(get("status_field_paths_json")))
        self.expected_values = json.loads(str(get("status_expected_values_json")))
        self.pose_input_topic = str(get("pose_input_topic"))
        self.pose_output_topic = str(get("pose_output_topic"))
        self.expected_pose_frame = str(get("expected_pose_frame")).lstrip("/")
        self.expected_pose_child_frame = str(get("expected_pose_child_frame")).lstrip("/")
        self.output_child = str(get("output_pose_child_frame")).lstrip("/")
        self.mirror_topic = str(get("pose_mirror_topic"))
        self.broadcast_pose_tf = bool(get("broadcast_pose_tf"))
        self.broadcast_input_pose_tf = bool(get("broadcast_input_pose_tf"))
        self.initial_preserve_height = bool(get('initial_pose_preserve_height'))
        self.last_output_height = None
        self.last_output_stamp_ns = 0
        self.last_output_receive_time = 0.0
        pose_qos_depth = int(get("pose_qos_depth"))
        pose_stats_period_s = float(get("pose_stats_period_s"))
        period = float(get("status_discovery_period_s"))
        if not all((self.status_input_topic.startswith("/"),
                    self.status_output_topic.startswith("/"),
                    self.pose_input_topic.startswith("/"),
                    self.pose_output_topic.startswith("/"),
                    0.2 <= period <= 5.0,
                    1 <= pose_qos_depth <= 1000,
                    1.0 <= pose_stats_period_s <= 60.0,
                    isinstance(self.field_paths, dict),
                    isinstance(self.expected_values, dict))):
            raise ValueError("invalid localization adapter configuration")

        # The observed Lite Odometry publisher offers RELIABLE QoS. Use a
        # reliable, deeper queue on both sides to avoid the shallow BEST_EFFORT
        # sensor-data queue silently losing pose samples under scheduling load.
        self.pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=pose_qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # The scan bridge subscribes with BEST_EFFORT; a RELIABLE publisher is
        # compatible with it and also lets normal CLI/rosbag observers receive.
        self.status_pub = self.create_publisher(
            DiagnosticArray, self.status_output_topic, 10)
        self.pose_pub = self.create_publisher(
            Odometry, self.pose_output_topic, self.pose_qos)
        self.pose_mirror = (self.create_publisher(Odometry, self.mirror_topic, self.pose_qos)
                            if self.mirror_topic else None)
        if self.mirror_topic == self.pose_input_topic:
            raise ValueError('pose_mirror_must_not_loop_back_to_input')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self) if self.broadcast_pose_tf else None
        self.input_tf_broadcaster = TransformBroadcaster(self) if self.broadcast_input_pose_tf else None
        initial_input, initial_output = str(get('initial_pose_input_topic')), str(get('initial_pose_output_topic'))
        if initial_input and initial_output:
            if initial_input == initial_output:
                raise ValueError('initialpose_must_not_loop_back_to_input')
            self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, initial_output, 10)
            self.initial_sub = self.create_subscription(PoseWithCovarianceStamped, initial_input, self.on_initial_pose, 10)
        self.pose_sub = self.create_subscription(
            Odometry, self.pose_input_topic, self.on_pose, self.pose_qos)
        self.status_sub = None
        self.status_type = ""
        self.status_counts = {"received": 0, "published": 0, "mapping_errors": 0}
        self.pose_counts = {
            "received": 0, "published": 0, "invalid_stamp": 0,
            "unexpected_frame": 0, "unexpected_child_frame": 0,
        }
        self.create_timer(period, self.discover_status_type)
        self.create_timer(pose_stats_period_s, self.report_pose_stats)
        self.create_timer(pose_stats_period_s, self.report_status_stats)
        self.get_logger().info(
            "N-01 backend adapter: status %s -> %s; pose %s -> %s"
            % (self.status_input_topic, self.status_output_topic,
               self.pose_input_topic, self.pose_output_topic))
        if self.status_mode == 'mapping':
            self.get_logger().info('Mapping session and normalized pose readiness; no relocation/fitness inferred.')
        elif self.status_forwarding_enabled:
            self.get_logger().warning(
                "Verified LocStatus field mapping loaded; unknown fields fail closed. "
                "No absent TF, IMU or general quality fields are inferred.")
        else:
            self.get_logger().info("Mapping pose adapter: localization readiness forwarding disabled.")

    def discover_status_type(self):
        if not self.status_forwarding_enabled:
            return
        matches = [types for name, types in self.get_topic_names_and_types()
                   if name == self.status_input_topic]
        if not matches or len(matches[0]) != 1:
            return
        type_name = matches[0][0]
        if type_name == self.status_type and self.status_sub is not None:
            return
        try:
            message_class = get_message(type_name)
            if self.status_sub is not None:
                self.destroy_subscription(self.status_sub)
            self.status_sub = self.create_subscription(
                message_class, self.status_input_topic, self.on_status,
                qos_profile_sensor_data)
            self.status_type = type_name
            self.get_logger().info(
                "Discovered external localization status type: %s" % type_name)
        except Exception as exc:
            self.status_sub = None
            self.status_type = ""
            self.get_logger().error("Cannot load status type %s: %s" % (type_name, exc))

    @staticmethod
    def stamp_ns(stamp):
        return int(stamp.sec) * 1000000000 + int(stamp.nanosec)

    def on_status(self, message):
        if self.status_mode == 'mapping':
            self.on_mapping_status(message)
            return
        self.status_counts["received"] += 1
        values = {}
        errors = []
        for key in ("session", "map_loaded", "reloc_ok", "reloc_fitness"):
            try:
                raw = resolve_path(message, self.field_paths.get(key, ""))
                if key in ("map_loaded", "reloc_ok"):
                    values[key] = "true" if strict_bool(raw) else "false"
                elif key == "session":
                    values[key] = str(raw).strip().lower()
                else:
                    fitness = float(raw)
                    if not math.isfinite(fitness):
                        raise ValueError("fitness_not_finite")
                    values[key] = repr(fitness)
            except (ValueError, TypeError, AttributeError) as exc:
                values[key] = ""
                errors.append("%s:%s" % (key, exc))
        expected_session = str(self.expected_values.get("session", "tracking")).lower()
        ready = (values["map_loaded"] == "true"
                 and values["session"] == expected_session
                 and values["reloc_ok"] == "true"
                 and not errors)
        try:
            stamp = resolve_path(message, self.status_stamp_path)
            source_stamp_ns = self.stamp_ns(stamp)
            if source_stamp_ns <= 0:
                raise ValueError("zero_status_stamp")
            header = resolve_path(message, self.status_header_path)
            frame_id = str(getattr(header, "frame_id", ""))
        except (ValueError, TypeError, AttributeError) as exc:
            source_stamp_ns = 0
            frame_id = ""
            ready = False
            errors.append("stamp:%s" % exc)
        values["backend_status_type"] = self.status_type
        values["status_mapping_valid"] = str(not errors).lower()
        values["status_mapping_errors"] = ";".join(errors) if errors else "none"
        values["source_stamp_valid"] = str(source_stamp_ns > 0).lower()
        values["readiness_semantics"] = "map_loaded_and_tracking_and_reloc_ok"
        output = DiagnosticArray()
        output.header.stamp.sec = source_stamp_ns // 1000000000
        output.header.stamp.nanosec = source_stamp_ns % 1000000000
        output.header.frame_id = frame_id
        status = DiagnosticStatus()
        status.name = "localization"
        status.level = DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR
        status.message = "tracking_ready" if ready else "localization_not_ready"
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        output.status = [status]
        self.status_pub.publish(output)
        self.status_counts["published"] += 1
        if errors:
            self.status_counts["mapping_errors"] += 1

    def on_mapping_status(self, message):
        self.status_counts['received'] += 1
        stamp = message.header.stamp
        values = mapping_navigation_status(
            message.session, self.stamp_ns(stamp), self.get_clock().now().nanoseconds,
            self.last_output_stamp_ns, time.monotonic() - self.last_output_receive_time)
        ready, reason = localization_is_tracking(values)
        status = DiagnosticStatus(name='localization',
                                  level=DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR,
                                  message=reason,
                                  values=[KeyValue(key=k, value=v) for k, v in values.items()])
        output = DiagnosticArray(header=copy.deepcopy(message.header), status=[status])
        self.status_pub.publish(output)
        self.status_counts['published'] += 1

    def on_pose(self, message):
        """Normalize the configured physical child; default mode remains passthrough."""
        self.pose_counts["received"] += 1
        issues = pose_contract_issues(
            message, self.expected_pose_frame, self.expected_pose_child_frame)
        for issue in issues:
            if issue == "zero_or_invalid_stamp":
                self.pose_counts["invalid_stamp"] += 1
            elif issue.startswith("unexpected_frame_id:"):
                self.pose_counts["unexpected_frame"] += 1
            elif issue.startswith("unexpected_child_frame_id:"):
                self.pose_counts["unexpected_child_frame"] += 1
        if self.input_tf_broadcaster and not issues:
            input_tf = TransformStamped()
            input_tf.header = copy.deepcopy(message.header)
            input_tf.child_frame_id = message.child_frame_id
            p = message.pose.pose.position
            q = message.pose.pose.orientation
            input_tf.transform.translation.x, input_tf.transform.translation.y, input_tf.transform.translation.z = p.x, p.y, p.z
            input_tf.transform.rotation = copy.deepcopy(q)
            self.input_tf_broadcaster.sendTransform(input_tf)
        if self.output_child and self.output_child != self.expected_pose_child_frame:
            if issues:
                return  # Cannot transform a sample with an unknown frame/time.
            try:
                mount = self.source_from_output()
                message = copy.deepcopy(message)
                self.convert_pose(message.pose, mount)
                v, w = message.twist.twist.linear, message.twist.twist.angular
                linear, angular, cov = change_twist_child(
                    (v.x,v.y,v.z), (w.x,w.y,w.z), message.twist.covariance, mount)
                v.x,v.y,v.z = linear
                w.x,w.y,w.z = angular
                message.twist.covariance = cov
                message.child_frame_id = self.output_child
            except Exception as exc:
                self.get_logger().warning('Pose frame conversion waiting: '+str(exc), throttle_duration_sec=5.)
                return
        self.pose_pub.publish(message)
        self.last_output_height = float(message.pose.pose.position.z)
        self.last_output_stamp_ns = self.stamp_ns(message.header.stamp)
        self.last_output_receive_time = time.monotonic()
        if self.pose_mirror:
            self.pose_mirror.publish(message)
        if self.tf_broadcaster:
            tf = TransformStamped()
            tf.header = copy.deepcopy(message.header)
            tf.child_frame_id = message.child_frame_id
            p = message.pose.pose.position
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z = p.x,p.y,p.z
            tf.transform.rotation = copy.deepcopy(message.pose.pose.orientation)
            self.tf_broadcaster.sendTransform(tf)
        self.pose_counts["published"] += 1

    def source_from_output(self):
        tf = self.tf_buffer.lookup_transform(
            self.expected_pose_child_frame, self.output_child, rclpy.time.Time())
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x,t.y,t.z), (q.x,q.y,q.z,q.w)

    @staticmethod
    def convert_pose(pose_with_covariance, mount):
        p, q = pose_with_covariance.pose.position, pose_with_covariance.pose.orientation
        t, r, cov = change_pose_child((p.x,p.y,p.z), (q.x,q.y,q.z,q.w),
                                    pose_with_covariance.covariance, mount)
        p.x,p.y,p.z = t
        q.x,q.y,q.z,q.w = r
        pose_with_covariance.covariance = cov

    def on_initial_pose(self, message):
        if message.header.frame_id.lstrip('/') != self.expected_pose_frame:
            return
        try:
            message = copy.deepcopy(message)
            # RViz's 2D tool supplies z=0 even in maps whose floor is not zero.
            # Preserve the current map-referenced height; do not seed +0.9m.
            if self.initial_preserve_height and abs(message.pose.pose.position.z) < 1e-8:
                age = (self.get_clock().now().nanoseconds-self.last_output_stamp_ns)/1e9
                if self.last_output_height is None or not 0 <= age <= 1.5:
                    raise ValueError('2D initial pose requires a fresh map-referenced height')
                message.pose.pose.position.z = self.last_output_height
            t, q = self.source_from_output()
            self.convert_pose(message.pose, invert_transform(t,q))
            self.initial_pub.publish(message)
        except Exception as exc:
            self.get_logger().warning('Initial pose conversion waiting: '+str(exc), throttle_duration_sec=5.)

    def report_status_stats(self):
        counts = self.status_counts
        self.get_logger().info(
            "status_forward_stats received={received} published={published} "
            "mapping_errors={mapping_errors}".format(**counts))

    def report_pose_stats(self):
        counts = self.pose_counts
        self.get_logger().info(
            "pose_forward_stats received={received} published={published} "
            "invalid_stamp={invalid_stamp} unexpected_frame={unexpected_frame} "
            "unexpected_child_frame={unexpected_child_frame}".format(**counts))


def main(args=None):
    rclpy.init(args=args)
    node = N01LocalizationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ROS signal handling may already have shut the default context down
        # before KeyboardInterrupt reaches this finally block.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
