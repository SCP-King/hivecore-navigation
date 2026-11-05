"""Backend-neutral N-01 localization adapter.

Discovers the external status message type at runtime and maps only explicitly
configured field paths. Unknown or unmapped status fields always fail closed.
"""
import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
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


class N01LocalizationAdapter(Node):
    def __init__(self):
        super().__init__("n01_localization_adapter")
        defaults = {
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
            "expected_pose_child_frame": "base_footprint",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
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
        period = float(get("status_discovery_period_s"))
        if not all((self.status_input_topic.startswith("/"),
                    self.status_output_topic.startswith("/"),
                    self.pose_input_topic.startswith("/"),
                    self.pose_output_topic.startswith("/"),
                    0.2 <= period <= 5.0,
                    isinstance(self.field_paths, dict),
                    isinstance(self.expected_values, dict))):
            raise ValueError("invalid localization adapter configuration")

        self.status_pub = self.create_publisher(
            DiagnosticArray, self.status_output_topic, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(
            Odometry, self.pose_output_topic, qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(
            Odometry, self.pose_input_topic, self.on_pose, qos_profile_sensor_data)
        self.status_sub = None
        self.status_type = ""
        self.status_counts = {"received": 0, "published": 0, "mapping_errors": 0}
        self.pose_counts = {"received": 0, "published": 0, "rejected": 0}
        self.create_timer(period, self.discover_status_type)
        self.get_logger().info(
            "N-01 backend adapter: status %s -> %s; pose %s -> %s"
            % (self.status_input_topic, self.status_output_topic,
               self.pose_input_topic, self.pose_output_topic))
        self.get_logger().warning(
            "Verified LocStatus field mapping loaded; unknown fields fail closed. "
            "No absent TF, IMU or general quality fields are inferred.")

    def discover_status_type(self):
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

    def on_pose(self, message):
        self.pose_counts["received"] += 1
        stamp_ns = self.stamp_ns(message.header.stamp)
        frame = str(message.header.frame_id).lstrip("/")
        child = str(message.child_frame_id).lstrip("/")
        if (stamp_ns <= 0 or not self.expected_pose_frame
                or not self.expected_pose_child_frame
                or frame != self.expected_pose_frame
                or child != self.expected_pose_child_frame):
            self.pose_counts["rejected"] += 1
            return
        self.pose_pub.publish(message)
        self.pose_counts["published"] += 1


def main(args=None):
    rclpy.init(args=args)
    node = N01LocalizationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
