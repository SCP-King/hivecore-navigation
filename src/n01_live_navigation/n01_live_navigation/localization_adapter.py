"""Hivecore-specific localization status adapter.

This is the only N-01 module that knows the vendor LocalizationStatus schema.
It translates that schema into the stable, backend-neutral DiagnosticArray
contract consumed by the navigation core.
"""
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


class HivecoreLocalizationAdapter(Node):
    def __init__(self):
        super().__init__("n01_hivecore_localization_adapter")
        self.declare_parameter("input_topic", "/localization_status")
        self.declare_parameter("input_type", "hivecore_slam3d/msg/LocalizationStatus")
        self.declare_parameter("output_topic", "/n01/localization/status")
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.input_type = str(self.get_parameter("input_type").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        if not self.input_topic or not self.output_topic:
            raise ValueError("localization adapter topics must be non-empty")
        self.input_class = get_message(self.input_type)
        self.publisher = self.create_publisher(
            DiagnosticArray, self.output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            self.input_class, self.input_topic, self.on_status, qos_profile_sensor_data)
        self.get_logger().info(
            "Hivecore-specific status adapter: %s -> %s" %
            (self.input_topic, self.output_topic))

    @staticmethod
    def _bool(value):
        return "true" if bool(value) else "false"

    def on_status(self, message):
        values = {
            "map_loaded": self._bool(getattr(message, "config_context_map_loaded", False)),
            "role": str(getattr(message, "control_signal_outer_role", "")),
            "mode": str(getattr(message, "control_signal_session_mode", "")),
            "pose_valid": self._bool(getattr(message, "registration_output_pose_published", False)),
            "tf_valid": self._bool(getattr(message, "registration_output_tf_published", False)),
            "quality_valid": self._bool(getattr(message, "registration_output_fitness_evaluation_ok", False)),
            "tracking_valid": self._bool(getattr(message, "registration_output_tracking_compose_ok", False)),
            "imu_sync_ok": self._bool(getattr(message, "fusion_path_imu_sync_ok", False)),
        }
        try:
            fitness = float(getattr(message, "registration_output_fitness_mse", float("nan")))
        except (TypeError, ValueError):
            fitness = float("nan")
        values["fitness_mse"] = repr(fitness) if math.isfinite(fitness) else "nan"

        ready = all((
            values["map_loaded"] == "true",
            values["role"].lower() == "localization",
            values["mode"].lower() == "tracking",
            values["pose_valid"] == "true",
            values["tf_valid"] == "true",
            values["quality_valid"] == "true",
            values["tracking_valid"] == "true",
        ))
        status = DiagnosticStatus()
        status.name = "localization"
        status.level = DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR
        status.message = "tracking_ready" if ready else "localization_not_ready"
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        output = DiagnosticArray()
        output.header = message.header
        output.status = [status]
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = HivecoreLocalizationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
