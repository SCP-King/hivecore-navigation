"""Fail-closed safety acceptance monitor; reports status and never commands actuators."""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .core import evaluate_safety_gate, stamped_data_is_fresh


class LiveSafetySupervisor(Node):
    def __init__(self):
        super().__init__("n01_live_safety_supervisor")
        defaults = {
            "system_health_topic": "/n01/live_navigation/system_health",
            "feasibility_topic": "/n01/live_navigation/hive2_feasibility",
            "dynamic_obstacle_topic": "/n01/live_navigation/dynamic_obstacle_status",
            "operator_hold_topic": "/n01/live_navigation/operator_hold",
            "max_health_age_s": 0.75,
            "max_feasibility_age_s": 1.0,
            "max_dynamic_status_age_s": 0.6,
            "operator_hold": True,
            "emergency_stop_chain_verified": False,
            "communication_loss_stop_verified": False,
            "localization_loss_stop_verified": False,
            "dynamic_obstacle_stop_verified": False,
            "manual_takeover_verified": False,
            "wheel_controller_mapping_verified": False,
            "wheel_contact_geometry_verified": False,
            "collision_envelope_verified": False,
            "braking_distance_verified": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.max_health_age = float(get("max_health_age_s"))
        self.max_feasibility_age = float(get("max_feasibility_age_s"))
        self.max_dynamic_age = float(get("max_dynamic_status_age_s"))
        self.operator_hold = bool(get("operator_hold"))
        self.physical_checks = {
            name: bool(get(name))
            for name in (
                "emergency_stop_chain_verified",
                "communication_loss_stop_verified",
                "localization_loss_stop_verified",
                "dynamic_obstacle_stop_verified",
                "manual_takeover_verified",
                "wheel_controller_mapping_verified",
                "wheel_contact_geometry_verified",
                "collision_envelope_verified",
                "braking_distance_verified",
            )
        }
        if not (
            0.1 <= self.max_health_age <= 3.0
            and 0.1 <= self.max_feasibility_age <= 3.0
            and 0.1 <= self.max_dynamic_age <= 3.0
        ):
            raise ValueError("safety monitor freshness bounds invalid")

        self.health = {}
        self.feasibility = {}
        self.dynamic_status = {}
        self.health_rx = None
        self.feasibility_rx = None
        self.dynamic_rx = None
        self.create_subscription(
            String, str(get("system_health_topic")), self.on_health, 10)
        self.create_subscription(
            String, str(get("feasibility_topic")), self.on_feasibility, 10)
        self.create_subscription(
            String, str(get("dynamic_obstacle_topic")), self.on_dynamic_status, 10)
        self.create_subscription(
            Bool, str(get("operator_hold_topic")), self.on_operator_hold, 10)
        self.status_pub = self.create_publisher(
            String, "/n01/live_navigation/safety_status", 10)
        self.create_timer(0.25, self.publish_status)
        self.get_logger().warning(
            "SAFETY MONITOR ONLY: operator hold defaults active; execution is never authorized")

    @staticmethod
    def _parse(msg, fallback):
        try:
            value = json.loads(msg.data)
            return value if isinstance(value, dict) else fallback
        except (ValueError, TypeError, AttributeError):
            return fallback

    def on_health(self, msg):
        self.health = self._parse(msg, {"status": "INVALID_SYSTEM_HEALTH"})
        self.health_rx = time.monotonic()

    def on_feasibility(self, msg):
        self.feasibility = self._parse(msg, {"status": "INVALID_FEASIBILITY"})
        self.feasibility_rx = time.monotonic()

    def on_dynamic_status(self, msg):
        self.dynamic_status = self._parse(msg, {"status": "INVALID_DYNAMIC_STATUS"})
        self.dynamic_rx = time.monotonic()

    def on_operator_hold(self, msg):
        self.operator_hold = bool(msg.data)

    def publish_status(self):
        now = time.monotonic()
        health_age = now - self.health_rx if self.health_rx is not None else None
        feasibility_age = (
            now - self.feasibility_rx if self.feasibility_rx is not None else None)
        dynamic_age = now - self.dynamic_rx if self.dynamic_rx is not None else None
        health_fresh = (
            health_age is not None and 0.0 <= health_age <= self.max_health_age)
        feasibility_fresh = (
            feasibility_age is not None and 0.0 <= feasibility_age <= self.max_feasibility_age)
        dynamic_receive_fresh = (
            dynamic_age is not None and 0.0 <= dynamic_age <= self.max_dynamic_age)
        dynamic_stamp_fresh, dynamic_stamp_age = stamped_data_is_fresh(
            self.dynamic_status.get("cloud_stamp_ns"),
            self.get_clock().now().nanoseconds,
            self.max_dynamic_age,
        )
        dynamic_fresh = bool(dynamic_receive_fresh and dynamic_stamp_fresh)
        status = evaluate_safety_gate(
            health_fresh=health_fresh,
            navigation_ready=self.health.get("navigation_ready") is True,
            feasibility_fresh=feasibility_fresh,
            feasibility_status=self.feasibility.get("status", "UNKNOWN"),
            candidate_review_ready=self.feasibility.get("candidate_review_ready") is True,
            operator_hold=self.operator_hold,
            physical_checks=self.physical_checks,
            dynamic_obstacle_fresh=dynamic_fresh,
            dynamic_obstacle_status=self.dynamic_status.get("status", "UNKNOWN"),
        )
        status.update({
            "schema": "n01.live_safety_status.v1",
            "operator_hold": bool(self.operator_hold),
            "system_health_age_s": health_age,
            "feasibility_age_s": feasibility_age,
            "dynamic_obstacle_status_age_s": dynamic_age,
            "dynamic_obstacle_receive_fresh": bool(dynamic_receive_fresh),
            "dynamic_obstacle_cloud_stamp_age_s": dynamic_stamp_age,
            "dynamic_obstacle_status_fresh": bool(dynamic_fresh),
            "dynamic_obstacle_status": self.dynamic_status,
            "physical_checks": self.physical_checks,
            "candidate_only": True,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        })
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = LiveSafetySupervisor()
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
