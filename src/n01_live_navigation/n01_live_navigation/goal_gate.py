"""Fail-closed RViz goal relay; planning inputs only, never robot commands."""
import json
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from .core import goal_is_valid


class LiveGoalGate(Node):
    def __init__(self):
        super().__init__("n01_live_scan_goal_gate")
        self.health = {}
        self.health_rx_monotonic = None
        self.health_error = "waiting_system_health"
        self.live_ready = False
        self.navigation_ready = False
        self.static_ready = False
        self.live_reason = "waiting_system_health"
        self.last_goal_status = None
        self.last_goal_monotonic = None
        # Use one supervisor snapshot for every gate decision. Separate Bool
        # and bridge topics can be sampled at different times and disagree.
        self.create_subscription(
            String, "/n01/live_navigation/system_health", self.on_system_health, 10)
        self.create_subscription(
            PoseStamped, "/move_base_simple/goal", self.on_goal, 10)
        self.goal_pub = self.create_publisher(
            PoseStamped, "/n01/live_navigation/goal_gated", 10)
        self.status_pub = self.create_publisher(
            String, "/n01/live_navigation/goal_gate_status", 10)
        # Publish a heartbeat even before the operator selects a goal. This
        # distinguishes an idle gate from a missing/dead gate process.
        self.create_timer(0.5, self.publish_heartbeat)
        self.get_logger().warning(
            "RViz goal relay is fail-closed until localization, fresh scan, body pose and static PCD are ready")

    def on_system_health(self, msg):
        try:
            health = json.loads(msg.data)
            if not isinstance(health, dict):
                raise ValueError("system_health_not_object")
            blockers = health.get("blockers")
            if not isinstance(blockers, list):
                raise ValueError("system_health_blockers_missing")
            self.health = health
            self.health_rx_monotonic = time.monotonic()
            self.health_error = ""
            self.navigation_ready = bool(
                health.get("status") == "READY_FOR_GOAL_PLANNING"
                and health.get("navigation_ready") is True
                and not blockers
            )
            self.static_ready = health.get("static_overlay_ready") is True
            self.live_ready = self.navigation_ready
            self.live_reason = (
                "ready" if self.navigation_ready
                else str(blockers[0]) if blockers
                else "system_health_not_ready"
            )
        except (ValueError, TypeError, AttributeError) as exc:
            self.health = {}
            self.health_rx_monotonic = None
            self.health_error = str(exc)
            self.navigation_ready = False
            self.static_ready = False
            self.live_ready = False
            self.live_reason = "invalid_system_health_message"

    def publish_heartbeat(self):
        now = time.monotonic()
        health_age = (
            now - self.health_rx_monotonic
            if self.health_rx_monotonic is not None else None)
        health_fresh = bool(
            health_age is not None and 0.0 <= health_age <= 0.75)
        goal_age = (
            now - self.last_goal_monotonic
            if self.last_goal_monotonic is not None else None)
        last = self.last_goal_status or {}
        status = {
            "schema": "n01.live_goal_gate.v2",
            "event": "heartbeat",
            "goal_received": self.last_goal_status is not None,
            "goal_event_age_s": goal_age,
            "accepted": False,
            "last_goal_accepted": last.get("last_goal_accepted", False),
            "reason": (
                "waiting_for_goal" if self.last_goal_status is None
                else str(last.get("reason", "goal_event_recorded"))),
            "localization_reason": self.live_reason,
            "system_health_status": self.health.get("status", "WAITING"),
            "system_health_fresh": health_fresh,
            "system_health_age_s": health_age,
            "system_health_blockers": self.health.get("blockers", []),
            "system_health_error": self.health_error,
            "navigation_ready": self.navigation_ready,
            "static_overlay_ready": self.static_ready,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def on_goal(self, msg):
        health_age = (
            time.monotonic() - self.health_rx_monotonic
            if self.health_rx_monotonic is not None else None)
        health_fresh = bool(
            health_age is not None and 0.0 <= health_age <= 0.75)
        valid, reason = goal_is_valid(
            msg,
            live_ready=health_fresh and self.live_ready and self.navigation_ready,
            static_ready=health_fresh and self.static_ready,
        )
        if not health_fresh:
            valid, reason = False, "system_health_missing_or_stale"
        elif not self.navigation_ready and self.health.get("blockers"):
            valid, reason = False, str(self.health["blockers"][0])
        stamp_ns = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        age_s = (now_ns - stamp_ns) / 1e9 if stamp_ns > 0 else None
        if valid and (age_s is None or age_s > 5.0 or age_s < -0.5):
            valid, reason = False, "goal_stamp_missing_stale_or_future"
        status = {
            "schema": "n01.live_goal_gate.v2",
            "event": "goal_evaluated",
            "goal_received": True,
            "goal_event_age_s": 0.0,
            "accepted": bool(valid),
            "last_goal_accepted": bool(valid),
            "reason": reason,
            "localization_reason": self.live_reason,
            "system_health_status": self.health.get("status", "WAITING"),
            "system_health_fresh": health_fresh,
            "system_health_age_s": health_age,
            "system_health_blockers": self.health.get("blockers", []),
            "system_health_error": self.health_error,
            "navigation_ready": self.navigation_ready,
            "static_overlay_ready": self.static_ready,
            "goal_stamp_age_s": age_s,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
        self.last_goal_status = dict(status)
        self.last_goal_monotonic = time.monotonic()
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))
        if not valid:
            self.get_logger().warning("RViz goal rejected: " + reason)
            return
        self.goal_pub.publish(msg)
        self.get_logger().info(
            "RViz goal forwarded to native SCAN: x=%.3f y=%.3f"
            % (msg.pose.position.x, msg.pose.position.y))


def main(args=None):
    rclpy.init(args=args)
    node = LiveGoalGate()
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
