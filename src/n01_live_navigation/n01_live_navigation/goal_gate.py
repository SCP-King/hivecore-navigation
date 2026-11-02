"""Fail-closed RViz goal relay: require fresh live inputs and SCAN static PCD layer."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from .core import goal_is_valid



class LiveGoalGate(Node):
    def __init__(self):
        super().__init__("n01_live_scan_goal_gate")
        self.live_ready = False
        self.static_ready = False
        self.live_reason = "waiting_live_adapter"
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, "/grid_map/static_overlay_ready", self.on_static_ready, latched)
        self.create_subscription(
            String, "/n01/live_navigation/readiness", self.on_live_status, 10)
        self.create_subscription(
            PoseStamped, "/move_base_simple/goal", self.on_goal, 10)
        self.goal_pub = self.create_publisher(
            PoseStamped, "/n01/live_navigation/goal_gated", 10)
        self.status_pub = self.create_publisher(
            String, "/n01/live_navigation/goal_gate_status", 10)
        self.get_logger().warning(
            "RViz goal relay is fail-closed until live localization/cloud and native SCAN PCD are ready")

    def on_static_ready(self, msg):
        self.static_ready = bool(msg.data)

    def on_live_status(self, msg):
        try:
            status = json.loads(msg.data)
            self.live_ready = (
                status.get("status") == "LIVE_SCAN_INPUTS_READY" and
                status.get("localization_status_fresh") is True and
                int(status.get("cloud_out", 0)) > 0)
            self.live_reason = status.get("localization_status", "unknown")
        except (ValueError, TypeError, AttributeError):
            self.live_ready = False
            self.live_reason = "invalid_live_readiness_message"

    def on_goal(self, msg):
        valid, reason = goal_is_valid(
            msg, live_ready=self.live_ready, static_ready=self.static_ready)
        self.status_pub.publish(String(data=json.dumps({
            "schema": "n01.live_goal_gate.v1",
            "accepted": valid,
            "reason": reason,
            "localization_reason": self.live_reason,
            "static_overlay_ready": self.static_ready,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }, sort_keys=True)))
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
