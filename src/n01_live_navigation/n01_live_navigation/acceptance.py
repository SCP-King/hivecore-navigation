"""Read-only snapshot of N-01 navigation stage topics."""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class AcceptanceSnapshot(Node):
    def __init__(self):
        super().__init__("n01_navigation_acceptance")
        self.data = {}
        self.body_rx = []
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for key, topic in (
            ("readiness", "/n01/live_navigation/readiness"),
            ("health", "/n01/live_navigation/system_health"),
            ("dynamic", "/n01/live_navigation/dynamic_obstacle_status"),
            ("feasibility", "/n01/live_navigation/hive2_feasibility"),
            ("safety", "/n01/live_navigation/safety_status"),
        ):
            self.create_subscription(String, topic, lambda m, k=key: self._json(k, m), 10)
        self.create_subscription(
            Bool, "/n01/live_navigation/navigation_ready",
            lambda m: self.data.update({"navigation_ready": bool(m.data)}), 10)
        self.create_subscription(
            Bool, "/grid_map/static_overlay_ready",
            lambda m: self.data.update({"static_overlay_ready": bool(m.data)}), latched)
        self.create_subscription(
            Odometry, "/body_pose", self._body_pose, qos_profile_sensor_data)

    def _json(self, key, msg):
        try:
            self.data[key] = json.loads(msg.data)
        except (ValueError, TypeError):
            self.data[key] = {"invalid": True}

    def _body_pose(self, msg):
        stamp = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
        self.body_rx.append((stamp, time.monotonic()))
        self.body_rx = self.body_rx[-300:]

    def snapshot(self):
        now_ns = self.get_clock().now().nanoseconds
        now = time.monotonic()
        gaps = [b[1] - a[1] for a, b in zip(self.body_rx, self.body_rx[1:])]
        rate = (
            (len(self.body_rx) - 1) / (self.body_rx[-1][1] - self.body_rx[0][1])
            if len(self.body_rx) > 1 and self.body_rx[-1][1] > self.body_rx[0][1]
            else 0.0)
        body_age = (now_ns - self.body_rx[-1][0]) / 1e9 if self.body_rx else None
        body_rx_age = now - self.body_rx[-1][1] if self.body_rx else None
        return {
            "topics": self.data,
            "body_pose": {
                "samples": len(self.body_rx),
                "receive_rate_hz": round(rate, 3),
                "max_receive_gap_s": round(max(gaps), 4) if gaps else None,
                "stamp_age_s": round(body_age, 4) if body_age is not None else None,
                "receive_age_s": round(body_rx_age, 4) if body_rx_age is not None else None,
            },
            "actuator_output": "DISABLED",
            "execution_authorized": False,
        }


def main(args=None):
    rclpy.init(args=args)
    node = AcceptanceSnapshot()
    try:
        deadline = time.monotonic() + 6.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(json.dumps(node.snapshot(), indent=2, ensure_ascii=False, sort_keys=True))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
