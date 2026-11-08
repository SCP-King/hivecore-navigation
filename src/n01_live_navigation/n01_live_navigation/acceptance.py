"""Read-only, goal-aware snapshot of the live N-01 planning pipeline.

The node never publishes goals or actuator commands. During the sampling window
the operator may set one RViz 2D Goal Pose to exercise Goal Gate -> native SCAN.
"""
import json
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from scan_planner_msgs.msg import Bspline, DataDisp
from std_msgs.msg import Bool, String


MANAGED_NODES = (
    "n01_static_map",
    "n01_live_scan_bridge",
    "n01_live_runtime_supervisor",
    "n01_live_safety_supervisor",
    "n01_live_dynamic_obstacle_tracker",
    "n01_hive2_trajectory_monitor",
    "n01_live_scan_goal_gate",
    "scan_planner_node",
)


class AcceptanceSnapshot(Node):
    def __init__(self):
        super().__init__("n01_navigation_acceptance")
        self.data = {}
        self.body_rx = []
        self.goal_input_count = 0
        self.goal_gated_count = 0
        self.bspline_count = 0
        self.data_display_count = 0
        self.last_goal_gate = None
        self.goal_gate_rx = None
        self.last_gated_goal = None
        self.last_bspline = None
        self.last_data_display = None
        self.graph_node_counts = Counter(
            f"{namespace.rstrip('/')}/{name}".replace("//", "/")
            for name, namespace in self.get_node_names_and_namespaces()
        )

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
            String, "/n01/live_navigation/goal_gate_status", self._goal_gate, 10)
        self.create_subscription(
            Bool, "/n01/live_navigation/navigation_ready",
            lambda m: self.data.update({"navigation_ready": bool(m.data)}), 10)
        self.create_subscription(
            Bool, "/grid_map/static_overlay_ready",
            lambda m: self.data.update({"static_overlay_ready": bool(m.data)}), latched)
        self.create_subscription(
            Odometry, "/body_pose", self._body_pose, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "/move_base_simple/goal", self._goal_input, 10)
        self.create_subscription(
            PoseStamped, "/n01/live_navigation/goal_gated", self._goal_gated, 10)
        self.create_subscription(
            Bspline, "/planning/bspline", self._bspline, 10)
        self.create_subscription(
            DataDisp, "/planning/data_display", self._data_display, 10)

    def _json(self, key, msg):
        try:
            self.data[key] = json.loads(msg.data)
        except (ValueError, TypeError):
            self.data[key] = {"invalid": True, "raw": str(msg.data)}

    def _goal_gate(self, msg):
        try:
            self.last_goal_gate = json.loads(msg.data)
        except (ValueError, TypeError):
            self.last_goal_gate = {"invalid": True, "raw": str(msg.data)}
        self.goal_gate_rx = time.monotonic()

    def _body_pose(self, msg):
        stamp = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
        self.body_rx.append((stamp, time.monotonic()))
        self.body_rx = self.body_rx[-300:]

    def _goal_input(self, msg):
        self.goal_input_count += 1

    def _goal_gated(self, msg):
        self.goal_gated_count += 1
        self.last_gated_goal = {
            "frame_id": msg.header.frame_id,
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
        }

    def _bspline(self, msg):
        self.bspline_count += 1
        self.last_bspline = {
            "traj_id": int(msg.traj_id),
            "order": int(msg.order),
            "control_points": len(msg.pos_pts),
        }

    def _data_display(self, msg):
        self.data_display_count += 1
        self.last_data_display = {
            "a": float(msg.a), "b": float(msg.b), "c": float(msg.c),
            "d": float(msg.d), "e": float(msg.e),
        }

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
        gate_age = now - self.goal_gate_rx if self.goal_gate_rx is not None else None
        duplicates = {
            name: self.graph_node_counts[f"/{name}"]
            for name in MANAGED_NODES
            if self.graph_node_counts[f"/{name}"] > 1
        }

        if self.goal_input_count == 0:
            planning_state = "WAITING_FOR_RVIZ_GOAL"
        elif self.last_goal_gate is None:
            planning_state = "GOAL_SEEN_BUT_GATE_STATUS_MISSING"
        elif self.last_goal_gate.get("last_goal_accepted") is not True:
            planning_state = "GOAL_REJECTED_OR_NOT_ACCEPTED"
        elif self.goal_gated_count == 0:
            planning_state = "GATE_ACCEPTED_BUT_NO_FORWARDED_GOAL_OBSERVED"
        elif self.bspline_count == 0:
            planning_state = "GOAL_FORWARDED_WAITING_FOR_NATIVE_SCAN_BSPLINE"
        else:
            planning_state = "NATIVE_SCAN_BSPLINE_RECEIVED"

        health = self.data.get("health", {})
        feasibility = self.data.get("feasibility", {})
        safety = self.data.get("safety", {})
        candidate_review_ready = feasibility.get("candidate_review_ready") is True
        return {
            "schema": "n01.live_planning_acceptance.v1",
            "sample_duration_s": 30,
            "graph": {
                "managed_node_counts": {
                    name: self.graph_node_counts[f"/{name}"] for name in MANAGED_NODES
                },
                "duplicate_managed_nodes": duplicates,
                "single_instance_pass": not duplicates and all(
                    self.graph_node_counts[f"/{name}"] == 1 for name in MANAGED_NODES
                ),
            },
            "navigation": {
                "navigation_ready_topic": self.data.get("navigation_ready"),
                "system_health": health,
                "static_overlay_ready": self.data.get("static_overlay_ready"),
                "readiness_diagnostic": self.data.get("readiness"),
            },
            "goal_pipeline": {
                "state": planning_state,
                "rviz_goal_input_count": self.goal_input_count,
                "goal_gate_status": self.last_goal_gate,
                "goal_gate_status_receive_age_s": (
                    round(gate_age, 4) if gate_age is not None else None),
                "forwarded_goal_count": self.goal_gated_count,
                "last_forwarded_goal": self.last_gated_goal,
                "bspline_count": self.bspline_count,
                "last_bspline": self.last_bspline,
                "data_display_count": self.data_display_count,
                "last_data_display": self.last_data_display,
            },
            "scene": {
                "dynamic_obstacle_status": self.data.get("dynamic"),
            },
            "candidate_review": {
                "feasibility": feasibility,
                "candidate_review_ready": candidate_review_ready,
                "safety": safety,
            },
            "body_pose": {
                "samples": len(self.body_rx),
                "receive_rate_hz": round(rate, 3),
                "max_receive_gap_s": round(max(gaps), 4) if gaps else None,
                "stamp_age_s": round(body_age, 4) if body_age is not None else None,
                "receive_age_s": round(body_rx_age, 4) if body_rx_age is not None else None,
            },
            "actuator_output": "DISABLED",
            "execution_authorized": False,
            "safety_authorization": "NEVER",
        }


def main(args=None):
    rclpy.init(args=args)
    node = AcceptanceSnapshot()
    try:
        duration_s = 30.0
        print(
            "N-01 read-only planning acceptance started. "
            "During the next 30 seconds, set ONE 2D Goal Pose in RViz. "
            "This tool does not publish goals or actuator commands.",
            flush=True,
        )
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(json.dumps(node.snapshot(), indent=2, ensure_ascii=False, sort_keys=True))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
