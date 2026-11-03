#!/usr/bin/env python3
"""One-shot SCAN planning test; publishes a goal only, never actuator commands."""
import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scan_planner_msgs.msg import Bspline
from std_msgs.msg import String


def parse_json(msg):
    try:
        value = json.loads(msg.data)
        return value if isinstance(value, dict) else None
    except (ValueError, TypeError, AttributeError):
        return None


def fqn(endpoint):
    namespace = str(endpoint.node_namespace).rstrip("/")
    return (namespace + "/" + str(endpoint.node_name)) if namespace else "/" + str(endpoint.node_name)


ROBOT_POSE_MAX_RX_AGE_S = 0.7
ROBOT_POSE_MAX_STAMP_AGE_S = 1.5
BODY_POSE_LIVE_ANCHOR_MAX_DELTA_S = 1.4


class PlanningSmokeTest(Node):
    def __init__(self, distance, timeout, readiness_timeout):
        super().__init__("n01_planning_smoke_test")
        self.distance = distance
        self.timeout = timeout
        self.readiness_timeout = readiness_timeout
        self.body = self.robot_pose = None
        self.health = self.safety = self.gate = self.feasibility = None
        self.body_rx = self.robot_pose_rx = None
        self.health_rx = self.safety_rx = None
        self.gate_rx = self.feasibility_rx = self.bspline_rx = None
        self.bspline = None
        self.bspline_count = 0

        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.create_subscription(Odometry, "/robot_pose", self.on_robot_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/body_pose", self.on_body, qos_profile_sensor_data)
        self.create_subscription(String, "/n01/live_navigation/system_health", self.on_health, 10)
        self.create_subscription(String, "/n01/live_navigation/safety_status", self.on_safety, 10)
        self.create_subscription(String, "/n01/live_navigation/goal_gate_status", self.on_gate, 10)
        self.create_subscription(String, "/n01/live_navigation/hive2_feasibility", self.on_feasibility, 10)
        self.create_subscription(Bspline, "/planning/bspline", self.on_bspline, 10)

    def on_robot_pose(self, msg):
        self.robot_pose, self.robot_pose_rx = msg, time.monotonic()

    def on_body(self, msg):
        self.body, self.body_rx = msg, time.monotonic()

    def on_health(self, msg):
        self.health, self.health_rx = parse_json(msg), time.monotonic()

    def on_safety(self, msg):
        self.safety, self.safety_rx = parse_json(msg), time.monotonic()

    def on_gate(self, msg):
        self.gate, self.gate_rx = parse_json(msg), time.monotonic()

    def on_feasibility(self, msg):
        self.feasibility, self.feasibility_rx = parse_json(msg), time.monotonic()

    def on_bspline(self, msg):
        self.bspline = msg
        self.bspline_rx = time.monotonic()
        self.bspline_count += 1

    def spin_until(self, predicate, deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return bool(predicate())

    def graph_contract(self):
        goal_subs = {fqn(x) for x in self.get_subscriptions_info_by_topic("/move_base_simple/goal")}
        if "/n01_live_scan_goal_gate" not in goal_subs:
            return False, "existing_goal_gate_not_subscribed"

        pubs = {fqn(x) for x in self.get_publishers_info_by_topic("/planning/bspline")}
        if pubs != {"/scan_planner_node"}:
            return False, "unexpected_bspline_publishers:" + repr(sorted(pubs))

        expected = {"/n01_live_runtime_supervisor", "/n01_hive2_trajectory_monitor"}
        # This smoke-test node subscribes to /planning/bspline itself so it can
        # verify the generated candidate. Permit only this known observer in
        # addition to the two runtime consumers; all other consumers remain
        # a fail-closed topology violation.
        observer = {"/n01_planning_smoke_test"}
        subs = {fqn(x) for x in self.get_subscriptions_info_by_topic("/planning/bspline")}
        unexpected = subs - expected - observer
        if unexpected:
            return False, "unexpected_bspline_consumers:" + repr(sorted(unexpected))
        if expected - subs:
            return False, "candidate_monitors_missing:" + repr(sorted(expected - subs))
        return True, "planning_only_graph_verified"

    def preflight(self, check_graph=True):
        now = time.monotonic()
        if (self.robot_pose is None or self.robot_pose_rx is None
                or now - self.robot_pose_rx > ROBOT_POSE_MAX_RX_AGE_S):
            return False, "robot_pose_receive_missing_or_stale"
        if self.robot_pose.header.frame_id.lstrip("/") != "map":
            return False, "robot_pose_frame_must_be_map"
        if self.robot_pose.child_frame_id.lstrip("/") != "base_footprint":
            return False, "robot_pose_child_frame_must_be_base_footprint"
        robot_stamp = (
            self.robot_pose.header.stamp.sec * 1000000000
            + self.robot_pose.header.stamp.nanosec
        )
        robot_stamp_age = (self.get_clock().now().nanoseconds - robot_stamp) / 1e9
        if robot_stamp <= 0 or not -0.2 <= robot_stamp_age <= ROBOT_POSE_MAX_STAMP_AGE_S:
            return False, "robot_pose_stamp_missing_or_stale"

        if self.body is None or self.body_rx is None or now - self.body_rx > 0.5:
            return False, "body_pose_missing_or_stale"
        if self.body.header.frame_id.lstrip("/") != "map":
            return False, "body_pose_frame_must_be_map"
        if self.body.child_frame_id.lstrip("/") != "base_footprint":
            return False, "body_pose_child_frame_must_be_base_footprint"
        body_stamp = self.body.header.stamp.sec * 1000000000 + self.body.header.stamp.nanosec
        body_stamp_age = (self.get_clock().now().nanoseconds - body_stamp) / 1e9
        if body_stamp <= 0 or not -0.2 <= body_stamp_age <= 0.5:
            return False, "body_pose_stamp_missing_or_stale"

        if self.health is None or self.health_rx is None or now - self.health_rx > 0.75:
            return False, "system_health_missing_or_stale"
        bridge = self.health.get("bridge")
        if not isinstance(bridge, dict):
            return False, "system_health_bridge_snapshot_missing"
        if (bridge.get("robot_pose_receive_fresh") is not True
                or bridge.get("body_pose_live_anchor_fresh") is not True):
            return False, "global_robot_pose_or_live_anchor_not_fresh"
        health_age = now - self.health_rx
        bridge_rx_age = bridge.get("robot_pose_receive_age_s")
        if (not isinstance(bridge_rx_age, (int, float))
                or not math.isfinite(float(bridge_rx_age))
                or not 0.0 <= float(bridge_rx_age) + health_age <= ROBOT_POSE_MAX_RX_AGE_S):
            return False, "global_robot_pose_receive_age_outside_budget"
        bridge_stamp_age = bridge.get("robot_pose_stamp_age_s")
        if (not isinstance(bridge_stamp_age, (int, float))
                or not math.isfinite(float(bridge_stamp_age))
                or not -0.2 <= float(bridge_stamp_age) + health_age <= ROBOT_POSE_MAX_STAMP_AGE_S):
            return False, "global_robot_pose_stamp_missing_or_stale"
        anchor_delta = bridge.get("body_pose_live_anchor_delta_s")
        if (not isinstance(anchor_delta, (int, float))
                or not math.isfinite(float(anchor_delta))
                or not 0.0 <= float(anchor_delta) + health_age
                <= BODY_POSE_LIVE_ANCHOR_MAX_DELTA_S):
            return False, "body_pose_live_anchor_delta_outside_budget"
        if (self.health.get("status") != "READY_FOR_GOAL_PLANNING"
                or self.health.get("navigation_ready") is not True
                or self.health.get("blockers") != []
                or self.health.get("static_overlay_ready") is not True):
            return False, "navigation_or_static_overlay_not_ready"

        if self.safety is None or self.safety_rx is None or now - self.safety_rx > 1.0:
            return False, "safety_status_missing_or_stale"
        if (self.safety.get("actuator_output") != "DISABLED"
                or self.safety.get("safety_authorization") != "NEVER"
                or self.safety.get("execution_authorized") is not False):
            return False, "safety_supervisor_not_explicitly_fail_closed"
        if not check_graph:
            return True, "freshness_and_readiness_verified"
        if self.goal_pub.get_subscription_count() < 1:
            return False, "goal_gate_subscription_missing"
        return self.graph_contract()

    def make_goal(self):
        pose = self.body.pose.pose
        q = pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-6:
            raise ValueError("body_pose_quaternion_invalid")
        qx, qy, qz, qw = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(pose.position.x + self.distance*math.cos(yaw))
        goal.pose.position.y = float(pose.position.y + self.distance*math.sin(yaw))
        # SCAN takes its planning Z from current body_pose; zero is the goal-gate sentinel.
        goal.pose.position.z = 0.0
        goal.pose.orientation.x, goal.pose.orientation.y = qx, qy
        goal.pose.orientation.z, goal.pose.orientation.w = qz, qw
        return goal

    @staticmethod
    def spline_summary(msg):
        order = int(msg.order)
        points = [(float(p.x), float(p.y), float(p.z)) for p in msg.pos_pts]
        knots = [float(v) for v in msg.knots]
        finite = (all(math.isfinite(v) for p in points for v in p)
                  and all(math.isfinite(v) for v in knots))
        valid = order >= 1 and len(points) >= order + 1 and len(knots) >= 2 and finite
        return {
            "valid": bool(valid), "order": order,
            "control_point_count": len(points), "knot_count": len(knots),
            "trajectory_id": int(msg.traj_id),
            "all_control_points_and_knots_finite": bool(finite),
        }

    def run_test(self):
        readiness_started = time.monotonic()
        preflight_deadline = readiness_started + self.readiness_timeout
        ok, reason = False, "navigation_preflight_timeout"
        goal = None
        while rclpy.ok() and time.monotonic() < preflight_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            ok, reason = self.preflight(check_graph=False)
            if not ok:
                continue
            # Recheck freshness and the ROS graph immediately before goal
            # construction/publication. Graph discovery is intentionally done
            # only at this final point, not 10 times per second while waiting.
            ok, reason = self.preflight(check_graph=True)
            if ok:
                try:
                    goal = self.make_goal()
                except ValueError as exc:
                    return 2, {"result": "PREFLIGHT_FAILED", "reason": str(exc)}
                break
        if not ok or goal is None:
            return 2, {
                "result": "PREFLIGHT_FAILED",
                "reason": reason,
                "readiness_wait_s": round(time.monotonic() - readiness_started, 3),
                "readiness_timeout_s": self.readiness_timeout,
                "goal_published": False,
            }

        baseline = self.bspline_count
        sent_at = time.monotonic()
        health_at_publish = dict(self.health)
        bridge_at_publish = health_at_publish.get("bridge", {})
        if not isinstance(bridge_at_publish, dict):
            bridge_at_publish = {}
        safety_at_publish = dict(self.safety)
        robot_pose_receive_age_at_publish = sent_at - self.robot_pose_rx
        robot_pose_stamp_at_publish = (
            self.robot_pose.header.stamp.sec * 1000000000
            + self.robot_pose.header.stamp.nanosec
        )
        robot_pose_stamp_age_at_publish = (
            self.get_clock().now().nanoseconds - robot_pose_stamp_at_publish
        ) / 1e9
        readiness_snapshot = {
            "status": health_at_publish.get("status"),
            "blockers": health_at_publish.get("blockers"),
            "static_overlay_ready": health_at_publish.get("static_overlay_ready"),
            "robot_pose_receive_fresh": bridge_at_publish.get("robot_pose_receive_fresh"),
            "robot_pose_receive_age_s": bridge_at_publish.get("robot_pose_receive_age_s"),
            "robot_pose_stamp_age_s": bridge_at_publish.get("robot_pose_stamp_age_s"),
            "body_pose_live_anchor_fresh": bridge_at_publish.get("body_pose_live_anchor_fresh"),
            "body_pose_live_anchor_delta_s": bridge_at_publish.get("body_pose_live_anchor_delta_s"),
            "observer_robot_pose_receive_age_s": round(robot_pose_receive_age_at_publish, 4),
            "observer_robot_pose_stamp_age_s": round(robot_pose_stamp_age_at_publish, 4),
            "system_health_receive_age_s": round(health_rx_age_at_publish, 4),
            "bridge_robot_pose_receive_age_effective_s": (
                round(float(bridge_at_publish["robot_pose_receive_age_s"])
                      + health_rx_age_at_publish, 4)
                if isinstance(bridge_at_publish.get("robot_pose_receive_age_s"), (int, float))
                else None
            ),
            "bridge_body_pose_live_anchor_delta_effective_s": (
                round(float(bridge_at_publish["body_pose_live_anchor_delta_s"])
                      + health_rx_age_at_publish, 4)
                if isinstance(bridge_at_publish.get("body_pose_live_anchor_delta_s"), (int, float))
                else None
            ),
        }
        health_rx_age_at_publish = (
            sent_at - self.health_rx if self.health_rx is not None else float("inf"))
        safety_rx_age_at_publish = (
            sent_at - self.safety_rx if self.safety_rx is not None else float("inf"))
        body_rx_age_at_publish = (
            sent_at - self.body_rx if self.body_rx is not None else float("inf"))
        body_stamp_at_publish = (
            self.body.header.stamp.sec * 1000000000
            + self.body.header.stamp.nanosec
        )
        body_stamp_age_at_publish = (
            self.get_clock().now().nanoseconds - body_stamp_at_publish
        ) / 1e9
        bridge_rx_age_at_publish = bridge_at_publish.get("robot_pose_receive_age_s")
        bridge_stamp_age_at_publish = bridge_at_publish.get("robot_pose_stamp_age_s")
        final_snapshot_valid = (
            robot_pose_receive_age_at_publish <= ROBOT_POSE_MAX_RX_AGE_S
            and -0.2 <= robot_pose_stamp_age_at_publish <= ROBOT_POSE_MAX_STAMP_AGE_S
            and body_rx_age_at_publish <= 0.5
            and -0.2 <= body_stamp_age_at_publish <= 0.5
            and health_rx_age_at_publish <= 0.75
            and safety_rx_age_at_publish <= 1.0
            and bridge_at_publish.get("robot_pose_receive_fresh") is True
            and isinstance(bridge_rx_age_at_publish, (int, float))
            and math.isfinite(float(bridge_rx_age_at_publish))
            and 0.0 <= float(bridge_rx_age_at_publish) + health_rx_age_at_publish
            <= ROBOT_POSE_MAX_RX_AGE_S
            and isinstance(bridge_stamp_age_at_publish, (int, float))
            and math.isfinite(float(bridge_stamp_age_at_publish))
            and -0.2 <= float(bridge_stamp_age_at_publish) + health_rx_age_at_publish
            <= ROBOT_POSE_MAX_STAMP_AGE_S
            and bridge_at_publish.get("body_pose_live_anchor_fresh") is True
            and isinstance(bridge_at_publish.get("body_pose_live_anchor_delta_s"), (int, float))
            and math.isfinite(float(bridge_at_publish.get("body_pose_live_anchor_delta_s")))
            and 0.0 <= float(bridge_at_publish.get("body_pose_live_anchor_delta_s"))
            + health_rx_age_at_publish <= BODY_POSE_LIVE_ANCHOR_MAX_DELTA_S
            and health_at_publish.get("status") == "READY_FOR_GOAL_PLANNING"
            and health_at_publish.get("navigation_ready") is True
            and health_at_publish.get("blockers") == []
            and health_at_publish.get("static_overlay_ready") is True
            and safety_at_publish.get("actuator_output") == "DISABLED"
            and safety_at_publish.get("safety_authorization") == "NEVER"
            and safety_at_publish.get("execution_authorized") is False
        )
        if not final_snapshot_valid:
            return 2, {
                "result": "PREFLIGHT_FAILED",
                "reason": "readiness_window_closed_at_publish",
                "readiness_snapshot": readiness_snapshot,
                "goal_published": False,
                "actuator_command_published": False,
                "execution_authorized": False,
            }
        self.goal_pub.publish(goal)

        if not self.spin_until(
            lambda: self.gate_rx is not None and self.gate_rx >= sent_at,
            sent_at + min(3.0, self.timeout)):
            return 3, {
                "result": "GOAL_GATE_TIMEOUT",
                "goal_published": True,
                "actuator_command_published": False,
                "execution_authorized": False,
                "readiness_snapshot": readiness_snapshot,
            }
        gate = self.gate or {}
        if gate.get("accepted") is not True:
            return 3, {
                "result": "GOAL_REJECTED",
                "goal_gate_status": gate,
                "goal_published": True,
                "actuator_command_published": False,
                "execution_authorized": False,
                "readiness_snapshot": readiness_snapshot,
            }

        if not self.spin_until(
            lambda: self.bspline_count > baseline and self.bspline_rx is not None
            and self.bspline_rx >= sent_at,
            sent_at + self.timeout):
            return 4, {
                "result": "BSPLINE_NOT_GENERATED",
                "goal_gate_status": gate,
                "system_health": self.health,
                "goal_published": True,
                "actuator_command_published": False,
                "execution_authorized": False,
                "readiness_snapshot": readiness_snapshot,
            }

        summary = self.spline_summary(self.bspline)
        self.spin_until(
            lambda: self.feasibility_rx is not None and self.feasibility_rx >= self.bspline_rx,
            time.monotonic() + 1.0)
        result = {
            "result": "BSPLINE_GENERATED" if summary["valid"] else "BSPLINE_INVALID",
            "goal_map_xyz": [round(goal.pose.position.x, 4),
                             round(goal.pose.position.y, 4), 0.0],
            "goal_gate_status": gate,
            "bspline": summary,
            "hive2_candidate_feasibility": self.feasibility,
            "safety_status": {
                "status": safety_at_publish.get("status"),
                "operator_hold": safety_at_publish.get("operator_hold"),
                "actuator_output": safety_at_publish.get("actuator_output"),
                "safety_authorization": safety_at_publish.get("safety_authorization"),
                "execution_authorized": safety_at_publish.get("execution_authorized"),
            },
            "published_topic": "/move_base_simple/goal",
            "goal_distance_m": self.distance,
            "goal_published": True,
            "readiness_snapshot": readiness_snapshot,
            "readiness_wait_s": round(time.monotonic() - readiness_started, 3),
            "robot_pose_receive_age_s": round(robot_pose_receive_age_at_publish, 4),
            "robot_pose_stamp_age_s": round(robot_pose_stamp_age_at_publish, 4),
            "actuator_command_published": False,
            "operator_hold_changed": False,
            "execution_authorized": False,
        }
        return (0 if summary["valid"] else 5), result


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Generate one fail-closed, planning-only SCAN B-spline candidate.")
    parser.add_argument("--distance", type=float, default=0.5,
                        help="short forward test-goal distance in metres (0.3..1.5)")
    parser.add_argument("--timeout", type=float, default=15.0,
                        help="maximum wait for a new B-spline (5..60 seconds)")
    parser.add_argument("--readiness-timeout", type=float, default=120.0,
                        help="maximum wait for a fresh READY window (10..180 seconds)")
    parser.add_argument("--confirm-planning-only", action="store_true", required=True,
                        help="explicitly confirm this sends one planning goal only")
    args = parser.parse_args(argv)
    if not 0.3 <= args.distance <= 1.5:
        parser.error("--distance must be between 0.3 and 1.5 metres")
    if not 5.0 <= args.timeout <= 60.0:
        parser.error("--timeout must be between 5 and 60 seconds")
    if not 10.0 <= args.readiness_timeout <= 180.0:
        parser.error("--readiness-timeout must be between 10 and 180 seconds")

    rclpy.init(args=[])
    node = PlanningSmokeTest(args.distance, args.timeout, args.readiness_timeout)
    try:
        code, result = node.run_test()
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
        return code
    except KeyboardInterrupt:
        print(json.dumps({"result": "INTERRUPTED", "actuator_command_published": False}))
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
