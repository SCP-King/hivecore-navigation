"""Diagnostic G1 observer for actual SCAN 3D map + published Bspline.

Reports evidence only; a ROS smoke observation is NOT full G1 acceptance.
Never publishes any robot command.
"""
import json
import math
import os
from pathlib import Path
import time

if os.environ.get("ROS_DOMAIN_ID") != "88" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise RuntimeError("G1 observer requires ROS_DOMAIN_ID=88 and ROS_LOCALHOST_ONLY=1")

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from scan_planner_msgs.msg import Bspline


def valid_bspline(msg):
    try:
        degree = int(msg.order)
        pts = msg.pos_pts
        knots = msg.knots
        return bool(
            1 <= degree <= 6
            and len(pts) > degree
            and len(knots) == len(pts) + degree + 1
            and all(math.isfinite(k) for k in knots)
            and all(a <= b for a, b in zip(knots, knots[1:]))
            and knots[len(pts)] > knots[degree]
            and all(math.isfinite(v) for pt in pts for v in (pt.x, pt.y, pt.z))
        )
    except (ValueError, TypeError, OverflowError):
        return False


class G1Monitor(Node):
    def __init__(self):
        super().__init__("scan_g1_observer")
        self.declare_parameter("max_duration_s", 90.0)
        self.declare_parameter("settle_after_valid_s", 8.0)
        self.max_duration = float(self.get_parameter("max_duration_s").value)
        self.settle = float(self.get_parameter("settle_after_valid_s").value)
        if not (10.0 <= self.max_duration <= 300.0 and 1.0 <= self.settle <= 120.0
                and all(math.isfinite(x) for x in (self.max_duration, self.settle))):
            raise ValueError("invalid monitor duration")
        self.start = time.monotonic()
        self.max_map_points = self.map_count = 0
        self.traj_count = self.valid_count = self.bad_count = 0
        self.cloud_count = self.goal_count = self.odom_count = 0
        self.first_valid_time = None
        self.first_traj_id = None
        self.first_bspline_time = None
        self.last_bspline_time = None
        self.first_goal_time = None
        self.graph_seen_publishers = set()
        self.graph_seen_cmd_vel = set()
        self.graph_last_bspline_publishers = []
        self.graph_last_cmd_vel_publishers = []
        self.graph_last_topic_types = []
        self.done = False
        self.create_subscription(PointCloud2, "/grid_map/occupancy_inflate",
                                 self.on_map, qos_profile_sensor_data)
        self.create_subscription(Bspline, "/planning/bspline", self.on_bspline, 10)
        self.create_subscription(PointCloud2, "/cloud", self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/body_pose", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/move_base_simple/goal", self.on_goal, 10)
        self.create_timer(1.0, self.tick)
        self.get_logger().info("G1 monitor active: deadline %.0fs; observing /planning/bspline" % self.max_duration)

    def seconds(self):
        return round(time.monotonic() - self.start, 2)

    def on_map(self, msg):
        self.map_count += 1
        self.max_map_points = max(self.max_map_points, msg.width * msg.height)

    def on_cloud(self, msg):
        self.cloud_count += 1

    def on_odom(self, msg):
        self.odom_count += 1

    def on_goal(self, msg):
        self.goal_count += 1
        if self.first_goal_time is None:
            self.first_goal_time = self.seconds()
            self.get_logger().info("First goal observed at +%.2fs" % self.first_goal_time)

    def on_bspline(self, msg):
        self.traj_count += 1
        t = self.seconds()
        if self.first_bspline_time is None:
            self.first_bspline_time = t
            self.get_logger().info("First Bspline received at +%.2fs; id=%d, ctrl_pts=%d" %
                                   (t, msg.traj_id, len(msg.pos_pts)))
        self.last_bspline_time = t
        if valid_bspline(msg):
            self.valid_count += 1
            if self.first_valid_time is None:
                self.first_valid_time = time.monotonic()
                self.first_traj_id = int(msg.traj_id)
        else:
            self.bad_count += 1
            self.get_logger().error("Malformed Bspline at +%.2fs" % t)

    def endpoint_records(self, topic):
        records = []
        for info in self.get_publishers_info_by_topic(topic):
            records.append({
                "node_name": info.node_name,
                "namespace": info.node_namespace,
                "topic_type": info.topic_type,
                "reliability": str(info.qos_profile.reliability),
                "durability": str(info.qos_profile.durability),
            })
        return records

    def tick(self):
        if self.done:
            return
        try:
            publishers = self.endpoint_records("/planning/bspline")
            cmd_publishers = self.endpoint_records("/cmd_vel")
            self.graph_last_bspline_publishers = publishers
            self.graph_last_cmd_vel_publishers = cmd_publishers
            self.graph_last_topic_types = [
                {"name": topic, "types": types}
                for topic, types in self.get_topic_names_and_types()
                if topic.endswith("bspline") or topic.endswith("cmd_vel")
            ]
            for ep in publishers:
                self.graph_seen_publishers.add(ep["namespace"] + "/" + ep["node_name"])
            for ep in cmd_publishers:
                self.graph_seen_cmd_vel.add(ep["namespace"] + "/" + ep["node_name"])
        except Exception as ex:
            self.get_logger().warning("ROS graph query: %s" % ex)
        elapsed = time.monotonic() - self.start
        done_valid = (self.first_valid_time is not None
                      and elapsed >= 15.0
                      and time.monotonic() - self.first_valid_time >= self.settle)
        if not done_valid and elapsed < self.max_duration:
            if int(elapsed) % 10 == 0:
                self.get_logger().info("G1 +%.0fs cloud=%d goal=%d map=%d max_pts=%d bspline=%d pubs=%d" %
                                       (elapsed, self.cloud_count, self.goal_count,
                                        self.map_count, self.max_map_points,
                                        self.traj_count, len(self.graph_last_bspline_publishers)))
            return
        self.done = True
        result = {
            "test_type": "G1_OFFLINE_DIAGNOSTIC_SMOKE",
            "seconds": self.seconds(),
            "cloud_messages": self.cloud_count,
            "odom_messages": self.odom_count,
            "goal_messages": self.goal_count,
            "first_goal_s": self.first_goal_time,
            "inflated_map_messages": self.map_count,
            "max_inflated_map_points": self.max_map_points,
            "bspline_messages": self.traj_count,
            "valid_bspline_messages": self.valid_count,
            "invalid_bspline_messages": self.bad_count,
            "first_bspline_s": self.first_bspline_time,
            "last_bspline_s": self.last_bspline_time,
            "first_traj_id": self.first_traj_id,
            "bspline_publishers_seen": sorted(self.graph_seen_publishers),
            "bspline_publishers_at_end": self.graph_last_bspline_publishers,
            "cmd_vel_publishers_seen": sorted(self.graph_seen_cmd_vel),
            "cmd_vel_publishers_at_end": self.graph_last_cmd_vel_publishers,
            "topic_type_snapshot": self.graph_last_topic_types,
        }
        if self.graph_seen_cmd_vel:
            result["smoke_observation"] = "UNSAFE_GRAPH"
        elif self.max_map_points > 0 and self.valid_count > 0:
            result["smoke_observation"] = "OBSERVED"
        else:
            result["smoke_observation"] = "NOT_OBSERVED"
        result["full_G1_acceptance"] = "NOT_TESTED"
        report = Path(os.environ["ROS_HOME"]) / "g1_smoke_report.json"
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("G1_OBSERVER_REPORT=" + json.dumps(result, ensure_ascii=False), flush=True)
        self.get_logger().info("G1 diagnostic report written: %s" % report)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = G1Monitor()
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
