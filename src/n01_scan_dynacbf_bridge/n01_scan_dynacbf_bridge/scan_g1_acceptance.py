"""Real SCAN G1 offline acceptance scenarios; never publishes motion commands.

Scenario cases (separate planner instance per case):
  clear   : free straight route with distant/off-axis obstacles
  blocked : central 3D obstacle present BEFORE a goal, route must detour
  changed : obstacle inserted AFTER initial trajectory, a new safe plan required
  no_odom : goals/clouds without body odom must NOT yield a trajectory

Trajectory samples use the published cubic B-spline, not a synthetic planner.
Map samples use the actual /grid_map/occupancy_inflate PointCloud2, not raw cloud.
Offline evidence only: robot execution, lidar occlusion, PCD/Bag, dynamic MPC,
and physical collision safety are outside this acceptance.
"""
import argparse
import bisect
import json
import math
import os
from pathlib import Path
import struct
import time

if os.environ.get("ROS_DOMAIN_ID") != "88" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise RuntimeError("G1 acceptance requires ROS_DOMAIN_ID=88 and ROS_LOCALHOST_ONLY=1")
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from scan_planner_msgs.msg import Bspline
from .scan_g1_feed import xyz_cloud, test_obstacle_points
from .scan_g1_monitor import valid_bspline

from .g1_collision_geometry import (
    cloud_voxel_index, path_collision, path_margin_proximity,
)

def blocking_points():
    """Dense obstacle wall across nominal straight-line path."""
    return [(1.1 + ix * .06, iy * .07, z)
            for ix in range(7) for iy in range(-7, 8)
            for z in (.2, .4, .6, .8, 1.0, 1.2)]


def pointcloud_xyz(msg):
    fields = {f.name: f.offset for f in msg.fields}
    if not {"x", "y", "z"}.issubset(fields):
        return []
    endian = ">" if msg.is_bigendian else "<"
    get = struct.Struct(endian + "f")
    raw = msg.data
    xyz = []
    # Sample capped at 120k; do not let corrupted cloud consume unbounded time.
    count = min(int(msg.width) * int(msg.height), 120000)
    for i in range(count):
        base = i * msg.point_step
        if base + max(fields[k] for k in ("x", "y", "z")) + 4 > len(raw):
            break
        p = tuple(get.unpack_from(raw, base + fields[k])[0] for k in ("x", "y", "z"))
        if all(math.isfinite(v) for v in p):
            xyz.append(p)
    return xyz


def spline_at(control, knots, degree, t):
    """de Boor evaluator: B-spline order in SCAN message is polynomial degree."""
    n = len(control)
    t = min(max(t, knots[degree]), knots[n])
    span = min(n - 1, max(degree, bisect.bisect_right(knots, t) - 1))
    values = [list(control[span - degree + j]) for j in range(degree + 1)]
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            start = span - degree + j
            denom = knots[start + degree + 1 - r] - knots[start]
            alpha = (t - knots[start]) / denom if denom > 1e-12 else 0.0
            values[j] = [(1.0 - alpha) * values[j - 1][k] + alpha * values[j][k]
                         for k in range(3)]
    return tuple(values[degree])


def sample_spline(msg, count=121):
    pts = [(p.x, p.y, p.z) for p in msg.pos_pts]
    knots = list(msg.knots)
    degree = int(msg.order)
    a, b = knots[degree], knots[len(pts)]
    if b <= a:
        raise ValueError("non-positive spline time interval")
    return [spline_at(pts, knots, degree, a + (b - a) * j / (count - 1))
            for j in range(count)]




class G1Acceptance(Node):
    def __init__(self, scenario, report_path):
        super().__init__("scan_g1_accept_" + scenario)
        self.scenario = scenario
        self.report_path = Path(report_path)
        self.start = time.monotonic()
        self.max_seconds = 18.0 if scenario == "no_odom" else 42.0
        self.body_pub = self.create_publisher(Odometry, "/body_pose", qos_profile_sensor_data)
        self.sensor_pub = self.create_publisher(Odometry, "/sensor_pose", qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, "/cloud", qos_profile_sensor_data)
        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.create_subscription(PointCloud2, "/grid_map/occupancy_inflate",
                                 self.on_map, qos_profile_sensor_data)
        self.create_subscription(Bspline, "/planning/bspline", self.on_bspline, 10)
        self.clear_points = test_obstacle_points()
        self.block_points = self.clear_points + blocking_points()
        self.map_count = self.max_points = self.bs_count = 0
        self.invalid = 0
        self.blocked_map_seen = False
        self.blocked_map_s = None
        self.occupied = set()
        self.messages = []
        self.first_traj_id = None
        self.first_traj_s = None
        self.first_goal_s = None
        self.goal_count = 0
        self.goals_after_injection = 0
        self.last_goal = -999
        self.injected_s = None
        self.cmd_publishers_seen = set()
        self.cmd_graph_query_errors = 0
        self.done = False
        self.ticks = 0
        self.create_timer(.1, self.tick)
        self.get_logger().info("Acceptance case=%s, report=%s" % (scenario, report_path))

    def elapsed(self):
        return time.monotonic() - self.start

    def on_map(self, msg):
        pts = pointcloud_xyz(msg)
        self.map_count += 1
        self.max_points = max(self.max_points, len(pts))
        if pts:
            self.occupied = cloud_voxel_index(pts)
        if self.scenario in ("blocked", "changed") and not self.blocked_map_seen:
            center = sum(1 for x, y, z in pts
                         if .95 <= x <= 1.70 and abs(y) <= .65 and .28 <= z <= 1.20)
            if center >= 20:
                self.blocked_map_seen = True
                self.blocked_map_s = round(self.elapsed(), 2)
                self.get_logger().info("Blocking inflated voxels observed at +%.2fs" %
                                       self.blocked_map_s)

    def on_bspline(self, msg):
        self.bs_count += 1
        if not valid_bspline(msg):
            self.invalid += 1
            return
        try:
            sampled = sample_spline(msg)
        except (ValueError, IndexError, TypeError, OverflowError):
            self.invalid += 1
            return
        when = round(self.elapsed(), 2)
        if self.first_traj_id is None:
            self.first_traj_id = int(msg.traj_id)
            self.first_traj_s = when
            self.get_logger().info("first SCAN trajectory: id=%d at +%.2fs" %
                                   (self.first_traj_id, when))
        self.messages.append({
            "traj_id": int(msg.traj_id),
            "received_s": when,
            "path": sampled,
            "max_lateral_deviation_m": round(max(abs(p[1]) for p in sampled), 3),
            "max_vertical_deviation_m": round(max(abs(p[2] - .4) for p in sampled), 3),
        })

    def publish_pose(self, pub, stamp, z):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_link" if z == .4 else "sensor"
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.w = 1.0
        pub.publish(msg)

    def send_goal(self, stamp):
        goal = PoseStamped()
        goal.header.stamp = stamp
        goal.header.frame_id = "world"
        goal.pose.position.x = 2.2
        goal.pose.position.z = .4
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.goal_count += 1
        if self.scenario == "changed" and self.injected_s is not None:
            self.goals_after_injection += 1
        self.last_goal = self.elapsed()
        if self.first_goal_s is None:
            self.first_goal_s = round(self.last_goal, 2)
        self.get_logger().info("sent offline goal attempt=%d, +%.2fs" %
                               (self.goal_count, self.last_goal))

    def tick(self):
        if self.done:
            return
        self.ticks += 1
        now = self.elapsed()
        stamp = self.get_clock().now().to_msg()
        if self.scenario != "no_odom":
            self.publish_pose(self.body_pub, stamp, .4)
        self.publish_pose(self.sensor_pub, stamp, .8)
        active_block = (self.scenario == "blocked")
        if self.scenario == "changed" and self.first_traj_id is not None:
            if self.injected_s is None:
                self.injected_s = round(now, 2)
                self.get_logger().warning("Injecting central obstacle at +%.2fs" % now)
            active_block = True
        if self.ticks % 2 == 0:
            self.cloud_pub.publish(xyz_cloud(stamp, self.block_points if active_block
                                             else self.clear_points))
        ready = now >= 5.0
        if self.scenario == "blocked":
            ready = ready and self.blocked_map_seen
        # Changed scenario must test autonomous replanning, NOT a new goal.
        # Retry goal delivery only until first SCAN trajectory is observed.
        can_goal = self.scenario != "changed" or self.first_traj_id is None
        if ready and can_goal and (now - self.last_goal > 12.0) and self.goal_count < 3:
            self.send_goal(stamp)
        try:
            publishers = self.get_publishers_info_by_topic("/cmd_vel")
            for publisher in publishers:
                self.cmd_publishers_seen.add(publisher.node_namespace + "/" + publisher.node_name)
        except Exception as exc:
            self.cmd_graph_query_errors += 1
            self.get_logger().warning("cmd_vel graph query failed: %s" % exc)
        if now < self.max_seconds:
            return
        self.done = True
        self.finish()

    def finish(self):
        report = self.evaluate()
        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.report_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n",
                       encoding="utf-8")
        tmp.replace(self.report_path)
        print("G1_ACCEPTANCE_CASE=" + json.dumps(report, ensure_ascii=False), flush=True)
        self.get_logger().info("case=%s result=%s report=%s" %
                               (self.scenario, report["result"], self.report_path))
        rclpy.shutdown()

    def evaluate(self):
        issues = []
        if self.cmd_publishers_seen:
            issues.append("actual_cmd_vel_publishers_detected")
        if self.cmd_graph_query_errors:
            issues.append("cmd_vel_graph_unverifiable")
        if self.invalid:
            issues.append("malformed_spline_messages")
        if self.scenario == "no_odom":
            if self.bs_count:
                issues.append("trajectory_published_without_body_odometry")
            if not self.goal_count:
                issues.append("negative_goal_was_not_sent")
        else:
            if self.map_count == 0 or self.max_points == 0:
                issues.append("no_inflated_map")
            if not self.messages:
                issues.append("no_valid_real_scan_trajectory")
            if self.goal_count == 0:
                issues.append("goal_not_published")
        if self.scenario in ("blocked", "changed") and not self.blocked_map_seen:
            issues.append("no_evidence_blocking_obstacle_entered_inflated_map")

        examined = []
        candidates = []
        for msg in self.messages:
            if self.scenario == "changed":
                # Require map confirmation and a DISTINCT new trajectory ID.
                valid_time = (self.blocked_map_s is not None
                              and msg["received_s"] >= self.blocked_map_s + .5)
                new_id = self.first_traj_id is not None and msg["traj_id"] != self.first_traj_id
                if valid_time and new_id:
                    candidates.append(msg)
            elif self.scenario == "blocked":
                if self.blocked_map_s is not None and msg["received_s"] > self.blocked_map_s:
                    candidates.append(msg)
            else:
                candidates.append(msg)

        if self.scenario == "changed" and not candidates:
            issues.append("no_new_trajectory_after_obstacle_insertion")
        if self.scenario == "changed" and self.goals_after_injection:
            issues.append("test_contaminated_by_new_goal_after_obstacle")
        if self.scenario == "blocked" and not candidates:
            issues.append("no_trajectory_after_map_blocked")

        if self.scenario != "no_odom" and candidates and self.occupied:
            # Evaluate latest candidate against latest inflation map.
            for m in candidates[-5:]:
                collisions = path_collision(m["path"], self.occupied)
                proximity = path_margin_proximity(m["path"], self.occupied)
                examined.append({
                    "traj_id": m["traj_id"],
                    "received_s": m["received_s"],
                    "collision_samples": len(collisions),
                    "collision_examples": collisions[:6],
                    "supplementary_9cm_proximity_samples": len(proximity),
                    "max_lateral_deviation_m": m["max_lateral_deviation_m"],
                    "max_vertical_deviation_m": m["max_vertical_deviation_m"],
                    "path_start": [round(v, 3) for v in m["path"][0]],
                    "path_end": [round(v, 3) for v in m["path"][-1]],
                })
            if any(x["collision_samples"] > 0 for x in examined):
                issues.append("candidate_path_intersects_inflated_occupancy")
            if self.scenario == "blocked":
                detours = [m for m in examined if m["collision_samples"] == 0
                           and (m["max_lateral_deviation_m"] >= .22
                                or m["max_vertical_deviation_m"] >= .25)]
                if not detours:
                    issues.append("blocked_route_not_proven_to_detour_safely")
        elif self.scenario != "no_odom" and candidates and not self.occupied:
            issues.append("collision_analysis_unavailable_no_map_points")

        return {
            "test_type": "G1_OFFLINE_FULL_ACCEPTANCE_CASE",
            "scenario": self.scenario,
            "elapsed_s": round(self.elapsed(), 2),
            "result": "PASS" if not issues else "FAIL",
            "issues": issues,
            "map_messages": self.map_count,
            "max_inflated_map_points": self.max_points,
            "blocking_inflated_occupancy_seen": self.blocked_map_seen,
            "first_blocked_map_s": self.blocked_map_s,
            "goal_messages_sent": self.goal_count,
            "goals_after_obstacle_injection": self.goals_after_injection,
            "collision_metric": "SCAN double cylinder voxel occupancy (0.18m centre offsets)",
            "secondary_proximity_metric": "9cm sphere around trajectory centre; diagnostics only",
            "first_goal_s": self.first_goal_s,
            "bspline_messages": self.bs_count,
            "valid_bspline_messages": len(self.messages),
            "invalid_bspline_messages": self.invalid,
            "first_traj_id": self.first_traj_id,
            "first_traj_s": self.first_traj_s,
            "obstacle_inserted_s": self.injected_s,
            "checked_trajectories": examined,
            "cmd_vel_publishers_seen": sorted(self.cmd_publishers_seen),
            "cmd_vel_graph_query_errors": self.cmd_graph_query_errors,
            "scope": "OFFLINE_ONLY; no motor commands, no proof of physical/Bag safety",
        }


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=("clear", "blocked", "changed", "no_odom"),
                        required=True)
    parser.add_argument("--report", required=True)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = G1Acceptance(options.scenario, options.report)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
