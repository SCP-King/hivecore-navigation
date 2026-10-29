"""G1 observer: records actual map and SCAN Bspline topic evidence.

This is a limited smoke test, not proof of optimizer execution or continuous 3D
collision freedom. Never changes planner outputs or fakes PASS.
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
from sensor_msgs.msg import PointCloud2
from scan_planner_msgs.msg import Bspline


def valid_bspline(m):
    try:
        degree = int(m.order)
        points = m.pos_pts
        knots = m.knots
        valid = (1 <= degree <= 6 and len(points) > degree
                 and len(knots) == len(points) + degree + 1
                 and all(math.isfinite(k) for k in knots)
                 and all(a <= b for a, b in zip(knots, knots[1:]))
                 and knots[len(points)] > knots[degree]
                 and all(math.isfinite(v) for p in points for v in (p.x, p.y, p.z)))
        return bool(valid)
    except (ValueError, TypeError, OverflowError):
        return False


class G1Monitor(Node):
    def __init__(self):
        super().__init__("scan_g1_observer")
        self.start = time.monotonic()
        self.count_map = 0
        self.max_map_points = 0
        self.count_bspline = 0
        self.valid_bspline_count = 0
        self.bad_bspline_count = 0
        self.first_id = None
        self.create_subscription(PointCloud2, "/grid_map/occupancy_inflate",
                                 self.on_map, qos_profile_sensor_data)
        self.create_subscription(Bspline, "/planning/bspline", self.on_bspline, 10)
        self.create_timer(1.0, self.tick)
        self.finished = False

    def on_map(self, m):
        self.count_map += 1
        self.max_map_points = max(self.max_map_points, m.width * m.height)

    def on_bspline(self, m):
        self.count_bspline += 1
        if valid_bspline(m):
            self.valid_bspline_count += 1
            self.first_id = int(m.traj_id) if self.first_id is None else self.first_id
        else:
            self.bad_bspline_count += 1

    def tick(self):
        elapsed = time.monotonic() - self.start
        if elapsed < 40.0 or self.finished:
            return
        self.finished = True
        report = {
            "test_type": "G1_OFFLINE_SMOKE_ONLY",
            "seconds": round(elapsed, 2),
            "inflated_map_messages": self.count_map,
            "max_inflated_map_points": self.max_map_points,
            "bspline_messages": self.count_bspline,
            "valid_bspline_messages": self.valid_bspline_count,
            "invalid_bspline_messages": self.bad_bspline_count,
            "first_traj_id": self.first_id,
        }
        report["smoke_observation"] = "OBSERVED" if (self.max_map_points > 0 and self.valid_bspline_count > 0) else "NOT_OBSERVED"
        report["full_G1_acceptance"] = "NOT_TESTED"
        home = Path(os.environ.get("ROS_HOME", "/tmp"))
        out = home / "g1_smoke_report.json"
        home.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print("G1_OBSERVER_REPORT=" + json.dumps(report, ensure_ascii=False), flush=True)
        self.get_logger().info("G1 smoke report: %s" % out)
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
