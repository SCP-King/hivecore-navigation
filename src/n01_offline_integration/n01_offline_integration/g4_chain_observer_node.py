"""G4.7 independently observes research initial_path -> native SCAN -> G2/G3.

ROS_DOMAIN_ID=88 and N01_HISTORICAL_BAG_ONLY are mandatory.
No motion topics are created. This is NOT robot authorization.
"""
import json
import os
import time
from pathlib import Path as FilePath

if (os.environ.get("ROS_DOMAIN_ID") != "88" or
    os.environ.get("ROS_LOCALHOST_ONLY") != "1" or
    os.environ.get("N01_HISTORICAL_BAG_ONLY") != "1" or
    os.environ.get("N01_G4_RESEARCH_ENABLE") != "1"):
    raise RuntimeError("G4 observer restricted to isolated historical research")

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path as RosPath
from scan_planner_msgs.msg import Bspline
from n01_scan_dynacbf_bridge.scan_g1_monitor import valid_bspline
from .g4_chain_evidence import ChainEvidence
from .g4_reference_node import load_reference


class G4ChainObserver(Node):
    def __init__(self):
        super().__init__("n01_g4_chain_observer")
        self.declare_parameter("reference_file", "")
        self.declare_parameter("report_path", "")
        self.declare_parameter("duration_s", 165.0)
        self.declare_parameter("require_static_overlay", False)
        reference = self.get_parameter("reference_file").value
        report = self.get_parameter("report_path").value or os.environ.get("N01_G4_CHAIN_REPORT_PATH", "")
        self.duration = float(self.get_parameter("duration_s").value)
        if not reference or not report or not 10 <= self.duration <= 180:
            raise ValueError("G4 observer requires reference, report_path and bounded duration")
        self.chain = ChainEvidence(load_reference(reference))
        self.chain.static_pcd_required = bool(
            self.get_parameter("require_static_overlay").value)
        self.output = FilePath(report)
        self.start = time.monotonic()
        self.saved = False
        self.create_subscription(RosPath, "/initial_path", self.on_reference_path, 10)
        if self.chain.static_pcd_required:
            latched = QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(
                Bool, "/grid_map/static_overlay_ready", self.on_static_overlay, latched)
        self.create_subscription(String, "/n01/offline/g4_reference_status", self.on_status, 10)
        self.create_subscription(Bspline, "/planning/bspline", self.on_bspline, 10)
        self.create_subscription(RosPath, "/nav/test/global_path", self.on_bridge_path, 10)
        self.create_subscription(String, "/n01/offline/dynamic_tracks", self.on_dynamic, 10)
        self.create_subscription(String, "/n01/offline/g3_solver_report", self.on_g3, 10)
        self.create_subscription(RosPath, "/n01/offline/g3_candidate_path", self.on_g3_path, 10)
        self.create_timer(.5, self.tick)

    def on_static_overlay(self, msg):
        self.chain.native_static_pcd_status(bool(msg.data))

    def on_reference_path(self, msg):
        self.chain.initial_path(
            msg.header.frame_id.lstrip("/"),
            [[p.pose.position.x, p.pose.position.y, p.pose.position.z]
             for p in msg.poses])

    def on_status(self, msg):
        try:
            self.chain.reference_status(json.loads(msg.data))
        except (ValueError, TypeError):
            pass

    def on_bspline(self, msg):
        self.chain.native_bspline(bool(valid_bspline(msg)))

    def on_bridge_path(self, msg):
        if msg.header.frame_id.lstrip("/") == "map" and msg.poses:
            self.chain.bridge_path()

    def on_dynamic(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("actuator_output", "DISABLED") == "DISABLED":
                self.chain.dynamic_report()
            else:
                self.chain.cmd_vel_seen = True
        except (ValueError, TypeError, AttributeError):
            pass

    def on_g3(self, msg):
        try:
            data = json.loads(msg.data)
            if (data.get("actuator_output") == "DISABLED" and
                data.get("safety_authorization") == "NEVER"):
                self.chain.g3_report()
            else:
                self.chain.cmd_vel_seen = True
        except (ValueError, TypeError, AttributeError):
            pass

    def on_g3_path(self, msg):
        if msg.header.frame_id.lstrip("/") == "map" and len(msg.poses) > 1:
            self.chain.g3_path()

    def tick(self):
        if self.saved:
            return
        try:
            if self.get_publishers_info_by_topic("/cmd_vel"):
                self.chain.cmd_vel_seen = True
        except Exception:
            self.chain.graph_query_failed = True
        if time.monotonic() - self.start < self.duration:
            return
        report = self.chain.finalize()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        with self.output.open("x", encoding="utf-8") as file:
            json.dump(report, file, ensure_ascii=False, indent=2, allow_nan=False)
            file.write("\n")
        self.saved = True
        self.get_logger().warning("G4.7 observation result: " + report["result"])


def main(args=None):
    rclpy.init(args=args)
    node = G4ChainObserver()
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
