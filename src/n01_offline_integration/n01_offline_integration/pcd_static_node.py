"""Optional static factory PCD visualization: DISPLAY ONLY, never SCAN input.

A user-supplied PCD map has NO authoritative ROS frame in its header. The
N01_PCD_MAP_FRAME_ASSERTED gate is an operator assertion, NOT proof of
registration. The node does not publish TF or any robot control.
"""
import json
import os
from pathlib import Path
import struct

if (os.environ.get("ROS_DOMAIN_ID") != "88" or
        os.environ.get("ROS_LOCALHOST_ONLY") != "1" or
        os.environ.get("N01_HISTORICAL_BAG_ONLY") != "1" or
        os.environ.get("N01_PCD_MAP_FRAME_ASSERTED") != "1"):
    raise RuntimeError("PCD preview requires isolated historical replay and explicit map-frame assertion")

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from .pcd_static_preview import sample_pcd


class StaticPcdPreview(Node):
    def __init__(self):
        super().__init__("n01_static_pcd_preview")
        path = os.environ.get("N01_STATIC_PCD_PATH")
        if not path:
            raise RuntimeError("N01_STATIC_PCD_PATH must be explicitly supplied")
        points, meta = sample_pcd(path)
        # PointCloud2 XYZ32 only; preserve actual PCD coordinates without any
        # translation/rotation. ROS frame naming does not validate registration.
        packed = bytearray(12 * len(points))
        for i, xyz in enumerate(points):
            struct.pack_into("<fff", packed, i * 12, *xyz)
        self.msg = PointCloud2()
        self.msg.header.frame_id = "map"
        self.msg.height = 1
        self.msg.width = len(points)
        self.msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self.msg.is_bigendian = False
        self.msg.point_step = 12
        self.msg.row_step = 12 * len(points)
        self.msg.is_dense = True
        self.msg.data = bytes(packed)
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/n01/offline/static_pcd_map", qos)
        self.status_pub = self.create_publisher(
            String, "/n01/offline/static_pcd_preview_status", qos)
        self.status = {
            "schema": "n01.offline.pcd_preview.v1",
            "status": "STATIC_PCD_VISUALIZATION_ONLY",
            "map_frame_assumption": "USER_ASSERTED_NOT_VALIDATED_AGAINST_BAG",
            "source_file": str(Path(path).expanduser().resolve()),
            "source_points": meta["count"],
            "sampled_points": meta["sampled_points"],
            "sample_stride": meta["sample_stride"],
            "bounds_xyz_m": meta["bounds_xyz_m"],
            "pcd_format": meta["format"],
            "used_as_scan_planner_input": False,
            "robot_localization_verified": False,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
        report = os.environ.get("N01_PCD_PREVIEW_REPORT_PATH")
        if report:
            # Parent run directory is private to this demo; never overwrite.
            with Path(report).open("x", encoding="utf-8") as file:
                json.dump(self.status, file, indent=2, ensure_ascii=False)
                file.write("\n")
        self.sent = False
        self.create_timer(1.0, self.publish_once)
        self.get_logger().warning(
            "STATIC PCD IS A VIEW-ONLY BACKGROUND; frame agreement with "
            "recorded Bag must be manually checked. Not a global planner.")

    def publish_once(self):
        if not self.sent:
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.cloud_pub.publish(self.msg)
            self.status_pub.publish(
                String(data=json.dumps(self.status, ensure_ascii=False)))
            self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = StaticPcdPreview()
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
