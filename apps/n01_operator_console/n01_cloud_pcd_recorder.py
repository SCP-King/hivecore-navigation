#!/usr/bin/env python3
"""Accumulate /cloud_registered into a voxel-deduplicated PCD.

FAST-LIO's own pcd_save path segfaults on this machine when the accumulated
cloud is large (observed: `catch sig 2` then `Segmentation fault`, no file).
This recorder keeps its own voxel hash and writes an ASCII PCD on SIGINT/SIGTERM,
so the map survives even though the LIO process does not do the saving.
"""
import argparse
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CloudRecorder(Node):
    def __init__(self, out_path: str, voxel: float) -> None:
        super().__init__("n01_cloud_pcd_recorder")
        self.out_path = out_path
        self.voxel = max(voxel, 1e-3)
        self.acc = {}
        self.frames = 0
        self.saved = False

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(PointCloud2, "/cloud_registered", self.on_cloud, qos)
        self.create_timer(10.0, self.report)
        self.get_logger().warn(f"recording /cloud_registered -> {out_path}")

    def on_cloud(self, msg: PointCloud2) -> None:
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True)
        except Exception:
            return
        if pts.size == 0:
            return
        self.frames += 1
        inv = 1.0 / self.voxel
        acc = self.acc
        for row in pts:
            x = float(row[0])
            y = float(row[1])
            z = float(row[2])
            acc[(int(round(x * inv)), int(round(y * inv)), int(round(z * inv)))] = (x, y, z)

    def report(self) -> None:
        self.get_logger().info(
            f"frames={self.frames} voxels={len(self.acc)}")

    def dump(self, signum=None, frame=None) -> None:
        if self.saved:
            return
        self.saved = True
        n = len(self.acc)
        if n == 0:
            print("no points accumulated, nothing written", flush=True)
            raise SystemExit(1)
        with open(self.out_path, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {n}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {n}\n")
            f.write("DATA ascii\n")
            for x, y, z in self.acc.values():
                f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")
        print(f"SAVED {n} points ({self.frames} frames) -> {self.out_path}", flush=True)
        raise SystemExit(0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--voxel", type=float, default=0.05)
    args = ap.parse_args()

    rclpy.init()
    node = CloudRecorder(args.out, args.voxel)
    signal.signal(signal.SIGINT, node.dump)
    signal.signal(signal.SIGTERM, node.dump)
    try:
        rclpy.spin(node)
    except SystemExit:
        raise
    except Exception:
        pass
    finally:
        try:
            node.dump()
        except SystemExit:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
