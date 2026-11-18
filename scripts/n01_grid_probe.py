#!/usr/bin/env python3
"""Read-only probe for the projected occupancy-grid display path.

Prints one line per input so an empty navigation grid can be traced to its
source: reference cloud, live occupancy cloud, and the two OccupancyGrid topics.
No node is started, nothing is published, no motion is involved.
"""
import argparse
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def xyz(msg):
    if msg is None or msg.width == 0:
        return None
    n = msg.width * msg.height
    arr = np.frombuffer(msg.data, dtype=np.float32)
    step = msg.point_step // 4
    return arr.reshape(n, step)[:, :3]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", type=float, default=4.0, help="settle time in seconds")
    args = ap.parse_args()
    rclpy.init()
    node = Node("n01_grid_probe")
    q_grid = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    q_cloud = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
    data = {}

    def keep(key):
        def _cb(msg):
            data[key] = msg
        return _cb

    node.create_subscription(OccupancyGrid, "/grid_map/reference_grid", keep("ref_grid"), q_grid)
    node.create_subscription(OccupancyGrid, "/grid_map/live_grid", keep("live_grid"), q_grid)
    node.create_subscription(PointCloud2, "/grid_map/static_occupancy", keep("static_cloud"), q_grid)
    node.create_subscription(PointCloud2, "/grid_map/occupancy", keep("live_cloud"), q_cloud)
    node.create_subscription(PointCloud2, "/cloud", keep("scan"), q_cloud)
    node.create_subscription(PointCloud2, "/cloud_map", keep("cloud_map"), q_cloud)
    node.create_subscription(Odometry, "/body_pose", keep("body"), q_cloud)

    deadline = time.monotonic() + args.wait
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    def grid_line(key):
        msg = data.get(key)
        if msg is None:
            return f"{key}: 未收到（话题无数据）"
        arr = np.asarray(msg.data, dtype=np.int8)
        occ = int((arr == 100).sum())
        free = int((arr == 0).sum())
        unk = int((arr == -1).sum())
        return (f"{key}: {msg.info.width}x{msg.info.height} res={msg.info.resolution:.3f} "
                f"origin=({msg.info.origin.position.x:.2f},{msg.info.origin.position.y:.2f}) "
                f"occupied={occ} free={free} unknown={unk}")

    def cloud_line(key):
        msg = data.get(key)
        pts = xyz(msg)
        if pts is None:
            return f"{key}: 未收到或为空"
        z = pts[:, 2]
        return (f"{key}: {len(pts)} pts frame={msg.header.frame_id} "
                f"x[{pts[:,0].min():.2f},{pts[:,0].max():.2f}] "
                f"y[{pts[:,1].min():.2f},{pts[:,1].max():.2f}] "
                f"z[{z.min():.2f},{z.max():.2f}]")

    body = data.get("body")
    if body is not None:
        bx = body.pose.pose.position.x
        by = body.pose.pose.position.y
        bz = body.pose.pose.position.z
        print(f"body_pose: ({bx:.2f},{by:.2f},{bz:.2f})")
        for key in ("static_cloud", "live_cloud"):
            pts = xyz(data.get(key))
            if pts is None:
                continue
            d = np.hypot(pts[:, 0] - bx, pts[:, 1] - by)
            near = pts[d < 0.6]
            if len(near):
                print(f"  自车点检查 {key}: 0.6m 内 {len(near)} 点 "
                      f"z[{near[:,2].min():.2f},{near[:,2].max():.2f}] "
                      f"(机身 z={bz:.2f})")
            else:
                print(f"  自车点检查 {key}: 0.6m 内无点")
    print("=== 导航栅格显示链路 ===")
    for line in (grid_line("ref_grid"), grid_line("live_grid"),
                 cloud_line("static_cloud"), cloud_line("live_cloud"),
                 cloud_line("scan"), cloud_line("cloud_map")):
        print(line)
    print("判读：grid=0 且 static_cloud 有数据 -> 静态层没进规划窗口；"
          "grid=0 且 live_cloud 有数据 -> 命中点被高度带过滤；"
          "static_cloud/live_cloud 都没有 -> 导航桥没收到地图/扫描。")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
