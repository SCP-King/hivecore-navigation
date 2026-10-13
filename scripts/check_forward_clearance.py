#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_forward_clearance.py —— 下目标前的前方净空检查（只读）

读 /curr_state（[x, y, yaw]，导航系）与 /local_pcd（导航系点云），统计机器人
正前方走廊内的点数与最近距离，用来判断"能不能往前开 N 米"。

用法:
  python3 scripts/check_forward_clearance.py [--length 1.0] [--half-width 0.5] [--sec 4]

输出:
  走廊定义、点云总数、走廊内点数、最近距离、按距离分带的点数。
  判读经验: 走廊内 0.6 m 以内点数为 0 ⇒ 前方基本空旷，可下短距离目标。
"""
import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray

try:
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:
    pc2 = None


class Clearance(Node):
    def __init__(self, length, half_width):
        super().__init__("forward_clearance")
        self.length = length
        self.half_width = half_width
        self.state = None
        self.pts = None
        # /local_pcd 是普通可靠话题；/curr_state 也是
        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32MultiArray, "/curr_state",
                                 self._cb_state, qos)
        self.create_subscription(PointCloud2, "/local_pcd", self._cb_cloud, qos)

    def _cb_state(self, msg):
        if len(msg.data) >= 3 and all(math.isfinite(v) for v in msg.data[:3]):
            self.state = [float(v) for v in msg.data[:3]]

    def _cb_cloud(self, msg):
        if pc2 is None:
            return
        try:
            arr = pc2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True)
        except Exception:
            return
        if arr.size:
            self.pts = np.asarray(arr, dtype=np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--length", type=float, default=1.0,
                    help="走廊长度 [m]（从机器人当前位置沿朝向往前）")
    ap.add_argument("--half-width", type=float, default=0.5,
                    help="走廊半宽 [m]")
    ap.add_argument("--sec", type=float, default=4.0,
                    help="采集时长 [s]")
    args = ap.parse_args()

    if pc2 is None:
        print("ERROR: 缺少 sensor_msgs_py，无法解析 PointCloud2")
        return 2

    rclpy.init()
    node = Clearance(args.length, args.half_width)
    end = node.get_clock().now().nanoseconds + int(args.sec * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    state, pts = node.state, node.pts
    node.destroy_node()
    rclpy.shutdown()

    if state is None:
        print("ERROR: /curr_state 无数据 —— 上游位姿/控制器不通")
        return 3
    if pts is None or pts.size == 0:
        print("ERROR: /local_pcd 无数据 —— 感知不通")
        return 4

    x0, y0, yaw = state
    c, s = math.cos(yaw), math.sin(yaw)
    dx = pts[:, 0] - x0
    dy = pts[:, 1] - y0
    # 世界 → 机体（前向 = +x_body）
    fx = c * dx + s * dy           # 前向距离
    fy = -s * dx + c * dy          # 左向距离
    d = np.hypot(dx, dy)

    # 前方走廊：0 <= fx <= length 且 |fy| <= half_width，且 z 在可能碰撞高度带内
    z = pts[:, 2]
    in_band = (z > -0.35) & (z < 1.5)          # 排除地面以下与高空
    in_corr = in_band & (fx > 0.0) & (fx <= args.length) & \
        (np.abs(fy) <= args.half_width)

    print("=== 前方净空检查 ===")
    print("机器人位姿      : x=%.3f y=%.3f yaw=%.1f° (导航系)" %
          (x0, y0, math.degrees(yaw)))
    print("走廊定义        : 前方 0–%.2f m，半宽 ±%.2f m，z∈(-0.35, 1.5)" %
          (args.length, args.half_width))
    print("点云总点数      : %d" % pts.shape[0])

    n_corr = int(in_corr.sum())
    print("走廊内点数      : %d" % n_corr)
    if n_corr:
        fxc = fx[in_corr]
        print("走廊内最近距离  : %.3f m" % float(fxc.min()))
    else:
        print("走廊内最近距离  : 无点（空旷）")

    print("--- 按前向距离分带（走廊内）---")
    for lo, hi in ((0.0, 0.3), (0.3, 0.6), (0.6, 1.0), (1.0, 1.5), (1.5, 2.0)):
        m = in_corr & (fx >= lo) & (fx < hi)
        print("  %.1f–%.1f m : %d" % (lo, hi, int(m.sum())))

    print("--- 以机器人为心、半径 1.5 m 内的最近障碍（全向）---")
    m = (z > -0.35) & (z < 1.5) & (d < 1.5)
    if m.any():
        print("  最近距离: %.3f m（全向）" % float(d[m].min()))
    else:
        print("  1.5 m 内无障碍点")

    ok = n_corr == 0 or float(fx[in_corr].min()) > 0.8
    print("判读            : %s" %
          ("前方 0.8 m 内空旷，可下短距离目标" if ok
           else "⚠️ 前方 0.8 m 内有障碍，请缩短/改向后再下目标"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
