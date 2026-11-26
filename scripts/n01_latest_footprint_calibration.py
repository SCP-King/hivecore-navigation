#!/usr/bin/env python3
"""Calibrate the URDF steering hull against the latest running RSP model.

This is read-only with respect to ROS. It captures the live /robot_description
and the latest /joint_states, computes the convex footprint using the same
RobotProjection/steering_hull used by n01_robot_footprint, and writes a JSON
evidence file. Existing maps and safety parameters are not modified.
"""
import hashlib
import json
import math
import time
from pathlib import Path

from pathlib import Path as _N01Path
_N01_REPO = _N01Path(__file__).resolve().parents[1]
import sys as _n01_sys
for _p in (_N01_REPO / "src" / "n01_live_navigation",
           _N01_REPO / "install" / "n01_live_navigation" / "lib" / "python3.10" / "site-packages"):
    if _p.is_dir() and str(_p) not in _n01_sys.path:
        _n01_sys.path.insert(0, str(_p))

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def main() -> int:
    rclpy.init()
    node = Node("n01_latest_footprint_calibration")
    got = {}
    q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/robot_description",
                             lambda m: got.__setitem__("desc", m.data), q)
    positions = {}

    def on_joints(msg):
        for name, value in zip(msg.name, msg.position):
            if math.isfinite(float(value)):
                positions[name] = float(value)
    node.create_subscription(JointState, "/joint_states", on_joints,
                             qos_profile_sensor_data)
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and "desc" not in got:
        rclpy.spin_once(node, timeout_sec=0.1)
    # /joint_states has two publishers (legs and wheels); one --once sample is
    # not enough. Wait until both message families have been observed.
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if {"fl_leg_pitch1", "wheel_fl"} <= positions.keys():
            break
    if "desc" not in got or not {"fl_leg_pitch1", "wheel_fl"} <= positions.keys():
        print("missing /robot_description or fresh joint families", flush=True)
        rclpy.shutdown()
        return 2

    from n01_live_navigation.urdf_projection import RobotProjection
    from n01_live_navigation.robot_footprint import steering_hull

    xml = got["desc"]
    fresh = dict(positions)
    model = RobotProjection(xml)
    joints = {}
    for _, name, kind, _, _ in model.parents.values():
        if kind == "fixed":
            continue
        if kind == "continuous" or name.endswith("_ankle_yaw"):
            joints[name] = 0.0
        elif name in fresh:
            joints[name] = fresh[name]
        else:
            print("missing fresh joint:", name, flush=True)
            rclpy.shutdown()
            return 3
    polygon = steering_hull(model, joints)
    lo, hi = polygon.min(axis=0), polygon.max(axis=0)
    urdf_sha = hashlib.sha256(xml.encode()).hexdigest()
    out = {
        "schema": "n01.latest_footprint.v1",
        "captured_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "source": "live /robot_description + fresh /joint_states",
        "source_urdf_sha256": urdf_sha,
        "joint_values_rad": {k: v for k, v in sorted(joints.items())},
        "wheel_and_ankle_rule": "continuous wheels and ankle_yaw swept through full rotation",
        "footprint_vertices_base_link_m": [[float(x), float(y)] for x, y in polygon],
        "footprint_extent_m": [float(hi[0] - lo[0]), float(hi[1] - lo[1])],
        "footprint_min_xy_m": [float(lo[0]), float(lo[1])],
        "footprint_max_xy_m": [float(hi[0]), float(hi[1])],
        "environment_margin_m": 0.02,
        "note": "This capture does not modify map identity or safety limits.",
    }
    out_path = Path(__file__).resolve().parents[1] / "config" / "n01_latest_footprint.json"
    out_path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"wrote {out_path}")
    print(f"extent={out['footprint_extent_m']} min={out['footprint_min_xy_m']} max={out['footprint_max_xy_m']}")
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
