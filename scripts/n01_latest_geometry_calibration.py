#!/usr/bin/env python3
"""Capture latest-mesh footprint and vertical geometry evidence.

Requires a healthy stack: live /robot_description, all leg/wheel joints,
/body_pose and /n01/ground_reference/status. Read-only with respect to ROS.
"""
import json, math, time
from pathlib import Path
from collections import defaultdict

import numpy as np
from pathlib import Path as _N01Path
_N01_REPO = _N01Path(__file__).resolve().parents[1]
import sys as _n01_sys
for _p in (_N01_REPO / "src" / "n01_live_navigation",
           _N01_REPO / "install" / "n01_live_navigation" / "lib" / "python3.10" / "site-packages"):
    if _p.is_dir() and str(_p) not in _n01_sys.path:
        _n01_sys.path.insert(0, str(_p))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node("n01_latest_geometry_calibration")
    got = {"pos": {}}
    q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_joints(m):
        for n, v in zip(m.name, m.position):
            if math.isfinite(float(v)):
                got["pos"][n] = float(v)
    node.create_subscription(String, "/robot_description", lambda m: got.__setitem__("desc", m.data), q)
    node.create_subscription(JointState, "/joint_states", on_joints, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/body_pose", lambda m: got.__setitem__("body", m), qos_profile_sensor_data)
    node.create_subscription(String, "/n01/ground_reference/status", lambda m: got.__setitem__("ground", m.data), 10)

    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and "desc" not in got:
        rclpy.spin_once(node, timeout_sec=.1)
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline and not {"fl_leg_pitch1", "wheel_fl"} <= got["pos"].keys():
        rclpy.spin_once(node, timeout_sec=.1)
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline and not {"body", "ground"} <= got.keys():
        rclpy.spin_once(node, timeout_sec=.1)

    if "desc" not in got or not {"body", "ground"} <= got.keys():
        print("missing /robot_description, /body_pose or ground status", flush=True)
        rclpy.shutdown(); return 2

    from n01_live_navigation.urdf_projection import RobotProjection
    from n01_live_navigation.robot_footprint import steering_hull
    model = RobotProjection(got["desc"])
    joints = {}
    for _, name, kind, _, _ in model.parents.values():
        if kind == "fixed": continue
        if kind == "continuous" or name.endswith("_ankle_yaw"): joints[name] = 0.
        elif name in got["pos"]: joints[name] = got["pos"][name]
        else:
            print("missing joint", name, flush=True); rclpy.shutdown(); return 3
    footprint = steering_hull(model, joints)
    model.transforms_for(joints)
    all_z = []
    for link, t, points in model.shapes:
        f = model.transforms[link] @ t
        p = points @ f[:3, :3].T + f[:3, 3]
        all_z.append((float(p[:, 2].min()), float(p[:, 2].max()), link))
    min_z, _, min_link = min(all_z, key=lambda x: x[0])
    max_z = max(item[1] for item in all_z)
    body = got["body"].pose.pose.position
    ground = json.loads(got["ground"])
    plane = ground.get("plane") or {}
    a, b, c = (float(plane.get(k, 0.)) for k in ("a", "b", "c"))
    floor = a * body.x + b * body.y + c
    out = {
        "schema": "n01.latest_geometry.v1",
        "captured_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "source": "live /robot_description + /joint_states + /body_pose + ground status",
        "source_urdf_sha256": __import__("hashlib").sha256(got["desc"].encode()).hexdigest(),
        "joint_values_rad": dict(sorted(joints.items())),
        "footprint_extent_m": [float(np.ptp(footprint[:,0])), float(np.ptp(footprint[:,1]))],
        "body_pose_xyz_m": [float(body.x), float(body.y), float(body.z)],
        "ground_plane_abc": [a, b, c],
        "ground_z_at_body_xy_m": floor,
        "body_min_z_rel_base_link_m": min_z,
        "body_max_z_rel_base_link_m": max_z,
        "lowest_link": min_link,
        "wheel_bottom_map_z_m": float(body.z + min_z),
        "wheel_bottom_vs_floor_m": float(body.z + min_z - floor),
        "body_to_ground_m": float(body.z - floor),
        "suggested_static_ego_filter_z_down_m": float(abs(min_z) + .02),
        "suggested_static_ego_filter_z_up_m": float(max_z + .02),
        "suggested_static_ego_filter_radius_m": float(np.linalg.norm(footprint, axis=1).max()),
        "environment_margin_m": .02,
        "note": "evidence only; safety launch values are not modified",
    }
    path = Path(__file__).resolve().parents[1] / "config" / "n01_latest_geometry.json"
    path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print("wrote", path)
    for k in ("footprint_extent_m", "body_pose_xyz_m", "ground_z_at_body_xy_m",
              "body_min_z_rel_base_link_m", "body_max_z_rel_base_link_m",
              "wheel_bottom_map_z_m", "wheel_bottom_vs_floor_m"):
        print(k, out[k])
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
