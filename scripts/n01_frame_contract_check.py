#!/usr/bin/env python3
"""Read-only check of the unified N-01 TF contract.

Run on the robot after sourcing ROS 2 and the workspace:

    source /opt/ros/humble/setup.bash
    source ~/lyy/hivecore-navigation/install/setup.bash
    python3 scripts/n01_frame_contract_check.py

The script only reads TF; it never publishes commands, goals or TF.
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

EXPECTED_BF_BL_Z = 0.90
EXPECTED_BL_LIDAR_X = 0.295
WHEEL_RADIUS = 0.095
EXPECTED_BODY_TO_GROUND = 0.778
TOL = 0.03


def xyz(transform):
    t = transform.transform.translation
    return (float(t.x), float(t.y), float(t.z))


def main() -> int:
    rclpy.init()
    node = Node("n01_frame_contract_check")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    del listener  # keep the listener alive through the node
    # Static base_footprint->base_link arrives immediately; map->base_link and
    # the leg chains are dynamic and need a short spin window before lookup.
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (buffer.can_transform("base_footprint", "base_link", Time())
                and buffer.can_transform("map", "base_link", Time())
                and buffer.can_transform("base_link", "fl_foot_link", Time())):
            break

    def lookup(parent, child):
        try:
            return xyz(buffer.lookup_transform(
                parent, child, Time(), timeout=Duration(seconds=0.2)))
        except (TransformException, ValueError):
            return None

    bf_bl = lookup("base_footprint", "base_link")
    bl_lidar = lookup("base_link", "lidar_link")
    map_bl = lookup("map", "base_link")
    map_bf = lookup("map", "base_footprint")
    feet = {name: lookup("base_link", f"{name}_foot_link")
            for name in ("fl", "fr", "rl", "rr")}
    livox = lookup("base_link", "livox_frame")

    print("=== N-01 unified frame contract (read-only) ===")
    print(f"base_footprint -> base_link : {bf_bl}")
    print(f"base_link      -> lidar_link: {bl_lidar}")
    print(f"map -> base_footprint       : {map_bf}")
    print(f"map -> base_link            : {map_bl}")
    for name, value in feet.items():
        print(f"base_link -> {name}_foot_link : {value}")
    print(f"base_link -> livox_frame    : {livox}")

    failures = []
    if bf_bl is None:
        failures.append("base_footprint->base_link missing")
    elif abs(bf_bl[2] - EXPECTED_BF_BL_Z) > TOL:
        failures.append(
            f"base_footprint->base_link z={bf_bl[2]:.3f}, expected {EXPECTED_BF_BL_Z:.3f}")
    if bl_lidar is None:
        failures.append("base_link->lidar_link missing")
    elif abs(bl_lidar[0] - EXPECTED_BL_LIDAR_X) > TOL:
        failures.append(
            f"base_link->lidar_link x={bl_lidar[0]:.3f}, expected {EXPECTED_BL_LIDAR_X:.3f}")
    if map_bl is None:
        failures.append("map->base_link missing (SLAM localization not running?)")
    if all(value is not None for value in feet.values()):
        body_z = map_bl[2] if map_bl is not None else None
        # feet are base_link-relative; bring the lowest wheel contact into map.
        lowest_foot_z = min(value[2] for value in feet.values())
        body_to_ground = WHEEL_RADIUS - lowest_foot_z
        print(f"base_link -> lowest foot z  : {lowest_foot_z:.3f}")
        print(f"body_to_ground (leg+wheels) : {body_to_ground:.3f} "
              f"(expected {EXPECTED_BODY_TO_GROUND:.3f})")
        if body_z is not None:
            ground_z = body_z - body_to_ground
            print(f"local ground z in map       : {ground_z:.3f}")
            if abs(body_to_ground - EXPECTED_BODY_TO_GROUND) > 0.05:
                failures.append(
                    f"body_to_ground={body_to_ground:.3f}, expected "
                    f"{EXPECTED_BODY_TO_GROUND:.3f}; re-check leg pose")
    else:
        print("WARNING: one or more *_foot_link frames are missing")
    if livox is None:
        failures.append(
            "base_link->livox_frame missing; restart the localization/mapping desk "
            "so static_tf_params_file=config/n01_static_tf_params.yaml is applied")

    if failures:
        print("\nFAIL:")
        for item in failures:
            print("  -", item)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print("\nPASS: unified TF contract checks passed")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
