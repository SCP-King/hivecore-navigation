#!/usr/bin/env python3
"""Read-only ROS 2 graph probe without persistent ros2cli daemon."""
import json
import os
import sys
import time

if os.environ.get("ROS_DOMAIN_ID") != "88" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    sys.exit("G1 graph probe refuses non-isolated ROS environment")
import rclpy


def endpoint(entry):
    return {
        "node": entry.node_namespace.rstrip("/") + "/" + entry.node_name,
        "type": entry.topic_type,
        "reliability": str(entry.qos_profile.reliability),
        "durability": str(entry.qos_profile.durability),
    }


def main():
    rclpy.init(args=None)
    name = "n01_g1_graph_probe_" + str(os.getpid())
    node = rclpy.create_node(name, enable_rosout=False)
    try:
        time.sleep(1.5)  # Allow DDS discovery before sampling the graph.
        nodes = [
            {"name": n, "namespace": ns}
            for n, ns in node.get_node_names_and_namespaces()
            if n != name
        ]
        watched = [
            "/planning/bspline", "/grid_map/occupancy_inflate",
            "/cmd_vel", "/nav/test/cmd_vel"
        ]
        publishers = {
            topic: [endpoint(p) for p in node.get_publishers_info_by_topic(topic)]
            for topic in watched
        }
        subscriptions = {
            topic: [endpoint(p) for p in node.get_subscriptions_info_by_topic(topic)]
            for topic in watched
        }
        output = {
            "graph_probe": name,
            "nodes": nodes,
            "topics": node.get_topic_names_and_types(),
            "publishers": publishers,
            "subscriptions": subscriptions,
        }
        print(json.dumps(output, indent=2, ensure_ascii=False))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
