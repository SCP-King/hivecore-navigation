#!/usr/bin/env python3
"""Read-only Livox graph preflight: 0=reuse, 10=start, 2=conflict/error."""
import time



def decision(endpoints, same_name_count):
    if len(endpoints) > 1 or same_name_count > 1:
        return 2, 'ERROR: duplicate Livox nodes/publishers; inspect original launch owners before restarting.'
    if endpoints:
        if endpoints[0].topic_type != 'sensor_msgs/msg/PointCloud2':
            return 2, 'ERROR: existing /livox/lidar publisher is not PointCloud2; vendor localization requires PointCloud2.'
        return 0, 'Reusing existing /livox/lidar PointCloud2 publisher; no second Livox driver is started.'
    if same_name_count:
        return 2, 'ERROR: Livox node exists without lidar publisher; refusing duplicate driver startup.'
    return 10, 'No Livox driver found; starting vendor PointCloud2 driver.'


def main():
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node('n01_livox_start_preflight')
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        endpoints = node.get_publishers_info_by_topic('/livox/lidar')
        count = sum(name == 'livox_lidar_publisher' for name, _ in node.get_node_names_and_namespaces())
        code, message = decision(endpoints, count)
        print(message, flush=True)
        for endpoint in endpoints:
            name = endpoint.node_namespace.rstrip('/') + '/' + endpoint.node_name
            print(f'  {name}: {endpoint.topic_type}', flush=True)
        return code
    except Exception as exc:
        print(f'ERROR: Livox graph preflight failed: {exc}', flush=True)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
