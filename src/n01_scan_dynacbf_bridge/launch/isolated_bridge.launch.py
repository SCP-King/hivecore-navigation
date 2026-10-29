"""Isolation-only launch. No robot controllers or actual /cmd_vel."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '88' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Isolated tests require ROS_DOMAIN_ID=88 and ROS_LOCALHOST_ONLY=1')
    cfg = os.path.join(get_package_share_directory('n01_scan_dynacbf_bridge'), 'config', 'isolated.yaml')
    return LaunchDescription([
        Node(package='n01_scan_dynacbf_bridge', executable='scan_bspline_bridge',
             name='scan_bspline_bridge', parameters=[cfg], output='screen'),
        Node(package='n01_scan_dynacbf_bridge', executable='dcbf_test_guard',
             name='dcbf_test_guard', parameters=[cfg], output='screen'),
    ])
