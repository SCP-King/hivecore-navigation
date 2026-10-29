"""Isolated SCAN planner only; no synthetic localization or chassis controller."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '88' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Offline SCAN launch requires ROS_DOMAIN_ID=88 + ROS_LOCALHOST_ONLY=1')
    config = os.path.join(get_package_share_directory('scan_planner'), 'config', 'planner.yaml')
    return LaunchDescription([
        Node(package='scan_planner', executable='scan_planner_node',
             name='scan_planner_node', parameters=[config], output='screen'),
    ])
