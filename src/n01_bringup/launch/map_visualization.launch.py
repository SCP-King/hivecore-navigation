from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('mode', default_value='map_localized'),
        DeclareLaunchArgument('scan_topic', default_value='/livox/lidar/geo_filtered'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('n01_bringup'), 'rviz', 'n01_map_visualization.rviz'])),
        LogInfo(msg=[
            '[N-01 S4 map_visualization] World-coordinate SCAN occupancy map (map_localized). '
            'Starts ONLY the read-only perception + scan_grid_map chain and optional rviz. '
            'Does NOT start A*, B-spline planners, DynaCBF, MPC, Controller, Gate, '
            'any /cmd_vel publisher, 甲方 SLAM Console/GUI services, or 乙方 TF broadcaster.'
        ]),
        Node(
            package='n01_perception', executable='perception_node', name='n01_perception',
            output='screen', parameters=[{
                'input_topic': LaunchConfiguration('scan_topic'),
                'target_frame': LaunchConfiguration('map_frame'),
            }]),
        Node(
            package='n01_scan', executable='scan_grid_map', name='n01_scan_grid_map',
            output='screen', parameters=[
                PathJoinSubstitution([
                    FindPackageShare('n01_scan'), 'config', 'scan_map.yaml']),
                {
                    'mode': LaunchConfiguration('mode'),
                    'map_frame': LaunchConfiguration('map_frame'),
                    'lidar_frame': 'lidar_link',
                    'input_topic': '/local_pcd',
                },
            ]),
        Node(
            package='rviz2', executable='rviz2', name='n01_rviz', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')]),
    ])