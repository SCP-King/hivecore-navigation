from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('scan_topic', default_value='/livox/lidar/geo_filtered'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('n01_bringup'), 'rviz', 'n01_map_visualization.rviz'])),
        LogInfo(msg=[
            '[N-01 S3 map_visualization] Read-only vendor-map visualization. ',
            'This launch does not load a map, call GUI/SLAM services, publish TF, ',
            'or create /cmd_vel. It only starts the read-only perception path; ',
            'map localization and map publication must already be provided by the vendor.'
        ]),
        Node(
            package='n01_perception', executable='perception_node', name='n01_map_visualization',
            output='screen', parameters=[{
                'input_topic': LaunchConfiguration('scan_topic'),
                'target_frame': LaunchConfiguration('map_frame'),
            }]),
        Node(
            package='rviz2', executable='rviz2', name='n01_rviz', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')]),
    ])
