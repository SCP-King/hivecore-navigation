from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    input_frame = LaunchConfiguration('input_frame')
    target_frame = LaunchConfiguration('target_frame')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/livox/lidar/geo_filtered'),
        DeclareLaunchArgument('input_frame', default_value='lidar_link'),
        DeclareLaunchArgument('target_frame', default_value='lidar_link'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('n01_bringup'), 'rviz', 'n01_sensor_only.rviz'])),
        LogInfo(msg=[
            '[N-01 S3 sensor_only] Read-only. ',
            'geo_filtered requires vendor GUI-SLAM/preprocessor with enable_forward=true; ',
            'Does not assume /odom in the recorded Hive2 map->base_footprint TF tree. '
            'Sensor-only defaults to lidar_link; map-localized mode needs stamped TF. ',
            'No SLAM service, TF broadcaster, planner, controller, or /cmd_vel is started. ',
            'No fallback to raw /livox/lidar is performed.'
        ]),
        Node(
            package='n01_scan', executable='scan_validator', name='n01_scan_validator',
            output='screen', parameters=[{
                'input_topic': input_topic, 'input_frame': input_frame,
            }]),
        Node(
            package='n01_scan', executable='scan_grid_map', name='n01_scan_grid_map',
            output='screen', parameters=[{
                'mode': 'sensor_only', 'input_topic': '/local_pcd', 'map_frame': target_frame,
            }]),
        Node(
            package='n01_perception', executable='perception_node', name='n01_perception',
            output='screen', parameters=[{
                'input_topic': input_topic, 'target_frame': target_frame,
            }]),
        Node(
            package='rviz2', executable='rviz2', name='n01_rviz',
            output='screen', condition=IfCondition(rviz),
            arguments=['-d', [LaunchConfiguration('rviz_config')]]),
    ])
