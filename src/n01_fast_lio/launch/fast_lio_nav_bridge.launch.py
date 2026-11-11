"""FAST-LIO2 + N-01 adapters into navigation inputs.

- /robot_pose + /loc_status come from n01_fast_lio_pose_adapter (after ICP)
- n01_localization_adapter normalizes them to N-01 pose/status
- n01_live_scan_bridge consumes cloud_registered_body(imu_link) + chassis /odom
  and publishes /cloud + /body_pose for navigation
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("n01_fast_lio"), "launch", "fast_lio_localization.launch.py"
        ])),
        launch_arguments={
            "rviz": "false",
            "saved_map_path": LaunchConfiguration("saved_map_path"),
            "map_file_path": LaunchConfiguration("fastlio_map_path"),
        }.items(),
    )
    adapter = Node(
        package="n01_live_navigation",
        executable="n01_localization_adapter",
        name="n01_localization_adapter",
        output="screen",
        parameters=[{
            "status_input_topic": "/loc_status",
            "status_output_topic": "/n01/localization/status",
            "pose_input_topic": "/robot_pose",
            "pose_output_topic": "/n01/localization/pose",
            "expected_pose_frame": "map",
            "expected_pose_child_frame": "base_footprint",
        }],
    )
    bridge = Node(
        package="n01_live_navigation",
        executable="n01_live_scan_bridge",
        name="n01_live_scan_bridge",
        output="screen",
        parameters=[{
            "status_topic": "/n01/localization/status",
            "pose_topic": "/n01/localization/pose",
            "cloud_topic": "/n01/fast_lio/cloud_registered_body",
            "expected_cloud_frame": "imu_link",
            "odom_topic": "/odom",
            "odom_frame": "odom",
            "odom_child_frame": "base_link",
            "map_frame": "map",
            "body_frame": "base_footprint",
            "planner_body_frame": "base_link",
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "saved_map_path",
            default_value="/home/demo/lyy/slam_data/maps/mapping_lite_20261109_140536_087.pcd"),
        DeclareLaunchArgument(
            "fastlio_map_path",
            default_value="/home/demo/lyy/n01_runtime/slam/fast_lio_map.pcd"),
        LogInfo(msg=(
            "[N-01 FAST-LIO2] bridge: cloud_registered_body(imu_link) + chassis /odom "
            "-> /cloud + /body_pose; localization requires ICP alignment.")),
        localization,
        adapter,
        bridge,
    ])
