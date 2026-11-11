"""FAST-LIO2 live mapping -> normalized N-01 localization/navigation inputs.

This mode defines the current FAST-LIO session origin as ``map``.  It does not
load a historical PCD and therefore must never be mixed with offline/localized
navigation data from another map session.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("n01_fast_lio"), "launch", "fast_lio_mapping.launch.py"
        ])),
        launch_arguments={
            "map_frame_id": "n01_fast_lio_map",
            "rviz": "false",
            "map_file_path": LaunchConfiguration("map_file_path"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    pose_adapter = Node(
        package="n01_fast_lio", executable="n01_fast_lio_pose_adapter",
        name="n01_fast_lio_pose_adapter", output="screen",
        parameters=[{
            "input_topic": "/n01/fast_lio/odometry",
            "pose_topic": "/robot_pose", "status_topic": "/loc_status",
            "map_frame": "map", "fastlio_map_frame": "n01_fast_lio_map",
            "base_frame": "base_footprint", "imu_frame": "imu_link",
            "require_alignment": False, "publish_tf": True,
        }],
    )
    adapter = Node(
        package="n01_live_navigation", executable="n01_localization_adapter",
        name="n01_localization_adapter", output="screen",
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
        package="n01_live_navigation", executable="n01_live_scan_bridge",
        name="n01_live_scan_bridge", output="screen",
        parameters=[{
            "status_topic": "/n01/localization/status",
            "pose_topic": "/n01/localization/pose",
            "cloud_topic": "/n01/fast_lio/cloud_registered_body",
            "expected_cloud_frame": "imu_link",
            "odom_topic": "/odom", "odom_frame": "odom",
            "odom_child_frame": "base_link", "map_frame": "map",
            "body_frame": "base_footprint", "planner_body_frame": "base_link",
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "map_file_path",
            default_value="/home/demo/lyy/n01_runtime/slam/fast_lio_realtime_map.pcd"),
        LogInfo(msg=(
            "[N-01 FAST-LIO2 realtime] live session origin is map; mapping, "
            "normalized pose/status and SCAN cloud bridge are active together.")),
        mapping, pose_adapter, adapter, bridge,
    ])
