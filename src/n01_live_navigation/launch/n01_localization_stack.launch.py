"""Unified localization data path: desk -> adapter -> SCAN bridge.

One click in the GUI starts the vendor localization desk (sensors, IMU filter,
static TF, cloud preprocess, MOLA localization) plus the N-01 adapter and the
live SCAN bridge. The bridge output (/cloud, /body_pose) is exactly the
navigation input; the navigation launch then only starts static map + planner.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desk = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("hivecore_slam3d_lite_console"),
                "launch", "localization_lite_desk.launch.py",
            ])
        ]),
        launch_arguments={
            "launch_robot_state_publisher": "false",
            "static_tf_params_file": LaunchConfiguration("static_tf_params_file"),
            "localization_params_file": LaunchConfiguration("localization_params_file"),
        }.items(),
    )

    adapter = Node(
        package="n01_live_navigation", executable="n01_localization_adapter",
        name="n01_localization_adapter",
        parameters=[os.path.join(
            get_package_share_directory("n01_live_navigation"),
            "config", "slam3d_lite_adapter.yaml")],
        output="screen",
    )

    bridge = Node(
        package="n01_live_navigation", executable="n01_live_scan_bridge",
        name="n01_live_scan_bridge",
        parameters=[{
            "status_topic": "/n01/localization/status",
            "cloud_topic": "/livox/lidar/geo_filtered",
            "pose_topic": "/n01/localization/pose",
            "odom_topic": "/odom", "odom_frame": "odom",
            "odom_child_frame": "base_link",
            "expected_cloud_frame": "lidar_link",
            "map_frame": "map", "body_frame": "base_footprint",
            "planner_body_frame": "base_link",
            "max_status_age_s": 1.0, "max_pose_rx_age_s": 0.7,
            "max_pose_stamp_age_s": 1.5,
            "max_pose_anchor_delta_s": 1.25,
            "max_live_body_pose_anchor_delta_s": 1.4,
            "pose_cache_size": 64,
            "max_odom_rx_age_s": 0.2,
            "max_odom_stamp_age_s": 0.4,
            "odom_interpolation_max_gap_s": 0.08,
            "odom_cache_size": 512,
            "max_cloud_age_s": 0.9,
            "max_latest_scan_lag_s": 0.7,
            "body_pose_publish_period_s": 0.05,
            "max_live_body_pose_age_s": 0.25,
            "max_tf_wait_s": 0.55, "tf_retry_period_s": 0.02,
            "max_pending_clouds": 1, "max_points": 100000,
            "max_point_range_m": 12.0, "use_sim_time": False,
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "static_tf_params_file",
            default_value="/home/demo/lyy/hivecore-navigation/config/n01_static_tf_params.yaml"),
        DeclareLaunchArgument(
            "localization_params_file",
            default_value="/home/demo/lyy/n01_slam_gui/config/n01_localization_lite.yaml"),
        ExecuteProcess(
            cmd=["bash", "/home/demo/lyy/hivecore-navigation/scripts/start_livox_driver.sh"],
            output="screen",
        ),
        desk,
        adapter,
        bridge,
    ])
