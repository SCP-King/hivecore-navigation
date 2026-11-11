"""FAST-LIO2 independent mapping + saved-map localization for N-01."""
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
            "rviz": LaunchConfiguration("rviz"),
            "map_file_path": LaunchConfiguration("map_file_path"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    relocalizer = Node(
        package="n01_fast_lio",
        executable="n01_fast_lio_relocalizer",
        name="n01_fast_lio_relocalizer",
        output="screen",
        parameters=[{
            "map_file_path": LaunchConfiguration("saved_map_path"),
            "map_frame": "map",
            "fastlio_map_frame": "n01_fast_lio_map",
            "base_frame": "base_footprint",
            "imu_frame": "imu_link",
            "cloud_topic": "/n01/fast_lio/cloud_registered_body",
            "odom_topic": "/n01/fast_lio/odometry",
            "initialpose_topic": "/initialpose",
            "alignment_topic": "/n01/fast_lio/map_alignment",
        }],
    )
    adapter = Node(
        package="n01_fast_lio",
        executable="n01_fast_lio_pose_adapter",
        name="n01_fast_lio_pose_adapter",
        output="screen",
        parameters=[{
            "input_topic": "/n01/fast_lio/odometry",
            "pose_topic": "/robot_pose",
            "status_topic": "/loc_status",
            "map_frame": "map",
            "fastlio_map_frame": "n01_fast_lio_map",
            "base_frame": "base_footprint",
            "imu_frame": "imu_link",
            "alignment_topic": "/n01/fast_lio/map_alignment",
            "require_alignment": True,
            "publish_tf": True,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "map_file_path",
            default_value="/home/demo/lyy/n01_runtime/slam/fast_lio_map.pcd"),
        DeclareLaunchArgument(
            "saved_map_path",
            default_value="/home/demo/lyy/slam_data/maps/mapping_lite_20261109_140536_087.pcd"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        LogInfo(msg=(
            "[N-01 FAST-LIO2] independent mapping+localization: saved PCD ICP "
            "requires /initialpose; only after ICP success publish /robot_pose, "
            "/loc_status=tracking and map->base_footprint TF.")),
        mapping,
        relocalizer,
        adapter,
    ])
