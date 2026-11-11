from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _prepare_map_output(context):
    map_path = os.path.expanduser(
        LaunchConfiguration("map_file_path").perform(context))
    parent = Path(map_path).resolve().parent
    parent.mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("n01_fast_lio"), "config", "hive2_mid360.yaml"
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("n01_fast_lio"), "rviz", "fast_lio.rviz"
    ])

    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        namespace="/n01/fast_lio",
        name="mapping",
        output="screen",
        parameters=[
            config,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
                "map_file_path": LaunchConfiguration("map_file_path"),
                "common.publish_tf": False,
                "common.map_frame_id": LaunchConfiguration("map_frame_id"),
                "common.imu_frame_id": "imu_link",
            },
        ],
        # Upstream uses absolute topic names. Remap every output into an
        # N-01-only namespace; raw Livox input topics are read-only/shared.
        remappings=[
            ("/Odometry", "/n01/fast_lio/odometry"),
            ("/path", "/n01/fast_lio/path"),
            ("/cloud_registered", "/n01/fast_lio/cloud_registered"),
            ("/cloud_registered_body", "/n01/fast_lio/cloud_registered_body"),
            ("/cloud_effected", "/n01/fast_lio/cloud_effected"),
            ("/Laser_map", "/n01/fast_lio/map"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="n01_fast_lio_rviz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use ROS clock for isolated rosbag replay."),
        DeclareLaunchArgument(
            "map_file_path",
            default_value="/home/demo/lyy/n01_runtime/slam/fast_lio_map.pcd",
            description="Absolute output path used by /n01/fast_lio/map_save."),
        DeclareLaunchArgument(
            "map_frame_id", default_value="n01_fast_lio_map",
            description="FastLIO map frame; use map for independent N-01 localization."),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Start the optional FAST-LIO private-frame RViz view."),
        LogInfo(msg=[
            "[N-01 FAST-LIO2] Mapping/odometry backend only. ",
            "Reads /livox/lidar CustomMsg + /livox/imu. ",
            "Outputs are isolated under /n01/fast_lio; TF broadcast is OFF. ",
            "Vendor SLAM topics, launch files and TF authorities are untouched. ",
            "This launch does not provide saved-map localization/relocalization."
        ]),
        OpaqueFunction(function=_prepare_map_output),
        fast_lio,
        rviz,
    ])
