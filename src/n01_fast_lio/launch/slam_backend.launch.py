"""Backend selector that leaves the vendor SLAM launch/interface untouched.

vendor: start no replacement node; continue using the vendor's own launch.
fast_lio: start isolated FAST-LIO2 mapping and read-only odometry adapter.
The FAST-LIO private map is never silently aliased to the vendor map.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fast_launch = PathJoinSubstitution([
        FindPackageShare("n01_fast_lio"), "launch", "fast_lio_mapping.launch.py"])
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="vendor",
                              description="vendor or fast_lio; vendor is default"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "map_file_path",
            default_value="/home/demo/lyy/n01_runtime/slam/fast_lio_map.pcd"),
        DeclareLaunchArgument("rviz", default_value="false"),
        LogInfo(msg=[
            "[N-01 SLAM backend selector] vendor mode preserves the vendor launch, ",
            "topics, message types and TF contract; it does not start a second SLAM."
        ], condition=IfCondition(
            LaunchConfiguration("backend").equals("vendor"))),
        IncludeLaunchDescription(
            __import__("launch").launch_description_sources.PythonLaunchDescriptionSource(
                fast_launch),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_file_path": LaunchConfiguration("map_file_path"),
                "rviz": LaunchConfiguration("rviz"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("backend").equals("fast_lio"))),
        Node(
            package="n01_fast_lio",
            executable="n01_fast_lio_odometry_adapter",
            name="n01_fast_lio_odometry_adapter",
            output="screen",
            parameters=[{
                "input_topic": "/n01/fast_lio/odometry",
                "output_topic": "/n01/slam/odometry",
                "diagnostics_topic": "/n01/slam/diagnostics",
                "expected_frame": "n01_fast_lio_map",
                "expected_child_frame": "imu_link",
                "max_age_s": 0.5,
            }],
            condition=IfCondition(LaunchConfiguration("backend").equals("fast_lio"))),
    ])
