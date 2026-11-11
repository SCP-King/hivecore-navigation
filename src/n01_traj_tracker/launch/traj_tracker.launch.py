"""N-01 trajectory tracker launch.

Separate from live_scan.launch.py on purpose: the planner stack must stay able
to run with zero actuator output. This launch is the only place that can publish
/cmd_vel, and it is gated by `enable:=false` by default.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("n01_traj_tracker"), "config", "traj_tracker.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable", default_value="false",
            description="Publish /cmd_vel. Keep false until the area is clear."),
        DeclareLaunchArgument(
            "max_vx", default_value="0.30"),
        DeclareLaunchArgument(
            "max_vy", default_value="0.30"),
        DeclareLaunchArgument(
            "max_vyaw", default_value="0.40"),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value="/cmd_vel"),

        Node(
            package="n01_traj_tracker",
            executable="traj_tracker_node",
            name="n01_traj_tracker",
            output="screen",
            parameters=[
                config,
                {
                    "enable": LaunchConfiguration("enable"),
                    "max_vx": LaunchConfiguration("max_vx"),
                    "max_vy": LaunchConfiguration("max_vy"),
                    "max_vyaw": LaunchConfiguration("max_vyaw"),
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                },
            ],
        ),
    ])
