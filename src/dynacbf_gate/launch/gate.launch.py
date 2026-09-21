#!/usr/bin/env python3
# ============================================================================
# gate.launch.py —— 定位就绪门控节点
# ============================================================================
# 用法：
#   ros2 launch dynacbf_gate gate.launch.py cmd_vel_in:=/cmd_vel_nav \
#       cmd_vel_out:=/cmd_vel localization_topic:=/localization_status
# 通常不单独启动，而是被 dynacbf_bringup/dynacbf_real.launch.py include。
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("dynacbf_gate")

    return LaunchDescription([
        DeclareLaunchArgument("cmd_vel_in", default_value="/cmd_vel_nav",
                              description="控制器输出（本节点输入）。"),
        DeclareLaunchArgument("cmd_vel_out", default_value="/cmd_vel",
                              description="底盘实际 /cmd_vel（本节点输出）。"),
        DeclareLaunchArgument("localization_topic", default_value="/localization_status",
                              description="甲方 SLAM 定位状态话题。"),
        DeclareLaunchArgument("max_fitness_mse", default_value="0.5",
                              description="配准拟合误差门限。"),

        Node(
            package="dynacbf_gate",
            executable="gate_node",
            name="gate_node",
            output="screen",
            parameters=[
                os.path.join(share, "config", "gate_params.yaml"),
                {
                    "cmd_vel_in": LaunchConfiguration("cmd_vel_in"),
                    "cmd_vel_out": LaunchConfiguration("cmd_vel_out"),
                    "localization_topic": LaunchConfiguration("localization_topic"),
                    "max_fitness_mse": LaunchConfiguration("max_fitness_mse"),
                },
            ],
        ),
    ])
