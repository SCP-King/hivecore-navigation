import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = "n01_static_map"
    default_config = PathJoinSubstitution([
        get_package_share_directory(pkg), "config", "static_map.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument(
            "pcd_path",
            default_value="/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd",
            description="甲方 SLAM 静态 PCD 绝对路径(只读)"),
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("resolution", default_value="0.05"),
        LogInfo(msg=[
            "[n01_static_map] 只读静态地图供给层启动。"
            "加载甲方 SLAM PCD 为 map 帧静态占用 PointCloud2 -> /grid_map/static_occupancy。"
            "注意:动态险情(车辆/行人)不在本层,仍需 /livox/lidar/geo_filtered "
            "实时链路 + /local_pcd(乙方不接管动态感知)。"
            "PCD 坐标系与甲方 map 帧一致性需甲方定位确认(不臆造 TF)。",
        ]),
        Node(
            package=pkg, executable="static_map_node", name="n01_static_map",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "pcd_path": LaunchConfiguration("pcd_path"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "resolution": LaunchConfiguration("resolution"),
                },
            ],
        ),
    ])