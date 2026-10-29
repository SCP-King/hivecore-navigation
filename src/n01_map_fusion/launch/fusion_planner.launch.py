import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('n01_map_fusion')
    default_params = os.path.join(share, 'config', 'fusion_planner.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params',
                              default_value=default_params,
                              description='fusion/planner 参数 yaml'),
        DeclareLaunchArgument('pcd_path',
                              default_value='/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        LogInfo(msg=(
            'n01_map_fusion: 静态PCD + 实时占用融合 → A* 离线验证。'
            '前提: /grid_map/static_occupancy(n01_static_map) 与 /grid_map/occupancy'
            '(n01_perception→n01_scan, map 帧) 在跑;乙方不做 TF 假设。'
            '不发布 /cmd_vel*。')),
        Node(package='n01_static_map', executable='static_map_node',
             name='static_map_node',
             parameters=[{'pcd_path': LaunchConfiguration('pcd_path')}]),
        # 注意: fusion/planner 不覆盖节点名(node() 默认名 n01_map_fusion /
        # n01_path_planner),yaml 顶层 key 按该名匹配:
        #   - n01_static_map 顶层? 不,static_map_node 的参数只走 pcd_path overrides。
        #   - fusion_planner.yaml 顶层 = n01_map_fusion 与 n01_path_planner。
        Node(package='n01_map_fusion', executable='fusion_node',
             parameters=[LaunchConfiguration('params'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='n01_map_fusion', executable='planner_node',
             parameters=[LaunchConfiguration('params'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])