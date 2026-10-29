"""REAL recorded Bag -> reconstructed map pointcloud -> SCAN, offline only.

No synthetic sensor fixture. No raw bag play, raw TF publishers, or chassis I/O.
The historical map-based replay does not satisfy LocalizationStatus tracking.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    if (os.environ.get("ROS_DOMAIN_ID") != "88" or
            os.environ.get("ROS_LOCALHOST_ONLY") != "1" or
            os.environ.get("N01_HISTORICAL_BAG_ONLY") != "1"):
        raise RuntimeError("Historical Bag pipeline requires isolated explicit mode")
    pcfg=os.path.join(get_package_share_directory("scan_planner"),"config","planner.yaml")
    bag=LaunchConfiguration("bag_path")
    duration=LaunchConfiguration("duration_s")
    return LaunchDescription([
        DeclareLaunchArgument("bag_path"),
        DeclareLaunchArgument("duration_s",default_value="95.0"),
        Node(package="scan_planner",executable="scan_planner_node",
             name="scan_planner_node",parameters=[pcfg,{
                 "grid_map.frame_id":"map", "grid_map.cloud_is_world":True,
                 "use_sim_time":False
             }],output="screen"),
        Node(package="n01_scan_dynacbf_bridge",executable="scan_bspline_bridge",
             name="scan_bspline_bridge",parameters=[{
                 "frame_id":"map","bspline_topic":"/planning/bspline",
                 "odom_topic":"/body_pose","path_topic":"/nav/test/global_path",
                 "path_points":120,"arc_step":.05,"table_samples":400,
                 "odom_timeout_s":1.5,"trajectory_timeout_s":3.0,
                 "use_sim_time":False
             }],output="screen"),
        Node(package="n01_offline_integration",executable="n01_dynamic_tracker",
             name="n01_offline_dynamic_tracker",
             parameters=[{"use_sim_time":False}],output="screen"),
        Node(package="n01_offline_integration",executable="n01_dynamic_port",
             name="n01_dynamic_avoidance_interface",
             parameters=[{"use_sim_time":False}],output="screen"),
        Node(package="n01_offline_integration",executable="n01_offline_observer",
             name="n01_offline_integration_observer",parameters=[{
                 "historical_bag_mode":True,
                 "duration_s":ParameterValue(duration,value_type=float),
                 "report_path":EnvironmentVariable("N01_OFFLINE_REPORT_PATH",default_value=""),
                 "use_sim_time":False
             }],output="screen"),
        Node(package="n01_offline_integration",executable="n01_historical_bag",
             name="n01_historical_bag_replay",parameters=[{
                 "bag_path":ParameterValue(bag,value_type=str),
                 "max_clouds":180,
                 "scan_rate_hz":4.0,
                 "max_map_tf_age_s":1.0,
                 "goal_forward_m":1.5,
                 "use_sim_time":False
             }],output="screen"),
    ])
