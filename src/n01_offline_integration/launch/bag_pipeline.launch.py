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
        DeclareLaunchArgument("duration_s",default_value="165.0"),
        DeclareLaunchArgument("max_clouds",default_value="2000"),
        DeclareLaunchArgument("scan_rate_hz",default_value="8.0"),
        DeclareLaunchArgument("scan_navi_mode",default_value="1"),
        DeclareLaunchArgument("scan_ground_height_m",default_value="0.0"),
        DeclareLaunchArgument("scan_body_height_m",default_value="0.4"),
        # Source-identified mapping TF is opt-in; legacy tests retain +0.90m.
        DeclareLaunchArgument("historical_mount_profile",
                              default_value="LEGACY_HIVE2_0P90"),
        Node(package="scan_planner",executable="scan_planner_node",
             name="scan_planner_node",parameters=[pcfg,{
                 "grid_map.frame_id":"map", "grid_map.cloud_is_world":True,
                 "grid_map.ground_height":ParameterValue(
                     LaunchConfiguration("scan_ground_height_m"),value_type=float),
                 "grid_map.body_height":ParameterValue(
                     LaunchConfiguration("scan_body_height_m"),value_type=float),
                 "fsm.navi_mode":ParameterValue(LaunchConfiguration("scan_navi_mode"),value_type=int),
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
        Node(package="n01_offline_integration",executable="n01_path_risk",
             name="n01_offline_path_risk",
             parameters=[{"use_sim_time":False}],output="screen"),
        Node(package="n01_offline_integration",executable="n01_g3_optimizer",
             name="n01_g3_offline_optimizer",
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
                 "max_clouds":ParameterValue(LaunchConfiguration("max_clouds"),value_type=int),
                 "scan_rate_hz":ParameterValue(LaunchConfiguration("scan_rate_hz"),value_type=float),
                 "max_map_tf_age_s":1.0,
                 "goal_forward_m":1.5,
                 "historical_mount_profile":ParameterValue(
                     LaunchConfiguration("historical_mount_profile"),value_type=str),
                 "use_sim_time":False
             }],output="screen"),
    ])
