"""G4.6 gated historical Bag + SCAN mode3 + G4.5 research reference.

Only replayed lidar/odometry and diagnostic Path publications.
Never publishes /cmd_vel or a robot control topic. The reference is
UNVERIFIED and may be unsuitable even for simulated robot motion.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    required={
        "ROS_DOMAIN_ID":"88", "ROS_LOCALHOST_ONLY":"1",
        "N01_HISTORICAL_BAG_ONLY":"1", "N01_G4_RESEARCH_ENABLE":"1"
    }
    if any(os.environ.get(key)!=value for key,value in required.items()):
        raise RuntimeError(
            "G4.6 requires ROS_DOMAIN_ID=88, localhost, HISTORICAL_BAG_ONLY, "
            "G4_RESEARCH_ENABLE=1; never use with physical robot")
    share=get_package_share_directory("n01_offline_integration")
    original=os.path.join(share,"launch","bag_pipeline.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("bag_path"),
        DeclareLaunchArgument("reference_file"),
        DeclareLaunchArgument("report_path",default_value=""),
        DeclareLaunchArgument("duration_s",default_value="165.0"),
        DeclareLaunchArgument("max_clouds",default_value="2000"),
        DeclareLaunchArgument("historical_mount_profile",
                              default_value="LEGACY_HIVE2_0P90"),
        DeclareLaunchArgument("scan_rate_hz",default_value="8.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(original),
            launch_arguments={
                "bag_path":LaunchConfiguration("bag_path"),
                "duration_s":LaunchConfiguration("duration_s"),
                "max_clouds":LaunchConfiguration("max_clouds"),
                "scan_rate_hz":LaunchConfiguration("scan_rate_hz"),
                "scan_navi_mode":"3",
                "historical_mount_profile":LaunchConfiguration("historical_mount_profile"),
            }.items()),
        Node(package="n01_offline_integration",
             executable="n01_g4_chain_observer",
             name="n01_g4_chain_observer",
             parameters=[{
                 "reference_file":ParameterValue(
                     LaunchConfiguration("reference_file"),value_type=str),
                 "report_path":ParameterValue(
                     LaunchConfiguration("report_path"),value_type=str),
                 "duration_s":ParameterValue(
                     LaunchConfiguration("duration_s"),value_type=float),
                 "use_sim_time":False,
             }],output="screen"),
        Node(package="n01_offline_integration",
             executable="n01_g4_reference",
             name="n01_g4_research_reference",
             parameters=[{
                 "reference_file":ParameterValue(
                     LaunchConfiguration("reference_file"),value_type=str),
                 "max_start_offset_m":.75,
                 "scan_body_height_m":.4,
                 "max_start_vertical_offset_m":.5,
                 "use_sim_time":False,
             }],output="screen"),
    ])
