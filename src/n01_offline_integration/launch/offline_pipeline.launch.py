"""Integrated isolated offline SCAN pipeline; never starts robot/SLAM or /cmd_vel.

fixture:=true launches synthetic TF/status/cloud source; default false is
strict status mode and waits for real recorded inputs from an isolated replay.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID") != "88" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Requires ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1")
    base = get_package_share_directory("n01_offline_integration")
    planner = get_package_share_directory("scan_planner")
    cfg = os.path.join(base, "config", "offline_bridge.yaml")
    pcfg = os.path.join(planner, "config", "planner.yaml")
    fixture = LaunchConfiguration("fixture")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim = ParameterValue(use_sim_time, value_type=bool)
    flag = ParameterValue(fixture, value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("fixture", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("duration_s", default_value="75.0"),
        Node(package="n01_offline_integration", executable="n01_offline_bridge",
             name="n01_offline_sensor_bridge", parameters=[cfg, {
                 "fixture_mode":flag, "use_sim_time":sim
             }], output="screen"),
        Node(package="n01_offline_integration", executable="n01_offline_fixture",
             name="n01_offline_fixture_source", condition=IfCondition(fixture),
             parameters=[{"use_sim_time":sim}], output="screen"),
        Node(package="scan_planner", executable="scan_planner_node",
             name="scan_planner_node", parameters=[pcfg, {
                 "grid_map.frame_id":"map", "grid_map.cloud_is_world":True,
                 "use_sim_time":sim
             }], output="screen"),
        # B-spline reference path only; no DynaCBF actuator controller.
        Node(package="n01_scan_dynacbf_bridge", executable="scan_bspline_bridge",
             name="scan_bspline_bridge", parameters=[{
                 "frame_id":"map", "bspline_topic":"/planning/bspline",
                 "odom_topic":"/body_pose", "path_topic":"/nav/test/global_path",
                 "path_points":120, "arc_step":0.05, "table_samples":400,
                 "odom_timeout_s":.75, "trajectory_timeout_s":3.0,
                 "use_sim_time":sim
             }], output="screen"),
        Node(package="n01_offline_integration", executable="n01_dynamic_tracker",
             name="n01_offline_dynamic_tracker",
             parameters=[{"use_sim_time":sim}], output="screen"),
        Node(package="n01_offline_integration", executable="n01_path_risk",
             name="n01_offline_path_risk",
             parameters=[{"use_sim_time":sim}], output="screen"),
        Node(package="n01_offline_integration", executable="n01_dynamic_port",
             name="n01_dynamic_avoidance_interface",
             parameters=[{"use_sim_time":sim}], output="screen"),
        Node(package="n01_offline_integration", executable="n01_offline_observer",
             name="n01_offline_integration_observer",
             condition=IfCondition(fixture),
             parameters=[{
                 "duration_s":ParameterValue(LaunchConfiguration("duration_s"),value_type=float),
                 "report_path":EnvironmentVariable("N01_OFFLINE_REPORT_PATH",default_value=""),
                 "use_sim_time":sim
             }], output="screen"),
    ])
