"""Live vendor SLAM -> native SCAN integration; planning outputs only.

This launch does not start a SLAM backend, a controller, or any cmd_vel publisher.
The operator must explicitly assert that the supplied PCD belongs to the active
vendor map coordinate frame before enabling the static PCD layer.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    pcd = LaunchConfiguration("pcd_path").perform(context)
    asserted = LaunchConfiguration("pcd_map_frame_asserted").perform(context).lower()
    if asserted != "true":
        raise RuntimeError(
            "Refusing live SCAN launch: set pcd_map_frame_asserted:=true only after "
            "confirming this PCD is registered in the active vendor map frame")
    if not pcd.startswith("/"):
        raise RuntimeError("pcd_path must be an absolute path to the original PCD")
    ground_z = float(LaunchConfiguration("ground_z_map_m").perform(context))
    body_height = float(LaunchConfiguration("body_height_m").perform(context))
    if not (-1.5 <= ground_z <= 0.5 and 0.5 <= body_height <= 1.5):
        raise RuntimeError("ground/body height outside bounded research interval")

    planner_config = os.path.join(
        get_package_share_directory("scan_planner"), "config", "planner.yaml")
    return [
        Node(
            package="n01_static_map", executable="static_map_node",
            name="n01_live_static_pcd",
            parameters=[{
                "pcd_path": pcd, "resolution": 0.05, "publish_rate": 1.0,
                "frame_id": "map", "min_z": -1.75, "max_z": 3.35, "seed": 0,
            }],
            output="screen",
        ),
        Node(
            package="n01_live_navigation", executable="n01_live_scan_bridge",
            name="n01_live_scan_bridge",
            parameters=[{
                "status_topic": "/localization_status",
                "status_type": "hivecore_slam3d/msg/LocalizationStatus",
                "cloud_topic": "/livox/lidar/geo_filtered",
                "expected_cloud_frame": "lidar_link",
                "map_frame": "map", "body_frame": "base_link",
                "max_status_age_s": 1.0, "max_cloud_age_s": 0.6,
                "max_tf_wait_s": 0.45, "tf_retry_period_s": 0.02,
                "max_pending_clouds": 8, "max_points": 100000,
                "max_point_range_m": 12.0, "use_sim_time": False,
            }],
            output="screen",
        ),
        Node(
            package="n01_live_navigation", executable="n01_live_goal_gate",
            name="n01_live_scan_goal_gate", output="screen",
        ),
        Node(
            package="scan_planner", executable="scan_planner_node",
            name="scan_planner_node",
            parameters=[planner_config, {
                "use_sim_time": False,
                "fsm.navi_mode": 1,
                "fsm.thresh_replan": 1.0,
                "fsm.thresh_no_replan": 0.1,
                "fsm.planning_horizon": 7.5,
                "fsm.emergency_time": 1.0,
                "fsm.fail_safe": True,
                "fsm.max_replan_fail_count": 1000,
                "grid_map.resolution": 0.05,
                "grid_map.sliding_map_size_x": 10.0,
                "grid_map.sliding_map_size_y": 10.0,
                "grid_map.sliding_map_size_z": 5.0,
                "grid_map.local_update_range_x": 5.0,
                "grid_map.local_update_range_y": 5.0,
                "grid_map.local_update_range_z": 2.5,
                "grid_map.map_sliding_en": True,
                "grid_map.map_sliding_thresh": 0.2,
                # Research geometry only: candidate floor level from previous
                # PCD/Bag evidence, not a measured floor or certified body model.
                "grid_map.ground_height": ground_z,
                "grid_map.body_height": body_height,
                "grid_map.double_cylinder_radius": 0.33,
                "grid_map.double_cylinder_offset": 0.18,
                "grid_map.obstacles_inflation_z_up": 0.25,
                "grid_map.obstacles_inflation_z_down": 0.55,
                "grid_map.frame_id": "map",
                "grid_map.sliding_map_frame_id": "sliding_map",
                "grid_map.sensor_type": "lidar",
                "grid_map.cloud_is_world": True,
                "grid_map.need_extrinsic": False,
                "grid_map.static_cloud_enabled": True,
                "grid_map.static_cloud_topic": "/grid_map/static_occupancy",
                "grid_map.static_cloud_max_points": 1000000,
                "grid_map.p_hit": 0.85,
                "grid_map.p_miss": 0.30,
                "grid_map.p_min": 0.12,
                "grid_map.p_max": 0.98,
                "grid_map.p_occ": 0.80,
                "grid_map.max_ray_length": 5.0,
                "grid_map.vis_height": 0.3,
                "manager.max_vel": 0.25,
                "manager.max_acc": 0.25,
                "manager.max_jerk": 1.0,
                "manager.control_points_distance": 0.2,
                "manager.feasibility_tolerance": 0.5,
                "manager.planning_horizon": 7.5,
                "optimization.lambda_smooth": 1.0,
                "optimization.lambda_collision": 1.0,
                "optimization.lambda_feasibility": 0.1,
                "optimization.lambda_fitness": 1.0,
                "optimization.dist0": 0.33,
                "optimization.max_vel": 0.25,
                "optimization.max_acc": 0.25,
                "optimization.order": 3,
            }],
            remappings=[("move_base_simple/goal", "/n01/live_navigation/goal_gated")],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("pcd_path"),
        DeclareLaunchArgument("pcd_map_frame_asserted", default_value="false"),
        DeclareLaunchArgument("ground_z_map_m", default_value="-0.55"),
        DeclareLaunchArgument("body_height_m", default_value="0.90"),
        OpaqueFunction(function=_nodes),
    ])
