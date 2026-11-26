"""Normalized localization -> native SCAN integration; planning outputs only.

This launch does not start a SLAM backend, a controller, or any cmd_vel publisher.
Offline mode requires an asserted PCD in the active map frame. Realtime mode
builds occupancy from live rays, with an optional updating accumulated-map
reference. Accumulated maps are never raycast as if they were current scans.
"""
import os
import fcntl
import subprocess
from collections import Counter
from n01_live_navigation.calibration_store import load_map_ground
from n01_live_navigation.core import GROUND_CLEARANCE_M

from ament_index_python.packages import get_package_share_directory

# Keep the descriptor alive for the lifetime of this launch process. flock is
# released automatically if ros2 launch exits or is killed.
_LIVE_SCAN_LOCK_FD = None

_MANAGED_NODE_NAMES = (
    "/n01_robot_footprint",
    "/n01_static_map",
    "/n01_realtime_reference",
    "/n01_environment_display",
    "/n01_trajectory_display",
    "/n01_live_runtime_supervisor",
    "/n01_live_safety_supervisor",
    "/n01_live_dynamic_obstacle_tracker",
    "/n01_hive2_trajectory_monitor",
    "/n01_live_scan_goal_gate",
    "/n01_run_cmd_logger",
    "/scan_planner_node",
    "/n01_live_navigation_rviz",
)


def _guard_single_live_stack(start_adapter, start_rviz):
    """Refuse a second live stack before creating any ROS nodes."""
    global _LIVE_SCAN_LOCK_FD
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    lock_path = f"/tmp/n01_live_scan_launch_domain_{domain}.lock"
    _LIVE_SCAN_LOCK_FD = open(lock_path, "a", encoding="utf-8")
    try:
        fcntl.flock(_LIVE_SCAN_LOCK_FD.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError(
            f"Another N-01 live SCAN launch owns {lock_path}; refusing duplicate startup."
        ) from exc

    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            check=True, capture_output=True, text=True, timeout=6.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError(
            f"Cannot inspect ROS graph for single-instance protection: {exc}"
        ) from exc

    names = [line.strip() for line in result.stdout.splitlines() if line.strip().startswith("/")]
    counts = Counter(names)
    managed = list(_MANAGED_NODE_NAMES)
    if not start_rviz:
        managed.remove("/n01_live_navigation_rviz")
    if start_adapter:
        managed.append("/n01_localization_adapter")

    duplicates = {name: counts[name] for name in managed if counts[name] > 1}
    already_running = [name for name in managed if counts[name] == 1]
    if duplicates or already_running:
        details = []
        if duplicates:
            details.append("duplicate managed nodes: " + ", ".join(
                f"{name} x{count}" for name, count in sorted(duplicates.items())))
        if already_running:
            details.append("managed nodes already running: " + ", ".join(sorted(already_running)))
        raise RuntimeError(
            "Refusing to start a second N-01 live navigation stack; "
            + "; ".join(details)
            + ". Stop the existing N-01 live_scan.launch.py process first. "
              "Lite SLAM and the independently managed localization adapter are not stopped."
        )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _nodes(context):
    speed = float(LaunchConfiguration('navigation_speed_mps').perform(context))
    if not .01 <= speed <= 1.0:
        raise RuntimeError('navigation speed must be between 0.01 and 1.0 m/s')
    pcd = LaunchConfiguration("pcd_path").perform(context)
    map_mode = LaunchConfiguration("map_mode").perform(context).lower()
    reference_topic = LaunchConfiguration("realtime_reference_topic").perform(context).strip()
    live_reference = map_mode == 'realtime' and bool(reference_topic)
    asserted = LaunchConfiguration("pcd_map_frame_asserted").perform(context).lower()
    use_urdf = LaunchConfiguration("use_urdf_footprint").perform(context).lower() == "true"
    dc_radius = float(LaunchConfiguration("double_cylinder_radius").perform(context))
    dc_offset = float(LaunchConfiguration("double_cylinder_offset").perform(context))
    env_margin = float(LaunchConfiguration("environment_margin").perform(context))
    if not (0.1 <= dc_radius <= 0.5 and 0.0 <= dc_offset <= 0.4 and 0.0 <= env_margin <= 0.2):
        raise RuntimeError("invalid footprint model parameters")
    static_map_resolution = float(LaunchConfiguration("static_map_resolution").perform(context))
    if not (0.05 <= static_map_resolution <= 0.20):
        raise RuntimeError("static_map_resolution must be in [0.05, 0.20]")
    static_map_min_z = float(LaunchConfiguration("static_map_min_z").perform(context))
    static_map_max_height_m = float(LaunchConfiguration("static_map_max_height_m").perform(context))
    static_map_floor_clearance = float(
        LaunchConfiguration("static_map_floor_clearance").perform(context))
    if not (0.0 <= static_map_floor_clearance <= 0.20):
        raise RuntimeError("static_map_floor_clearance must be in [0, 0.20]")
    if not (0.2 <= static_map_max_height_m <= 3.0):
        raise RuntimeError("invalid static_map z window")
    local_map_size = float(LaunchConfiguration("local_map_size_m").perform(context))
    if not (10.0 <= local_map_size <= 60.0):
        raise RuntimeError("local_map_size_m must be in [10, 60]")
    requested_horizon = float(LaunchConfiguration("local_planning_horizon").perform(context))
    if requested_horizon < 0.0:
        raise RuntimeError("local_planning_horizon must be >= 0 (0 = auto)")
    # Local lookahead is tied to the local map edge, not a separate magic 10 m.
    local_horizon = (requested_horizon if requested_horizon > 0.0
                     else max(3.0, 0.5 * local_map_size - 1.0))
    if map_mode not in ("offline", "realtime"):
        raise RuntimeError("map_mode must be offline or realtime")
    if map_mode == "offline" and asserted != "true":
        raise RuntimeError(
            "Refusing live SCAN launch: set pcd_map_frame_asserted:=true only after "
            "confirming this PCD is registered in the active vendor map frame")
    if map_mode == "offline" and not pcd.startswith("/"):
        raise RuntimeError("pcd_path must be an absolute path to the original PCD")
    ground_value = LaunchConfiguration("ground_z_map_m").perform(context)
    profile = load_map_ground(pcd) if not ground_value and map_mode == 'offline' else None
    if not ground_value and profile is None:
        raise RuntimeError("No matching saved ground profile; synchronize ground or provide explicit plane parameters")
    ground_z = float(ground_value or profile['c'])
    ground_a = float(LaunchConfiguration("ground_slope_x").perform(context) or (profile['a'] if profile else 0.0))
    ground_b = float(LaunchConfiguration("ground_slope_y").perform(context) or (profile['b'] if profile else 0.0))
    if not (abs(ground_a) <= .035 and abs(ground_b) <= .035):
        raise RuntimeError("ground slope exceeds planar navigation limits")
    # Do not let the raw PCD floor enter the static occupancy at all. The
    # static source is loaded above the measured floor + clearance; local/
    # global filters must not be the only defense against floor noise.
    static_map_min_z_effective = max(
        static_map_min_z, ground_z + static_map_floor_clearance)
    static_map_max_z_effective = ground_z + static_map_max_height_m
    if not (static_map_min_z_effective < static_map_max_z_effective):
        raise RuntimeError("static_map z window collapses above ground; check ground/clearance")
    body_height = float(LaunchConfiguration("body_height_m").perform(context))
    body_to_ground = float(LaunchConfiguration("body_to_ground_m").perform(context))
    if not (-1.5 <= ground_z <= 0.5 and 0.5 <= body_height <= 1.5
            and 0.4 <= body_to_ground <= 1.2):
        raise RuntimeError("ground/body height outside bounded research interval")

    navigation_config = os.path.join(get_package_share_directory("n01_live_navigation"), "config", "navigation.yaml")
    planner_config = os.path.join(
        get_package_share_directory("scan_planner"), "config", "planner.yaml")
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    start_adapter = LaunchConfiguration("start_localization_adapter").perform(context).lower() == "true"
    start_rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    _guard_single_live_stack(start_adapter, start_rviz)
    return [
        Node(package='n01_live_navigation', executable='n01_environment_display',
             name='n01_environment_display', output='screen',
             prefix='nice -n 10',
             parameters=[{'ground_z_map_m': ground_z, 'ground_slope_x': ground_a,
                          'ground_slope_y': ground_b, 'resolution_m': .06, 'radius_m': 12.}]),
        *([Node(package='n01_live_navigation', executable='n01_realtime_reference',
                name='n01_realtime_reference', output='screen',
                parameters=[{'input_topic': reference_topic}])] if live_reference else []),
        Node(package="n01_live_navigation", executable="n01_robot_footprint",
             name="n01_robot_footprint", output="screen"),
        Node(package="n01_live_navigation", executable="n01_perception_consistency",
             name="n01_perception_consistency", output="screen",
             parameters=[{"radius_m": 1.2}]),
        Node(package="n01_live_navigation", executable="n01_trajectory_display",
             name="n01_trajectory_display", output="screen",
             prefix="nice -n 10",
             parameters=[{"ground_slope_x": ground_a, "ground_slope_y": ground_b,
                          "ground_z_map_m": ground_z}]),
        # The adapter may already be running as an independently supervised process.
        # Default false prevents duplicate publishers on /n01/localization/pose.
        Node(
            package="n01_live_navigation", executable="n01_localization_adapter",
            name="n01_localization_adapter",
            condition=IfCondition(LaunchConfiguration("start_localization_adapter")),
            parameters=[os.path.join(get_package_share_directory("n01_live_navigation"),
                "config", "slam3d_lite_adapter.yaml")],
            output="screen",
        ),
        Node(
            package="n01_static_map", executable="static_map_node",
            name="n01_static_map",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("map_mode"), "' == 'offline'"
            ])),
            parameters=[
                os.path.join(get_package_share_directory("n01_static_map"),
                    "config", "static_map.yaml"),
                {
                    "pcd_path": pcd,
                    # PGM is a GUI reference base, not the planner's static
                    # obstacle source. Keeping PCD here preserves the original
                    # SCAN local-planning corridor behavior.
                    "occupancy_grid_path": "",
                    "frame_id": "map",
                    "resolution": static_map_resolution,
                    "min_z": static_map_min_z_effective,
                    "max_z": static_map_max_z_effective,
                },
            ],
            output="screen",
        ),
        Node(
            package="n01_live_navigation", executable="n01_live_scan_bridge",
            name="n01_live_scan_bridge",
            condition=IfCondition(LaunchConfiguration("start_scan_bridge")),
            parameters=[navigation_config, {
                "status_topic": "/n01/localization/status",
                "cloud_topic": "/cloud_registered_body",
                "pose_topic": "/n01/localization/pose",
                "odom_topic": "/odom", "odom_frame": "odom",
                "odom_child_frame": "base_link",
                "expected_cloud_frame": "imu_link",
                "map_frame": "map", "body_frame": "base_link",
                "planner_body_frame": "base_link",
                "max_status_age_s": 3.0, "max_pose_rx_age_s": 3.0,
                "max_pose_stamp_age_s": 5.0,
                "max_pose_anchor_delta_s": 5.0,
                "max_live_body_pose_anchor_delta_s": 5.0,
                "pose_cache_size": 64,
                "max_odom_rx_age_s": 3.0,
                "max_odom_stamp_age_s": 3.0,
                "odom_interpolation_max_gap_s": 0.5,
                "odom_cache_size": 512,
                "max_cloud_age_s": 3.0,
                "max_latest_scan_lag_s": 3.0,
                "body_pose_publish_period_s": 0.05,
                "max_live_body_pose_age_s": 3.0,
                "max_tf_wait_s": 3.0, "tf_retry_period_s": 0.02,
                "ground_plane": [ground_a,ground_b,ground_z], "ground_configured": True,
                "max_pending_clouds": 3, "max_points": 100000,
                "max_point_range_m": 40.0, "use_sim_time": False,
            }],
            output="screen",
        ),
        Node(
            package="n01_live_navigation", executable="n01_live_runtime_supervisor",
            name="n01_live_runtime_supervisor",
            parameters=[{
                "max_body_pose_age_s": 3.0,
                "max_body_pose_rx_age_s": 3.0,
                "max_dynamic_diagnostic_age_s": 3.0,
            }],
            output="screen",
        ),
        Node(
            package="n01_live_navigation", executable="n01_live_safety_supervisor",
            name="n01_live_safety_supervisor", output="screen",
        ),
        Node(
            package="n01_live_navigation", executable="n01_live_dynamic_obstacle_tracker",
            name="n01_live_dynamic_obstacle_tracker",
            parameters=[{
                "cloud_topic": "/cloud",
                "static_topic": "/grid_map/static_occupancy",
                "map_frame": "map",
                "body_pose_topic": "/body_pose",
                "body_to_ground_m": body_to_ground,
                "ground_z_map_m": ground_z,
                "use_map_ground_plane": True,
                "ground_slope_x": ground_a,
                "ground_slope_y": ground_b,
                "reference_updates_enabled": live_reference,
                "body_height_m": body_height,
                "voxel_size_m": 0.30,
                "cluster_cell_m": 0.35,
                "min_cluster_points": 6,
                "max_points": 100000,
                "max_cloud_age_s": 3.0,
            }],
            output="screen",
        ),
        Node(
            package="n01_live_navigation", executable="n01_hive2_trajectory_monitor",
            name="n01_hive2_trajectory_monitor",
            additional_env={"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"},
            parameters=[navigation_config, {
                # 2026-11-17: blind spots are a mapping responsibility. Navigation
                # only blocks on explicit obstacles, so execution no longer
                # requires validated footprint coverage.
                "map_frame": "map",
                "map_id": LaunchConfiguration("map_id", default="").perform(context),
                # Screen the candidate at the speed the slow executor actually
                # commands (0.05 m/s), not at the planner's 0.25 m/s limit.
                # At 0.25 m/s the same path needs up to ~5.4 rad/s steering,
                # which trips the 1.57 rad/s screen and invalidates the path
                # before the robot can move.
                "max_speed_mps": speed,
                "max_accel_mps2": 0.25,
                "wheel_base_x_m": 0.56,
                "track_y_m": 0.27,
                "wheel_radius_m": 0.095,
                "delta_y_m": 0.085,
                "steering_limit_rad": 3.141592653589793,
                "environment_margin": env_margin,
                "max_bspline_receive_age_s": 3.0,
            }],
            output="screen",
        ),
        Node(
            package="n01_live_navigation", executable="n01_live_goal_gate",
            name="n01_live_scan_goal_gate", output="screen",
        ),
        Node(
            package="n01_live_navigation", executable="n01_navigation_diagnostics",
            name="n01_navigation_diagnostics", output="screen",
            prefix="nice -n 10",
            parameters=[{"run_log_root": "/home/demo/lyy/n01_runtime/run_logs",
                         "map_path": pcd}],
        ),
        Node(
            package="n01_live_navigation", executable="n01_run_cmd_logger",
            name="n01_run_cmd_logger", output="screen",
            prefix="nice -n 10",
            parameters=[{"log_dir": "/home/demo/lyy/n01_runtime/run_cmd"}],
        ),
        Node(
            package="rviz2", executable="rviz2", name="n01_live_navigation_rviz",
            condition=IfCondition(LaunchConfiguration("rviz")),
            prefix="nice -n 10",
            arguments=["-d", rviz_config],
            output="screen",
        ),
        Node(
            package="scan_planner", executable="scan_planner_node",
            name="scan_planner_node",
            parameters=[planner_config, navigation_config, {
                "use_sim_time": False,
                "fsm.navi_mode": 1,
                "fsm.thresh_replan": 0.1,
                "fsm.max_replan_interval_s": 0.5,
                "fsm.thresh_no_replan": 0.07,
                "fsm.planning_horizon": local_horizon,
                # Local target is a staged point on the global path, not the
                # final goal when the remaining global route is shorter than
                # the 14 m lookahead. This keeps local optimization local.
                "fsm.local_stage_horizon": 4.0,
                "fsm.final_handoff_radius": 0.50,
                "fsm.emergency_time": 1.0,
                "fsm.fail_safe": True,
                "grid_map.resolution": 0.10,
                # Confirmed ray evidence only changes the temporary overlay.
                "grid_map.static_observation_refresh": True,
                # Same rule for planning: no inferred unknown obstacles.
                # Let the global reference reach goals outside the loaded map;
                # unknown space is traversable, not a rejection reason.
                "grid_map.extend_reference_to_goal": True,
                "grid_map.static_prior_as_cost": False,
                # Global is read-only; local short-term evidence is fused in a
                # per-cycle snapshot and never written back to global.
                "grid_map.fused_2d_enabled": True,
                # The saved PCD is a complete static map: inside its XY extent,
                # no obstacle means free; outside remains unknown.
                "grid_map.pcd_bounds_free": True,
                # Planning may treat unknown as cost; execution stays gated.
                "grid_map.unknown_as_obstacle": False,
                "grid_map.sliding_map_size_x": local_map_size,
                "grid_map.sliding_map_size_y": local_map_size,
                "grid_map.sliding_map_size_z": 5.0,
                "grid_map.local_update_range_x": max(5.0, local_map_size / 2.0 - 1.0),
                "grid_map.local_update_range_y": max(5.0, local_map_size / 2.0 - 1.0),
                "grid_map.local_update_range_z": 3.0,
                "grid_map.map_sliding_en": True,
                "grid_map.map_sliding_thresh": 0.2,
                # Research geometry only: candidate floor level from previous
                # PCD/Bag evidence, not a measured floor or certified body model.
                "grid_map.ground_height": ground_z,
                "grid_map.ground_slope_x": ground_a,
                "grid_map.ground_slope_y": ground_b,
                "grid_map.body_height": body_height,
                "grid_map.use_urdf_footprint": use_urdf,
                "grid_map.double_cylinder_radius": dc_radius,
                "grid_map.double_cylinder_offset": dc_offset,
                "grid_map.environment_margin": env_margin,
                "grid_map.fixed_body_heading": True,
                # Asymmetric audited envelope; global/local/RViz share disks.
                "grid_map.footprint_centers_x": [-0.43, -0.10, 0.23],
                # Full vertical extent; 6cm floor band accounts for map floor scatter.
                "grid_map.ground_clearance": GROUND_CLEARANCE_M,
                "grid_map.obstacles_inflation_z_up": 0.90,
                # Body reference is now below the lidar by ~0.146m. Include
                # its reference plus a conservative 0.10m sensor allowance.
                "grid_map.obstacles_inflation_z_down": 0.25,
                "grid_map.frame_id": "map",
                "grid_map.sliding_map_frame_id": "sliding_map",
                "grid_map.sensor_type": "lidar",
                "grid_map.cloud_is_world": True,
                "grid_map.need_extrinsic": False,
                "grid_map.static_cloud_enabled": map_mode == "offline" or live_reference,
                "grid_map.static_ego_filter_enabled": True,
                "grid_map.static_ego_filter_radius": 0.60,
                "grid_map.static_ego_filter_margin": 0.06,
                "grid_map.static_ego_filter_z_down": 0.80,
                "grid_map.static_ego_filter_z_up": 0.90,
                "grid_map.live_ego_filter_enabled": True,
                "grid_map.live_ego_filter_margin": 0.06,
                "grid_map.raycast_dedup_resolution": 0.10,
                "grid_map.reference_updates_enabled": live_reference,
                "grid_map.static_cloud_topic": "/grid_map/static_occupancy",
                "grid_map.static_cloud_max_points": 1000000,
                "grid_map.p_hit": 0.85,
                "grid_map.p_miss": 0.30,
                "grid_map.p_min": 0.12,
                "grid_map.p_max": 0.98,
                "grid_map.p_occ": 0.80,
                "grid_map.max_ray_length": local_map_size / 2.0,
                # Display clipping only: retain walls/railings above the body.
                "grid_map.vis_height": 1.4,
                "manager.max_vel": speed,
                "manager.max_acc": 0.25,
                "manager.max_jerk": 1.0,
                "manager.control_points_distance": 0.2,
                "manager.feasibility_tolerance": 0.5,
                "manager.planning_horizon": local_horizon,
                "manager.global_waypoint_spacing": 0.30,
                # Keep the global guide trackable at the 4WS yaw-rate limit.
                "manager.min_turn_radius": 0.45,
                "manager.max_yaw_rate": 0.45,
                "optimization.lambda_smooth": 1.0,
                "optimization.lambda_collision": 1.0,
                "optimization.lambda_feasibility": 0.1,
                "optimization.lambda_fitness": 1.0,
                "optimization.dist0": 0.33,
                "optimization.max_vel": speed,
                "optimization.max_acc": 0.25,
                "optimization.order": 3,
            }],
            remappings=[("move_base_simple/goal", "/n01/live_navigation/goal_gated")],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('navigation_speed_mps', default_value='0.4'),
        DeclareLaunchArgument("map_mode", default_value="offline"),
        DeclareLaunchArgument("realtime_reference_topic", default_value="",
                             description="Optional accumulated SLAM map; live scan rays stay separate"),
        DeclareLaunchArgument("pcd_path", default_value=""),
        DeclareLaunchArgument("pcd_map_frame_asserted", default_value="false"),
        DeclareLaunchArgument("use_urdf_footprint", default_value="true"),
        DeclareLaunchArgument("double_cylinder_radius", default_value="0.25"),
        DeclareLaunchArgument("double_cylinder_offset", default_value="0.18"),
        DeclareLaunchArgument("environment_margin", default_value="0.02"),
        DeclareLaunchArgument("static_map_resolution", default_value="0.10"),
        DeclareLaunchArgument("static_map_min_z", default_value="-1.20"),
        DeclareLaunchArgument("static_map_max_height_m", default_value="1.60",
                              description="Unified obstacle ceiling above fitted ground"),
        DeclareLaunchArgument("static_map_floor_clearance", default_value="0.06",
                              description="Static PCD z floor = ground_z + clearance"),
        DeclareLaunchArgument("local_map_size_m", default_value="30.0",
                              description="Local sliding map size; planning horizon auto = half - 1 m"),
        DeclareLaunchArgument("local_planning_horizon", default_value="0.0",
                              description="0 = auto from local map half-size; >0 explicit metres"),
        DeclareLaunchArgument(
            "start_localization_adapter", default_value="false",
            description="Start adapter only when no independent adapter is already running"),
        # Map-referenced estimate from stationary pointcloud/pose audit, 2026-11-12.
        # Empty ground arguments restore the plane bound to the selected PCD.
        DeclareLaunchArgument("ground_z_map_m", default_value=""),
        DeclareLaunchArgument("ground_slope_x", default_value=""),
        DeclareLaunchArgument("ground_slope_y", default_value=""),
        DeclareLaunchArgument("body_height_m", default_value="0.90"),
        # Legacy diagnostic fallback only: production uses the map plane.
        DeclareLaunchArgument("body_to_ground_m", default_value="0.775"),
        DeclareLaunchArgument(
            "start_scan_bridge", default_value="true",
            description="Start the pose/cloud bridge; disable when the unified localization stack owns it"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("map_id", default_value=""),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                get_package_share_directory("n01_live_navigation"),
                "rviz", "n01_live_navigation.rviz")),
        OpaqueFunction(function=_nodes),
    ])
