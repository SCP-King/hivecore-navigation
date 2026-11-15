"""Unified mapping/localization data path: desk -> adapter -> SCAN bridge.

One click in the GUI starts the vendor localization desk (sensors, IMU filter,
static TF, cloud preprocess, MOLA localization) plus the N-01 adapter and the
live SCAN bridge. The bridge output (/cloud, /body_pose) is exactly the
navigation input; the navigation launch then only starts static map + planner.
"""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from n01_live_navigation.sensor_warmup import wait_for_mapping_inputs


def _localization_node(context):
    # Evaluate delayed substitutions in the persistent top-level context.
    mapping = LaunchConfiguration('slam_mode').perform(context) == 'mapping'
    if mapping:
        wait_for_mapping_inputs()
    params = LaunchConfiguration('mapping_params_file' if mapping else 'localization_params_file').perform(context)
    map_path = LaunchConfiguration('map_path').perform(context).strip()
    overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true',
        'auto_activate_localization': LaunchConfiguration('auto_activate_localization').perform(context) == 'true',
    }
    # Mapping and localization use the same MOLA synchronous scan host.
    # Bound IMU waiting/queued scans in both paths, not only localization.
    from n01_live_navigation.mola_pipeline import prepare_pipeline
    with open(params) as stream:
        settings = yaml.safe_load(stream)
    configured = settings.get('/**', settings.get('localization_lite_node', {})).get('ros__parameters', {})
    if configured.get('tracking_backend', 'mola') == 'mola':
        base = configured.get('mola_pipeline_yaml')
        if base:
            if not os.path.isabs(base):
                base = os.path.join(get_package_share_directory('hivecore_slam3d_lite_mapping'), base)
            overrides['mola_pipeline_yaml'] = prepare_pipeline(
                base,
                LaunchConfiguration('mola_pipeline_overrides_file').perform(context),
                '/home/demo/lyy/n01_runtime/config')
    if map_path:
        overrides['map_path'] = map_path
    if mapping:
        overrides.pop('auto_activate_localization')
        overrides['auto_activate_mapping'] = LaunchConfiguration('auto_activate_mapping').perform(context) == 'true'
        overrides['input_scan_topic'] = '/n01/slam/mapping_scan'
    package = 'hivecore_slam3d_lite_mapping' if mapping else 'hivecore_slam3d_lite_localization'
    node_name = 'mapping_lite_node' if mapping else 'localization_lite_node'
    return [Node(
        package=package, executable=node_name,
        name=node_name, parameters=[params, overrides], output='screen',
        additional_env=({'MOLA_VOXEL_STRIDE_ICP': os.environ.get('MOLA_VOXEL_STRIDE_ICP', '2')}
                        if mapping else {}),
        # Remap only this publisher/listener; never leak the rule to the adapter.
        remappings=[('/tf', '/n01/vendor_mapping/tf_raw' if mapping else '/n01/vendor_localization/tf_raw')],
    )]


def _adapter_node(context):
    mapping = LaunchConfiguration('slam_mode').perform(context) == 'mapping'
    return [Node(package='n01_live_navigation', executable='n01_localization_adapter',
                 name='n01_localization_adapter', output='screen',
                 parameters=[os.path.join(get_package_share_directory('n01_live_navigation'),
                     'config', 'slam3d_lite_mapping_adapter.yaml' if mapping else 'slam3d_lite_adapter.yaml')])]


def generate_launch_description():
    console = get_package_share_directory('hivecore_slam3d_lite_console')
    preproc = get_package_share_directory('hivecore_cloud_preprocessor')
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('imu_complementary_filter'),
            'launch', 'complementary_filter.launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('imu_filter_params_file'),
            'imu_data_raw_topic': LaunchConfiguration('imu_data_raw_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )
    preprocessing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(preproc, 'launch', 'cloud_preprocessor.launch.py')),
        launch_arguments={
            'slam_transport_declared_externally': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_fastdds_shm': LaunchConfiguration('use_fastdds_shm'),
            'static_tf_params_file': LaunchConfiguration('static_tf_params_file'),
            'lidar_cloud_preprocess_params_file': LaunchConfiguration('lidar_cloud_preprocess_params_file'),
            'pipeline_contract_file': '',
        }.items(),
    )

    adapter = OpaqueFunction(function=_adapter_node)

    bridge = Node(
        package="n01_live_navigation", executable="n01_live_scan_bridge",
        name="n01_live_scan_bridge",
        # Mapping and offline localization share the same per-scan navigation
        # path. The adapter explicitly distinguishes their status semantics.
        parameters=[{
            "status_topic": "/n01/localization/status",
            "cloud_topic": "/livox/lidar/geo_filtered",
            "pose_topic": "/n01/localization/pose",
            "odom_topic": "/odom", "odom_frame": "odom",
            "odom_child_frame": "base_link",
            "expected_cloud_frame": "lidar_link",
            "map_frame": "map", "body_frame": "base_link",
            "planner_body_frame": "base_link",
            "max_status_age_s": 1.0, "max_pose_rx_age_s": 0.7,
            "max_pose_stamp_age_s": 5.0,
            "max_pose_anchor_delta_s": 5.0,
            "max_live_body_pose_anchor_delta_s": 5.0,
            "pose_cache_size": 64,
            "max_odom_rx_age_s": 0.2,
            "max_odom_stamp_age_s": 0.4,
            "odom_interpolation_max_gap_s": 0.08,
            "odom_cache_size": 512,
            "max_cloud_age_s": 0.9,
            "max_latest_scan_lag_s": 0.7,
            "body_pose_publish_period_s": 0.05,
            "max_live_body_pose_age_s": 0.25,
            "max_tf_wait_s": 0.55, "tf_retry_period_s": 0.02,
            "max_pending_clouds": 1, "max_points": 100000,
            "max_point_range_m": 12.0, "use_sim_time": False,
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument('slam_mode', default_value='localization', choices=['localization','mapping']),
        DeclareLaunchArgument('auto_activate_mapping', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('mapping_params_file', default_value='/home/demo/lyy/hivecore-navigation/apps/n01_slam_gui/config/n01_mapping_lite.yaml'),
        DeclareLaunchArgument(
            "static_tf_params_file",
            default_value="/home/demo/lyy/hivecore-navigation/config/n01_static_tf_params.yaml"),
        DeclareLaunchArgument(
            "localization_params_file",
            default_value="/home/demo/lyy/hivecore-navigation/apps/n01_slam_gui/config/n01_localization_lite.yaml"),
        # Livox and robot_state_publisher belong to the independent desktop
        # chassis launcher. Missing chassis input must be reported, never
        # satisfied by silently starting another sensor/driver process.
        DeclareLaunchArgument('map_path', default_value=''),
        DeclareLaunchArgument('mola_pipeline_overrides_file', default_value=os.path.join(
            get_package_share_directory('n01_live_navigation'), 'config', 'mola_localization_overrides.yaml')),
        DeclareLaunchArgument('auto_activate_localization', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('use_fastdds_shm', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('launch_robot_state_publisher', default_value='false', choices=['false']),
        DeclareLaunchArgument('imu_data_raw_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('imu_filter_params_file', default_value=os.path.join(console,'config','livox_filter_config.yaml')),
        DeclareLaunchArgument('lidar_cloud_preprocess_params_file', default_value=os.path.join(preproc,'config','lidar_cloud_preprocess_params.yaml')),
        imu,
        preprocessing,
        Node(package='n01_live_navigation', executable='n01_mapping_scan_sampler',
             name='n01_mapping_scan_sampler', output='screen',
             condition=LaunchConfigurationEquals('slam_mode','mapping'),
             parameters=[{'max_rate_hz': 5.0, 'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        TimerAction(period=2.0, actions=[OpaqueFunction(function=_localization_node)]),
        adapter,
        bridge,
        Node(package='n01_live_navigation', executable='n01_wheel_joint_state',
             name='n01_wheel_joint_state', output='screen'),
        Node(package='n01_live_navigation', executable='n01_ground_reference',
             name='n01_ground_reference', output='screen'),
    ])
