import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


class LiveStackContractTests(unittest.TestCase):
    def test_scan_grid_map_pairs_cloud_and_sensor_pose_by_exact_stamp(self):
        header = (ROOT / "src/scan_planner/plan_env/include/plan_env/grid_map.h").read_text()
        source = (ROOT / "src/scan_planner/plan_env/src/grid_map.cpp").read_text()
        self.assertIn("ExactTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>", header)
        self.assertIn("cloudPoseCallback", header)
        self.assertIn("sync_cloud_pose_", source)
        self.assertIn("sensorPoseCallback(pose);", source)
        self.assertIn("cloudCallback(cloud);", source)
        self.assertNotIn(
            '\\n    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>',
            source,
        )

    def test_bspline_visualizer_uses_actual_headerless_message_contract(self):
        message = (ROOT / "src/scan_planner/scan_planner_msgs/msg/Bspline.msg").read_text()
        monitor = (ROOT / "src/n01_live_navigation/n01_live_navigation/trajectory_monitor.py").read_text()
        self.assertIn("int32 order", message)
        self.assertIn("builtin_interfaces/Time start_time", message)
        self.assertNotIn("msg.header", monitor)
        self.assertIn("msg.traj_id", monitor)

    def test_bridge_uses_robot_pose_interpolation_and_only_sensor_mount_tf(self):
        bridge = (ROOT / "src/n01_live_navigation/n01_live_navigation/live_scan_bridge.py").read_text()
        launch = (ROOT / "src/n01_live_navigation/launch/live_scan.launch.py").read_text()
        supervisor = (ROOT / "src/n01_live_navigation/n01_live_navigation/runtime_supervisor.py").read_text()
        self.assertIn('"pose_topic": "/robot_pose"', launch)
        self.assertIn('"odom_topic": "/odom"', launch)
        self.assertIn('"odom_child_frame": "base_link"', launch)
        self.assertIn('"body_frame": "base_footprint"', launch)
        self.assertIn("motion_compensated_pose(", bridge)
        self.assertIn("self.last_processed_scan_stamp_ns == self.last_cloud_input_stamp_ns", bridge)
        self.assertIn("def on_odometry(", bridge)
        self.assertIn("odom_receive_fresh", supervisor)
        self.assertIn("interpolate_pose(", bridge)
        self.assertIn("compose_transform(", bridge)
        self.assertIn("self.body_frame, self.cloud_frame, scan_time", bridge)
        self.assertNotIn("self.map_frame, self.cloud_frame, scan_time", bridge)
        self.assertNotIn("map_to_base_footprint_tf_not_observed", supervisor)
        self.assertIn("sensor_extrinsic_ready", supervisor)

    def test_bridge_is_latest_only_and_reports_scan_pose_timestamp_alignment(self):
        bridge = (ROOT / "src/n01_live_navigation/n01_live_navigation/live_scan_bridge.py").read_text()
        launch = (ROOT / "src/n01_live_navigation/launch/live_scan.launch.py").read_text()
        config = (ROOT / "src/n01_live_navigation/config/live_scan.yaml").read_text()
        self.assertIn('"queue_policy": "latest_only_exact_scan_stamp"', bridge)
        self.assertIn('"pose_scan_skew_s"', bridge)
        self.assertIn('"pose_scan_bracket_available"', bridge)
        self.assertIn('"pose_scan_anchor_available"', bridge)
        self.assertNotIn("pose_interpolation_max_gap_s", bridge)
        self.assertIn('"max_pose_stamp_age_s": 0.7', launch)
        self.assertIn('"max_pose_anchor_delta_s": 0.5', launch)
        self.assertIn('"max_pending_clouds": 1', launch)
        self.assertIn('max_pose_stamp_age_s: 0.7', config)
        self.assertIn('max_pose_anchor_delta_s: 0.5', config)
        self.assertIn('odom_interpolation_max_gap_s: 0.08', config)
        self.assertIn('odom_cache_size: 512', config)
        self.assertIn('max_pending_clouds: 1', config)
        self.assertIn('self.counts["drop_superseded"] += len(self.pending_clouds)', bridge)

    def test_live_navigation_remains_non_actuating(self):
        package = ROOT / "src/n01_live_navigation/n01_live_navigation"
        combined = "\n".join(path.read_text() for path in package.glob("*.py"))
        self.assertNotIn('create_publisher(Twist, "/cmd_vel"', combined)
        self.assertNotIn('create_publisher(Twist, "/cmd_wheel"', combined)
        self.assertIn('"actuator_output": "DISABLED"', combined)
        self.assertIn('"safety_authorization": "NEVER"', combined)

    def test_static_pcd_is_retained_once_for_late_joining_consumers(self):
        provider = (ROOT / "src/n01_static_map/n01_static_map/static_map_node.py").read_text()
        grid_map = (ROOT / "src/scan_planner/plan_env/src/grid_map.cpp").read_text()
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", provider)
        self.assertIn("rclcpp::QoS(1).reliable().transient_local()", grid_map)
        self.assertNotIn("create_timer(1.0 / self._publish_rate", provider)

    def test_global_map_cloud_is_not_clipped_by_sensor_origin_range(self):
        tracker = (ROOT / "src/n01_live_navigation/n01_live_navigation/dynamic_obstacle_tracker.py").read_text()
        self.assertIn("decode_pointcloud_xyz(msg, self.max_points, None)", tracker)

    def test_rviz_goal_tool_targets_fail_closed_goal_gate_input(self):
        rviz = (ROOT / "src/n01_live_navigation/rviz/n01_live_navigation.rviz").read_text()
        gate = (ROOT / "src/n01_live_navigation/n01_live_navigation/goal_gate.py").read_text()
        self.assertIn("Value: /move_base_simple/goal", rviz)
        self.assertIn('PoseStamped, "/move_base_simple/goal"', gate)

    def test_native_scan_is_configured_for_static_map_overlay(self):
        launch = (ROOT / "src/n01_live_navigation/launch/live_scan.launch.py").read_text()
        self.assertIn('"grid_map.static_cloud_enabled": True', launch)
        self.assertIn('"/grid_map/static_occupancy"', launch)
        self.assertIn('n01_live_dynamic_obstacle_tracker', launch)
        self.assertIn('n01_hive2_trajectory_monitor', launch)


if __name__ == "__main__":
    unittest.main()
