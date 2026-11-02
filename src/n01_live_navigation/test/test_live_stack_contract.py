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
            'cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>',
            source,
        )

    def test_live_navigation_remains_non_actuating(self):
        package = ROOT / "src/n01_live_navigation/n01_live_navigation"
        combined = "\n".join(path.read_text() for path in package.glob("*.py"))
        self.assertNotIn('create_publisher(Twist, "/cmd_vel"', combined)
        self.assertNotIn('create_publisher(Twist, "/cmd_wheel"', combined)
        self.assertIn('"actuator_output": "DISABLED"', combined)
        self.assertIn('"safety_authorization": "NEVER"', combined)

    def test_native_scan_is_configured_for_static_map_overlay(self):
        launch = (ROOT / "src/n01_live_navigation/launch/live_scan.launch.py").read_text()
        self.assertIn('"grid_map.static_cloud_enabled": True', launch)
        self.assertIn('"/grid_map/static_occupancy"', launch)
        self.assertIn('n01_live_dynamic_obstacle_tracker', launch)
        self.assertIn('n01_hive2_trajectory_monitor', launch)


if __name__ == "__main__":
    unittest.main()
