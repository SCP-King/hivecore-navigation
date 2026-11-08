"""Lean source-contract check for the optional FAST-LIO2 backend."""
import ast
import xml.etree.ElementTree as ET
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[3]


class FastLio2IntegrationContract(unittest.TestCase):
    def read(self, path):
        return (ROOT / path).read_text(encoding="utf-8")

    def test_upstream_is_pinned_and_kept_external(self):
        repos = self.read("n01_fast_lio2.repos")
        importer = self.read("scripts/import_n01_fast_lio2.sh")
        self.assertIn("hku-mars/FAST_LIO.git", repos)
        self.assertIn("cc3c9e8b1469b33b8072764d6db2348ed6dc597f", repos)
        self.assertIn("e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4", self.read(
            "docs/n01_fast_lio2_integration.md"))
        self.assertIn("patch_n01_fast_lio2.py", importer)
        self.assertIn("IKD_PIN=\"e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4\"", importer)
        self.assertTrue((ROOT / "src/n01_fast_lio/package.xml").is_file())

    def test_fast_lio_uses_raw_livox_and_private_outputs(self):
        config = self.read("src/n01_fast_lio/config/hive2_mid360.yaml")
        launch = self.read("src/n01_fast_lio/launch/fast_lio_mapping.launch.py")
        ast.parse(launch)
        self.assertIn("lid_topic: /livox/lidar", config)
        self.assertIn("imu_topic: /livox/imu", config)
        self.assertNotIn("lid_topic: /livox/lidar/geo_filtered", config)
        self.assertIn("time_sync_en: false", config)
        self.assertIn("map_frame_id: n01_fast_lio_map", config)
        self.assertIn("publish_tf: false", config)
        self.assertIn('("/Odometry", "/n01/fast_lio/odometry")', launch)
        self.assertIn('("/Laser_map", "/n01/fast_lio/map")', launch)
        self.assertIn('"common.publish_tf": False', launch)
        self.assertNotIn("/cmd_vel", launch)
        self.assertNotIn("chassis_controller", launch)
        runner = self.read("scripts/run_n01_fast_lio2_bag.sh")
        self.assertIn("ROS_DOMAIN_ID=88", runner)
        self.assertIn("ROS_LOCALHOST_ONLY=1", runner)
        self.assertIn('--topics "$LIDAR_TOPIC" "$IMU_TOPIC"', runner)
        self.assertIn("/n01/fast_lio/map_save", runner)
        self.assertNotIn("/cmd_vel", runner)

    def test_upstream_patch_is_pinned_and_tf_opt_in(self):
        patcher = self.read("scripts/patch_n01_fast_lio2.py")
        ast.parse(patcher)
        self.assertIn('PIN = "cc3c9e8b1469b33b8072764d6db2348ed6dc597f"', patcher)
        self.assertIn("n01_publish_tf = false", patcher)
        self.assertIn("if (n01_publish_tf) tf_br->sendTransform(trans);", patcher)
        self.assertIn("if (n01_publish_tf) {", patcher)
        self.assertIn("tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);", patcher)
        self.assertIn("common.map_frame_id", patcher)
        self.assertIn("common.imu_frame_id", patcher)

    def test_build_reuses_hive2_livox_overlay_without_stale_setup_chain(self):
        build = self.read("scripts/build_n01_fast_lio2.sh")
        self.assertIn("/opt/hivecore/hive2-motion/1.0.0/install/local_setup.bash", build)
        self.assertIn("HIVE2_LIVOX_LOCAL_SETUP", build)
        self.assertIn('source "$ROOT/scripts/lib/ros_env.sh"', build)
        self.assertIn('n01_source_setup "$HIVE2_LIVOX_LOCAL_SETUP"', build)
        self.assertIn("n01_source_setup", build)
        self.assertNotIn("source_overlay_compat", build)
        self.assertIn("ament_package_prefix livox_ros_driver2", build)
        self.assertIn("resource_index/packages/$package_name", build)
        self.assertIn("CMAKE_PREFIX_PATH", build)
        self.assertIn("AMENT_PREFIX_PATH", build)
        self.assertNotIn("apt install", build)
        self.assertNotIn("livox_ros_driver2.git", build)

    def test_package_manifest_is_valid_xml(self):
        ET.parse(ROOT / "src/n01_fast_lio/package.xml")

    def test_localization_contract_uses_normalized_lite_adapter_boundary(self):
        bridge = self.read("src/n01_offline_integration/config/offline_bridge.yaml")
        sensor = self.read("src/n01_bringup/config/n01_sensor.yaml")
        self.assertIn("status_topic: /n01/localization/status", bridge)
        self.assertNotIn("outer_value: localization", bridge)
        self.assertNotIn("tf_flag_path: registration_output_tf_published", bridge)
        self.assertIn("slam_rsp_disabled_confirmed: false", bridge)
        self.assertIn("input_topic: /livox/lidar/geo_filtered", sensor)
        self.assertIn("target_frame: map", sensor)


if __name__ == "__main__":
    unittest.main()
