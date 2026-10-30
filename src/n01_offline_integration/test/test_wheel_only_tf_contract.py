"""Non-actuating reference-only regression for the 25-frame Hive2 wheel R&D TF scope."""
import json
from pathlib import Path
import unittest

REPO = Path(__file__).resolve().parents[3]
CONTRACT = REPO / "config/reference/hive2_wheel_only_tf_contract_20261030.json"


class WheelOnlyTfContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.doc = json.loads(CONTRACT.read_text(encoding="utf-8"))

    def test_25_unique_frames_and_primary_navigation_tree(self):
        frames = self.doc["frames"]
        names = [f["name"] for f in frames]
        self.assertEqual(len(names), 25)
        self.assertEqual(len(names), len(set(names)))
        core = {"map", "base_footprint", "base_link", "lidar_link",
                "imu_link", "livox_frame", "lpms_imu_link"}
        self.assertEqual(
            {f["name"] for f in frames if f["usage"] == "NAV_PRIMARY"}, core)
        indexed = {f["name"]: f for f in frames}
        self.assertIsNone(indexed["map"]["parent"])
        self.assertEqual(indexed["base_footprint"]["parent"], "map")
        self.assertEqual(indexed["base_link"]["parent"], "base_footprint")
        self.assertEqual(indexed["lidar_link"]["parent"], "base_link")
        self.assertEqual(indexed["imu_link"]["parent"], "lidar_link")
        self.assertEqual(indexed["livox_frame"]["parent"], "imu_link")
        self.assertEqual(indexed["lpms_imu_link"]["parent"], "base_link")
        self.assertIsNone(indexed["fixed_frame"]["parent"])
        self.assertEqual(indexed["imu_filtered"]["parent"], "fixed_frame")
        self.assertNotIn("odom", names)

    def test_four_fixed_posture_legs_still_part_of_collision_envelope(self):
        frames = self.doc["frames"]
        legs = [f for f in frames if f["name"].startswith(
            ("fl_", "fr_", "rl_", "rr_"))]
        self.assertEqual(len(legs), 16)
        self.assertTrue(all(
            f["usage"] == "COLLISION_GEOMETRY_ONLY_IN_WHEEL_LOCKED_MODE"
            for f in legs))
        mode = self.doc["current_development"]
        self.assertEqual(mode["drive_mode"], "WHEEL_DRIVE_ONLY")
        self.assertTrue(mode["collision_geometry_includes_fixed_legs_wheels_and_shell"])
        self.assertFalse(mode["leg_joint_hold_feedback_verified"])
        self.assertIsNone(mode["leg_joint_angles_locked_pose_rad"])
        self.assertEqual(
            mode["wheel_steering_and_drive_kinematic_model"],
            "UNVERIFIED_DO_NOT_ASSUME_DIFFERENTIAL_OR_ACKERMANN")
        self.assertFalse(mode["physical_control_authorized"])

    def test_scan_timestamp_map_and_no_odom_guess(self):
        cloud = self.doc["cloud_contract"]
        self.assertEqual(cloud["filtered_frame"], "lidar_link")
        self.assertEqual(cloud["raw_frame"], "livox_frame")
        self.assertEqual(cloud["target_map_frame"], "map")
        self.assertTrue(cloud["no_odom_frame_in_recorded_bag_tf"])
        self.assertFalse(cloud["odom_fallback_permitted"])
        self.assertIn("timestamp", cloud["required_transform_at_scan_timestamp"])
        yaml = (REPO / "src/n01_bringup/config/n01_sensor.yaml").read_text()
        self.assertIn("    target_frame: map", yaml)
        self.assertNotIn("    target_frame: odom", yaml)
        launcher = (REPO / "src/n01_bringup/launch/sensor_only.launch.py").read_text()
        self.assertIn("default_value='lidar_link'", launcher)
        self.assertNotIn("must already provide /odom", launcher)

    def test_mount_conflict_unknown_ground_and_strict_research_scope(self):
        self.assertTrue(self.doc["artifact_type"].startswith("READ_ONLY"))
        self.assertEqual(self.doc["heights"]["nominal_sensor_height_above_base_footprint_z_m"], .9)
        self.assertIsNone(self.doc["heights"]["ground_z_map_m"])
        self.assertFalse(self.doc["heights"]["static_tf_conflict_resolved_live"])
        self.assertEqual(self.doc["current_development"]["actuator_output"], "DISABLED")
        self.assertEqual(self.doc["current_development"]["safety_authorization"], "NEVER")
        self.assertFalse(self.doc["evidence"]["independently_reextracted_from_original_bag_in_this_change"])


if __name__ == "__main__":
    unittest.main()
