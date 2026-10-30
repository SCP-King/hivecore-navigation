"""Offline-only regression for source-audited Hive2 motion YAML design boundaries.

No ROS, robot hardware, controller calls, or assumptions about actual control.
"""
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/n01_offline_integration"))

from n01_offline_integration.g3_mpc_cbf_core import solve_discrete_mpc_cbf

AUDIT = ROOT / "config/reference/hive2_robot_motion_audit_20261030.json"
WHEEL = ROOT / "config/reference/hive2_wheel_only_tf_contract_20261030.json"


class RobotMotionAuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = json.loads(AUDIT.read_text(encoding="utf-8"))
        cls.wheel = json.loads(WHEEL.read_text(encoding="utf-8"))

    def test_two_identical_primary_source_copies_provenance(self):
        p = self.data["provenance"]
        self.assertTrue(p["content_identical"])
        self.assertEqual(p["source_bytes_each"], 7000)
        self.assertEqual(p["source_lines_each"], 218)
        self.assertEqual(len(p["source_paths"]), 2)
        self.assertEqual(len(p["sha256_both"]), 64)
        self.assertFalse(p["effective_runtime_file_confirmed"])
        self.assertFalse(p["controller_implementation_inspected"])

    def test_four_drive_and_four_steering_joint_bindings(self):
        robot = self.data["robot"]
        self.assertEqual(set(robot["driving_joints"]),
                         {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"})
        self.assertEqual(set(robot["steering_joints"]),
                         {f"{leg}_ankle_yaw" for leg in ("fl", "fr", "rl", "rr")})
        self.assertEqual(len(robot["pitch_hold_joints"]), 8)
        self.assertTrue(robot["legs_lift_controller_initially_inactive"])
        self.assertTrue(robot["legs_balance_enabled_in_inactive_lift_controller"])
        self.assertFalse(robot["pitch_hold_feedback_verified"])
        self.assertEqual(robot["controller_command_timeout_s"], .2)

    def test_source_config_values_and_g3_known_accel_mismatch(self):
        k = self.data["geometry_from_yaml"]
        self.assertEqual(k["wheel_radius_m"], .095)
        self.assertEqual(k["wheel_base_x_m"], .56)
        self.assertEqual(k["wheel_base_y_m"], .27)
        self.assertEqual(k["delta_y_m"], .085)
        self.assertLess(k["steering_joint_urdf_limits_from_previous_xacro_rad"][1],
                        k["chassis_steer_soft_limit_rad"])
        v = self.data["configured_motion_limits"]
        self.assertEqual(v["chassis_max_linear_speed_mps"], 2.0)
        self.assertEqual(v["chassis_max_linear_acceleration_mps2"], 1.0)
        self.assertEqual(v["chassis_nominal_linear_acceleration_mps2"], .6)
        self.assertEqual(v["approved_navigation_max_angular_velocity_radps"], None)
        import inspect
        # Check actual G3 default instead of silently replacing it.
        g3_limit = inspect.signature(solve_discrete_mpc_cbf).parameters[
            "max_accel"].default
        self.assertGreater(g3_limit, v["chassis_max_linear_acceleration_mps2"])
        issues = {row["id"]: row for row in self.data["integration_issues"]}
        self.assertEqual(issues["G3_MAX_ACCEL_MISMATCH"]["g3_default_mps2"],
                         g3_limit)
        self.assertEqual(issues["G3_MAX_ACCEL_MISMATCH"]["chassis_config_max_mps2"],
                         v["chassis_max_linear_acceleration_mps2"])

    def test_no_odom_tf_or_real_robot_clearance_claim(self):
        c = self.data["controller_and_odom"]
        self.assertFalse(c["odom_publish_tf"])
        self.assertTrue(c["no_inference_of_odom_topic_absence"])
        self.assertFalse(c["odom_tf_enabled_in_actual_run_verified"])
        self.assertEqual(c["controller_manager_update_rate_hz"], 1000)
        self.assertIsNone(self.data["unresolved_values"]["maximum_navigation_yaw_rate_radps"])
        self.assertIsNone(self.data["unresolved_values"]["verified_stop_distance_by_speed_m"])
        self.assertEqual(self.data["safety"]["safety_authorization"], "NEVER")
        self.assertEqual(self.data["safety"]["actuator_output"], "DISABLED")
        self.assertFalse(self.wheel["current_development"]["controller_kinematics_verified"])


if __name__ == "__main__":
    unittest.main()
