"""Regression of G3.3 motion screening against audited Hive2 source settings.

No robot commands, TF publishers, SLAM runtime or ROS dependencies.
"""
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/n01_offline_integration"))
from n01_offline_integration.g3_hive2_wheel_screen import (
    SOURCE_SHA256, CHASSIS_MAX_LINEAR_SPEED_MPS,
    CHASSIS_MAX_LINEAR_ACCEL_MPS2, SCAN_MAX_SPEED_MPS,
    SCAN_MAX_ACCEL_MPS2, screen_xy_candidate, screen_g3_result)
from n01_offline_integration.g3_pipeline_core import G3HistoricalCoordinator
from n01_offline_integration.g3_frame_evidence import append_frame, summarize


class G33WheelOnlyScreenTests(unittest.TestCase):
    def test_limits_tied_to_exact_audited_hive2_yaml(self):
        data = json.loads((ROOT / "config/reference/hive2_robot_motion_audit_20261030.json").read_text())
        self.assertEqual(SOURCE_SHA256, data["provenance"]["sha256_both"])
        cfg = data["configured_motion_limits"]
        self.assertEqual(CHASSIS_MAX_LINEAR_SPEED_MPS, cfg["chassis_max_linear_speed_mps"])
        self.assertEqual(CHASSIS_MAX_LINEAR_ACCEL_MPS2,
                         cfg["chassis_max_linear_acceleration_mps2"])
        planner = (ROOT / "src/scan_planner/plan_manage/config/planner.yaml").read_text()
        self.assertIn("    manager.max_vel: 0.75", planner)
        self.assertIn("    manager.max_acc: 0.5", planner)
        self.assertEqual(SCAN_MAX_SPEED_MPS, .75)
        self.assertEqual(SCAN_MAX_ACCEL_MPS2, .5)

    def test_slow_nominal_candidate_never_becomes_wheel_certified(self):
        # 0.2 m/s, assumed start from rest gives 0.4 m/s^2.
        row = screen_xy_candidate([[0.,0.],[.1,0.],[.2,0.]], .5)
        self.assertEqual(row["status"],
                         "TRANSLATIONAL_BOUNDS_WITHIN_CONFIG_4WS_UNVERIFIED")
        self.assertFalse(row["scan_planning_limits_exceeded"])
        self.assertFalse(row["chassis_config_limits_exceeded"])
        self.assertFalse(row["wheel_steering_feasibility_verified"])
        self.assertFalse(row["robot_model_execution_feasible"])
        self.assertFalse(row["actual_initial_velocity_verified"])
        self.assertEqual(row["physical_safety_authorization"], "NEVER")

    def test_g3_default_acceleration_can_exceed_actual_chassis_config(self):
        # First interval: 0 -> 0.75 m/s from the G3 assumed 0 velocity:
        # a=3.0 m/s^2 > 1.0 configured maximum.
        row = screen_xy_candidate([[0.,0.],[.1875,0.],[.375,0.]], .25)
        self.assertEqual(row["status"], "CONFIGURED_CHASSIS_TRANSLATION_EXCEEDED")
        self.assertTrue(row["chassis_config_limits_exceeded"])
        self.assertTrue(row["scan_planning_limits_exceeded"])
        self.assertFalse(row["first_acceleration_measured"])

    def test_scan_limited_even_when_chassis_limit_respected(self):
        # First velocity .4 m/s at dt=.5 yields a=.8: chassis <1,
        # but exceeds SCAN's 0.5 m/s^2.
        row = screen_xy_candidate([[0.,0.],[.2,0.],[.4,0.]], .5)
        self.assertFalse(row["chassis_config_limits_exceeded"])
        self.assertTrue(row["scan_planning_limits_exceeded"])
        self.assertEqual(row["status"], "SCAN_TRANSLATIONAL_PLANNING_BOUNDS_EXCEEDED")

    def test_nonfinite_truncated_and_missing_dt_fail_closed(self):
        self.assertEqual(screen_xy_candidate([[0,0],[float("nan"),0]], .25)["status"],
                         "INVALID_XY_CANDIDATE")
        self.assertEqual(screen_g3_result({"candidate_xy_m": [[0,0],[.1,0]]})["status"],
                         "INVALID_DISCRETE_TIME")
        self.assertEqual(screen_g3_result({})["status"], "G3_HAS_NO_CANDIDATE")
        self.assertEqual(screen_xy_candidate([[0,0]], .25)["status"],
                         "NO_BOUNDED_XY_CANDIDATE")

    def test_g3_pipeline_post_screen_and_journal_have_hard_no_command(self):
        pipeline = G3HistoricalCoordinator()
        pipeline.set_path([(0.,0.),(2.,0.)], 12345)
        stamp = 1765000000000000000
        self.assertEqual(pipeline.put_pose(stamp,(0.,0.,.9)), [])
        result = pipeline.put_tracks({
            "schema":"n01.offline.tracks.v1","status":"PROCESSED",
            "frame_id":"map","stamp_ns":stamp,"tracks":[]})[0]
        self.assertTrue(result["solver_executed"])
        self.assertIn("hive2_motion_limit_screen", result)
        screen = result["hive2_motion_limit_screen"]
        self.assertGreater(screen["samples"], 0)
        self.assertEqual(screen["physical_safety_authorization"], "NEVER")
        self.assertEqual(result["safety_authorization"], "NEVER")
        summary = pipeline.evidence()
        self.assertEqual(summary["hive2_xy_screened_candidate_frames"], 1)
        self.assertFalse(summary["hive2_4ws_execution_verified"])
        with tempfile.TemporaryDirectory() as d:
            log = Path(d)/"journal.jsonl"
            with log.open("w", encoding="utf-8") as handle:
                row = append_frame(handle,result)
            self.assertIn("hive2_motion_limit_screen", row)
            self.assertFalse(row["hive2_motion_limit_screen"]["wheel_steering_feasibility_verified"])
            audited = summarize(log)
            self.assertEqual(audited["counts"]["hive2_screened_attempts"],1)
            self.assertEqual(audited["counts"]["attempts"],1)


if __name__ == "__main__":
    unittest.main()
