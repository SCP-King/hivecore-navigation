import math
import struct
import unittest
from types import SimpleNamespace

import numpy as np

from n01_live_navigation.core import (
    goal_is_valid,
    estimate_velocity,
    localization_is_tracking,
    rotate_translate,
    decode_pointcloud_xyz,
    transform_points,
    cluster_xy_points,
    four_wheel_steer_screen,
    interpolate_pose,
    compose_transform,
    motion_compensated_pose,
    scan_output_is_fresh,
    evaluate_safety_gate,
    stamped_data_is_fresh,
    trajectory_window_is_active,
    select_pose_anchor,
    select_latest_tf_stamp,
    pose_samples_equivalent,
    sensor_cloud_stamp_gate,
    ground_z_from_body_pose,
)


class LiveScanCoreTests(unittest.TestCase):
    def test_status_requires_map_tracking_and_successful_relocalization(self):
        status = {
            "map_loaded": "true", "session": "tracking",
            "reloc_ok": "true", "reloc_fitness": "0.23328",
        }
        self.assertEqual(localization_is_tracking(status), (True, "tracking_ready"))
        status["session"] = "relocalization"
        self.assertEqual(localization_is_tracking(status), (False, "localization_not_tracking"))
        status["session"] = "tracking"
        status["reloc_ok"] = "false"
        self.assertEqual(localization_is_tracking(status), (False, "localization_relocalization_not_ok"))

    def test_status_rejects_invalid_fitness(self):
        status = {"map_loaded": "true", "session": "tracking",
                  "reloc_ok": "true", "reloc_fitness": "nan"}
        self.assertEqual(localization_is_tracking(status),
                         (False, "localization_reloc_fitness_invalid"))

    def test_status_contract_mismatch_fails_closed(self):
        self.assertEqual(
            localization_is_tracking(SimpleNamespace()),
            (False, "localization_status_contract_mismatch"),
        )

    def test_identity_transform(self):
        self.assertEqual(
            rotate_translate((1, 2, 3), (0, 0, 0), (0, 0, 0, 1)),
            (1.0, 2.0, 3.0),
        )

    def test_quarter_turn_and_translation(self):
        p = rotate_translate((1, 0, 0), (2, 3, 4),
                             (0, 0, math.sqrt(.5), math.sqrt(.5)))
        self.assertAlmostEqual(p[0], 2.0)
        self.assertAlmostEqual(p[1], 4.0)
        self.assertAlmostEqual(p[2], 4.0)

    def test_bad_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            rotate_translate((0, 0, 0), (0, 0, 0), (0, 0, 0, 0))

    def test_vectorized_transform_matches_quarter_turn(self):
        points = np.asarray([[1, 0, 0], [0, 1, 0]], dtype=np.float32)
        result = transform_points(
            points, (2, 3, 4), (0, 0, math.sqrt(.5), math.sqrt(.5)))
        np.testing.assert_allclose(result, [[2, 4, 4], [1, 3, 4]], atol=1e-6)

    def test_pointcloud_decoder_honors_offsets_and_padding(self):
        fields = [
            SimpleNamespace(name="x", offset=0, datatype=7, count=1),
            SimpleNamespace(name="y", offset=4, datatype=7, count=1),
            SimpleNamespace(name="z", offset=8, datatype=7, count=1),
        ]
        payload = struct.pack("<ffff", 1.0, 2.0, 3.0, 123.0)
        payload += struct.pack("<ffff", 4.0, 5.0, 6.0, 456.0)
        msg = SimpleNamespace(
            width=2, height=1, point_step=16, row_step=32,
            fields=fields, is_bigendian=False, data=payload,
        )
        decoded = decode_pointcloud_xyz(msg, max_points=2, max_range_m=10.0)
        np.testing.assert_allclose(decoded, [[1, 2, 3], [4, 5, 6]])

    def test_pointcloud_decoder_rejects_oversize(self):
        msg = SimpleNamespace(
            width=2, height=1, point_step=12, row_step=24,
            fields=[], is_bigendian=False, data=b"\0" * 24,
        )
        with self.assertRaises(ValueError):
            decode_pointcloud_xyz(msg, max_points=1)

    def test_cluster_xy_points_separates_components(self):
        points = np.asarray([
            [0.00, 0.00, 0.1], [0.02, 0.01, 0.2], [0.04, 0.02, 0.3],
            [2.00, 2.00, 0.1], [2.02, 2.01, 0.2], [2.04, 2.02, 0.3],
        ], dtype=np.float32)
        clusters = cluster_xy_points(points, cell_size=0.2, min_points=3)
        self.assertEqual(len(clusters), 2)

    def test_four_wheel_screen_straight_path(self):
        points = np.column_stack((
            np.linspace(0.0, 5.0, 30),
            np.zeros(30),
            np.zeros(30),
        ))
        result = four_wheel_steer_screen(points)
        self.assertTrue(result["necessary_conditions_pass"])
        self.assertAlmostEqual(result["max_abs_steering_angle_rad"], 0.0, places=5)
        self.assertTrue(result["controller_model_verified"])
        self.assertFalse(result["execution_authorized"])

    def test_goal_gate_requires_live_and_static_readiness(self):
        goal = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=0.0)))
        self.assertEqual(goal_is_valid(goal, live_ready=False, static_ready=True),
                         (False, "live_localization_or_cloud_not_ready"))
        self.assertEqual(goal_is_valid(goal, live_ready=True, static_ready=False),
                         (False, "native_scan_static_pcd_not_ready"))
        self.assertEqual(goal_is_valid(goal, live_ready=True, static_ready=True),
                         (True, "accepted"))
        goal.header.frame_id = "odom"
        self.assertFalse(goal_is_valid(goal, live_ready=True, static_ready=True)[0])

    def test_robot_pose_interpolates_position_and_orientation_at_scan_time(self):
        samples = [
            (1_000_000_000, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            (1_200_000_000, (2.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0)),
        ]
        pose = interpolate_pose(samples, 1_100_000_000, max_gap_s=0.5)
        self.assertIsNotNone(pose)
        np.testing.assert_allclose(pose[0], (1.0, 0.0, 0.0), atol=1e-8)
        np.testing.assert_allclose(
            pose[1], (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)), atol=1e-7)

    def test_robot_pose_interpolation_never_extrapolates_or_crosses_large_gap(self):
        samples = [
            (1_000_000_000, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            (3_000_000_000, (2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ]
        self.assertIsNone(interpolate_pose(samples, 500_000_000, max_gap_s=0.5))
        self.assertIsNone(interpolate_pose(samples, 3_500_000_000, max_gap_s=0.5))
        self.assertIsNone(interpolate_pose(samples, 2_000_000_000, max_gap_s=0.5))

    def test_live_tf_lag_uses_actual_transform_stamp_with_bounded_age(self):
        requested = 1_000_000_000
        # 37.76 ms behind odometry is normal bounded TF publication latency.
        self.assertEqual(
            select_latest_tf_stamp(requested, 962_240_000, 0.15),
            962_240_000,
        )
        # Static transforms (stamp zero) apply at the requested odometry time.
        self.assertEqual(select_latest_tf_stamp(requested, 0, 0.15), requested)
        # Do not silently use a future transform or an excessively stale one.
        self.assertIsNone(select_latest_tf_stamp(requested, 1_001_000_000, 0.15))
        self.assertIsNone(select_latest_tf_stamp(requested, 800_000_000, 0.15))
        self.assertIsNone(select_latest_tf_stamp(requested, 962_240_000, 0.02))

    def test_stale_global_anchor_is_usable_only_with_explicit_odom_budget(self):
        samples = [
            (1_000_000_000, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ]
        self.assertEqual(
            select_pose_anchor(samples, 2_100_000_000, 1.25)[0],
            1_000_000_000,
        )
        self.assertIsNone(select_pose_anchor(samples, 2_300_000_000, 1.25))

    def test_duplicate_pose_samples_treat_quaternion_sign_as_equivalent(self):
        existing = (1_000_000_000, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
        self.assertTrue(pose_samples_equivalent(
            existing, (1.01, 2.0, 3.0), (0.0, 0.0, 0.0, -1.0)))
        self.assertFalse(pose_samples_equivalent(
            existing, (1.1, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)))
        yaw = math.sin(0.1 / 2.0)
        self.assertFalse(pose_samples_equivalent(
            existing, (1.0, 2.0, 3.0), (0.0, 0.0, yaw, math.cos(0.1 / 2.0))))

    def test_live_body_pose_uses_separate_anchor_bound_from_scan_compensation(self):
        samples = [
            (1_000_000_000, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            (1_700_000_000, (0.7, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ]
        target = 1_650_000_000
        # Scan-time compensation keeps the strict 0.5 s bound and chooses
        # the nearby future anchor rather than extrapolating from an old pose.
        scan_anchor = select_pose_anchor(samples, target, 0.5)
        self.assertEqual(scan_anchor[0], 1_700_000_000)
        # Current body pose may propagate from the latest causal SLAM anchor
        # for up to the independently bounded 0.7 s using timestamped /odom.
        live_anchor = select_pose_anchor(samples, target, 0.7)
        self.assertEqual(live_anchor[0], 1_000_000_000)
        self.assertIsNone(select_pose_anchor(samples, target, 0.04))

    def test_body_pose_and_sensor_extrinsic_compose_into_map_lidar(self):
        translation, rotation = compose_transform(
            (1.0, 0.0, 0.0),
            (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
        np.testing.assert_allclose(translation, (1.0, 1.0, 0.0), atol=1e-7)
        np.testing.assert_allclose(
            rotation, (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)), atol=1e-7)


    def test_odometry_relative_motion_propagates_map_anchor(self):
        yaw90 = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
        pose = motion_compensated_pose(
            ((10.0, 0.0, 0.0), yaw90),
            ((1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            ((2.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        )
        np.testing.assert_allclose(pose[0], (10.0, 1.0, 0.0), atol=1e-7)
        np.testing.assert_allclose(pose[1], yaw90, atol=1e-7)

    def test_odometry_rotation_is_applied_about_body_anchor(self):
        yaw90 = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
        pose = motion_compensated_pose(
            ((3.0, 4.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            ((0.0, 0.0, 0.0), yaw90),
        )
        np.testing.assert_allclose(pose[0], (3.0, 4.0, 0.0), atol=1e-7)
        np.testing.assert_allclose(pose[1], yaw90, atol=1e-7)

    def test_scan_output_accepts_only_bounded_fresh_lag(self):
        fresh, lag = scan_output_is_fresh(
            1_000_000_000, 1_250_000_000, 0.30, 0.10, 0.60, 0.45)
        self.assertTrue(fresh)
        self.assertAlmostEqual(lag, 0.25)
        self.assertFalse(scan_output_is_fresh(
            1_000_000_000, 1_600_000_000, 0.30, 0.10, 0.60, 0.45)[0])
        self.assertFalse(scan_output_is_fresh(
            1_000_000_000, 1_100_000_000, 0.70, 0.10, 0.60, 0.45)[0])
        self.assertFalse(scan_output_is_fresh(
            None, 1_100_000_000, 0.10, 0.10, 0.60, 0.45)[0])

    def test_trajectory_window_rejects_future_and_expired_bspline(self):
        active, age = trajectory_window_is_active(
            1_000_000_000, 2_000_000_000, 3.0)
        self.assertTrue(active)
        self.assertAlmostEqual(age, 1.0)
        self.assertFalse(trajectory_window_is_active(
            2_500_000_000, 2_000_000_000, 3.0)[0])
        self.assertFalse(trajectory_window_is_active(
            1_000_000_000, 5_000_000_000, 3.0)[0])
        self.assertEqual(trajectory_window_is_active(0, 2_000_000_000, 3.0),
                         (False, None))
        self.assertEqual(trajectory_window_is_active(
            1_000_000_000, 2_000_000_000, 0.0), (False, None))

    def test_cloud_stamp_gate_accepts_bounded_delay_and_rejects_bad_order(self):
        now_ns = 10_000_000_000
        ok, reason, age = sensor_cloud_stamp_gate(
            9_300_000_000, now_ns, 0.9, previous_stamp_ns=9_000_000_000)
        self.assertTrue(ok)
        self.assertEqual(reason, "accepted")
        self.assertAlmostEqual(age, 0.7)
        self.assertEqual(
            sensor_cloud_stamp_gate(8_900_000_000, now_ns, 0.9)[1],
            "cloud_stamp_stale",
        )
        self.assertEqual(
            sensor_cloud_stamp_gate(10_300_000_000, now_ns, 0.9)[1],
            "cloud_stamp_in_future",
        )
        self.assertEqual(
            sensor_cloud_stamp_gate(9_200_000_000, now_ns, 0.9, 9_200_000_000)[1],
            "cloud_stamp_not_monotonic",
        )
        self.assertEqual(
            sensor_cloud_stamp_gate(0, now_ns, 0.9)[1],
            "cloud_stamp_invalid",
        )

    def test_sensor_stamp_freshness_rejects_stale_future_and_zero_stamps(self):
        now_ns = 2_000_000_000
        fresh, age = stamped_data_is_fresh(1_800_000_000, now_ns, 0.5)
        self.assertTrue(fresh)
        self.assertAlmostEqual(age, 0.2)
        self.assertFalse(stamped_data_is_fresh(1_000_000_000, now_ns, 0.5)[0])
        self.assertFalse(stamped_data_is_fresh(2_300_000_000, now_ns, 0.5)[0])
        self.assertEqual(stamped_data_is_fresh(0, now_ns, 0.5), (False, None))
        self.assertEqual(stamped_data_is_fresh(None, now_ns, 0.5), (False, None))

    def test_safety_gate_never_authorizes_motion(self):
        result = evaluate_safety_gate(
            health_fresh=True,
            navigation_ready=True,
            feasibility_fresh=True,
            feasibility_status="NECESSARY_CONDITIONS_PASS",
            candidate_review_ready=True,
            operator_hold=False,
            physical_checks={},
        )
        self.assertFalse(result["execution_authorized"])
        self.assertEqual(result["actuator_output"], "DISABLED")
        self.assertEqual(result["safety_authorization"], "NEVER")
        self.assertTrue(result["physical_acceptance_missing"])

    @staticmethod
    def _all_physical_checks_pass():
        return {
            name: True for name in (
                "emergency_stop_chain_verified",
                "communication_loss_stop_verified",
                "localization_loss_stop_verified",
                "dynamic_obstacle_stop_verified",
                "manual_takeover_verified",
                "wheel_controller_mapping_verified",
                "wheel_contact_geometry_verified",
                "collision_envelope_verified",
                "braking_distance_verified",
            )
        }

    def _evaluate_clear_dynamic_scene(self, dynamic_fresh=True, dynamic_status="NO_UNMATCHED_CLUSTERS"):
        return evaluate_safety_gate(
            health_fresh=True,
            navigation_ready=True,
            feasibility_fresh=True,
            feasibility_status="NECESSARY_CONDITIONS_PASS",
            candidate_review_ready=True,
            operator_hold=False,
            physical_checks=self._all_physical_checks_pass(),
            dynamic_obstacle_fresh=dynamic_fresh,
            dynamic_obstacle_status=dynamic_status,
        )

    def test_dynamic_obstacle_status_missing_or_stale_blocks_review(self):
        result = self._evaluate_clear_dynamic_scene(dynamic_fresh=False)
        self.assertEqual(result["status"], "DYNAMIC_OBSTACLE_STATUS_NOT_READY")
        self.assertIn("dynamic_obstacle_status_missing_or_stale", result["blockers"])
        self.assertFalse(result["execution_authorized"])

    def test_dynamic_obstacle_candidates_require_review(self):
        result = self._evaluate_clear_dynamic_scene(
            dynamic_fresh=True, dynamic_status="CANDIDATES_AVAILABLE")
        self.assertEqual(result["status"], "DYNAMIC_OBSTACLE_REVIEW_REQUIRED")
        self.assertTrue(result["dynamic_obstacle_review_required"])
        self.assertIn("dynamic_obstacle_candidates_require_review", result["blockers"])
        self.assertFalse(result["execution_authorized"])

    def test_fresh_clear_scene_still_never_authorizes_actuation(self):
        result = self._evaluate_clear_dynamic_scene()
        self.assertEqual(result["status"], "REVIEW_ONLY_NOT_AUTHORIZED")
        self.assertTrue(result["dynamic_obstacle_status_fresh"])
        self.assertFalse(result["execution_authorized"])
        self.assertEqual(result["actuator_output"], "DISABLED")
        self.assertEqual(result["safety_authorization"], "NEVER")

    def test_ground_z_from_body_pose_uses_leg_offset(self):
        self.assertAlmostEqual(ground_z_from_body_pose(0.917, 0.778), 0.139, places=6)
        self.assertAlmostEqual(ground_z_from_body_pose(0.0, 0.778), -0.778, places=6)
        self.assertIsNone(ground_z_from_body_pose(float("nan"), 0.778))
        self.assertIsNone(ground_z_from_body_pose(0.917, 0.0))
        self.assertIsNone(ground_z_from_body_pose(None, 0.778))

    def test_velocity_requires_valid_timestamp_delta(self):
        v, ok = estimate_velocity(((0, 0, 0), 1.0), ((.1, 0, 0), 1.1))
        self.assertTrue(ok)
        self.assertAlmostEqual(v[0], 1.0, places=5)
        self.assertFalse(estimate_velocity(None, ((0, 0, 0), 1.0))[1])
        self.assertFalse(estimate_velocity(((0, 0, 0), 1.0),
                                           ((1, 0, 0), 1.01))[1])


if __name__ == "__main__":
    unittest.main()
