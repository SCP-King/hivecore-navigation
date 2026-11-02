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
)


class LiveScanCoreTests(unittest.TestCase):
    def test_status_requires_real_tracking_and_published_tf(self):
        msg = SimpleNamespace(
            config_context_map_loaded=True,
            control_signal_outer_role="localization",
            control_signal_session_mode="tracking",
            registration_output_pose_published=True,
            registration_output_tf_published=True,
            registration_output_fitness_evaluation_ok=True,
            registration_output_tracking_compose_ok=True,
            fusion_path_imu_sync_ok=False,
        )
        self.assertEqual(localization_is_tracking(msg), (True, "tracking_ready"))
        msg.control_signal_session_mode = "relocalization"
        self.assertFalse(localization_is_tracking(msg)[0])

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
        self.assertFalse(result["controller_model_verified"])
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

    def test_velocity_requires_valid_timestamp_delta(self):
        v, ok = estimate_velocity(((0, 0, 0), 1.0), ((.1, 0, 0), 1.1))
        self.assertTrue(ok)
        self.assertAlmostEqual(v[0], 1.0, places=5)
        self.assertFalse(estimate_velocity(None, ((0, 0, 0), 1.0))[1])
        self.assertFalse(estimate_velocity(((0, 0, 0), 1.0),
                                           ((1, 0, 0), 1.01))[1])


if __name__ == "__main__":
    unittest.main()
