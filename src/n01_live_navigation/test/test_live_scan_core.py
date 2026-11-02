import math
import unittest
from types import SimpleNamespace

from types import SimpleNamespace

from n01_live_navigation.core import (
    goal_is_valid,
    estimate_velocity, localization_is_tracking, rotate_translate,
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
        self.assertEqual(localization_is_tracking(msg)[0], False)

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

    def test_velocity_requires_valid_timestamp_delta(self):
        v, ok = estimate_velocity(((0, 0, 0), 1.0), ((.1, 0, 0), 1.1))
        self.assertTrue(ok)
        self.assertAlmostEqual(v[0], 1.0, places=5)
        self.assertFalse(estimate_velocity(None, ((0, 0, 0), 1.0))[1])
        self.assertFalse(estimate_velocity(((0, 0, 0), 1.0),
                                           ((1, 0, 0), 1.01))[1])


if __name__ == "__main__":
    unittest.main()
