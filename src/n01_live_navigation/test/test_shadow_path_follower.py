import math
import unittest

from n01_live_navigation.shadow_path_follower import (
    cbf_speed_limit,
    lookahead_target,
)


class ShadowPathFollowerTests(unittest.TestCase):
    def test_lookahead_walks_polyline(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        target, dist = lookahead_target(pts, (0.0, 0.0), 0.8)
        self.assertIsNotNone(target)
        self.assertAlmostEqual(target[0], 0.8, places=3)
        self.assertAlmostEqual(dist, 0.8, places=3)

    def test_cbf_stops_inside_safe_radius(self):
        speed, margin = cbf_speed_limit(0.50, 0.55, 0.20, 0.15)
        self.assertEqual(speed, 0.0)
        self.assertLessEqual(margin, 0.0)

    def test_cbf_limits_near_obstacle(self):
        speed, margin = cbf_speed_limit(0.60, 0.55, 0.20, 0.15)
        self.assertAlmostEqual(margin, 0.05, places=3)
        self.assertLess(speed, 0.15)
        self.assertGreater(speed, 0.0)

    def test_cbf_keeps_desired_when_clear(self):
        speed, margin = cbf_speed_limit(2.0, 0.55, 0.20, 0.15)
        self.assertEqual(speed, 0.15)
        self.assertGreater(margin, 0.0)

    def test_shadow_source_has_no_cmd_vel_publisher(self):
        import inspect
        import n01_live_navigation.shadow_path_follower as mod
        src = inspect.getsource(mod)
        self.assertNotIn('"/cmd_vel"', src)
        self.assertNotIn("Twist", src)
        self.assertIn('"actuator_output": "DISABLED"', src)
        self.assertIn('"execution_authorized": False', src)
        self.assertIn('"safety_authorization": "NEVER"', src)


if __name__ == "__main__":
    unittest.main()
