"""G4.9 route PCD Z evidence: real measurements, NEVER floor inference."""
import ast
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from n01_offline_integration.pcd_route_height_profile import (
    profile_route_surfaces,
)


class RouteSurfaceHeightProfileTest(unittest.TestCase):
    def test_bimodal_surface_heights_are_not_misreported_as_floor(self):
        xyz = ([(.02, .02, -.52)] * 15 +
               [(.02, .02, 2.68)] * 21 +
               [(.54, 0., -.49)] * 11)
        report = profile_route_surfaces(xyz, [[0., 0.], [.5, 0.]])
        self.assertEqual(report["route_waypoints"], 2)
        self.assertGreater(report["rows"][0]["pcd_samples_within_xy_radius"], 0)
        heights = report["rows"][0]["dominant_height_bins"]
        self.assertTrue(any(x["z_interval_m"][0] > 2. for x in heights))
        self.assertTrue(any(x["z_interval_m"][0] < 0. for x in heights))
        self.assertIsNone(report["ground_z_selected_m"])
        self.assertFalse(report["flat_floor_verified"])
        self.assertFalse(report["robot_clearance_verified"])
        self.assertEqual(report["actuator_output"], "DISABLED")

    def test_no_measured_points_is_unknown_not_clearance(self):
        report = profile_route_surfaces([(10., 10., .2)],
                                        [[0., 0.], [1., 0.]])
        self.assertEqual(report["waypoints_without_nearby_pcd_samples"], 2)
        self.assertIsNone(report["rows"][0]["surface_z_quantiles_m"])
        self.assertEqual(report["rows"][0]["dominant_height_bins"], [])
        self.assertFalse(report["support_continuity_verified"])

    def test_invalid_xy_and_nonfinite_points(self):
        with self.assertRaises(ValueError):
            profile_route_surfaces([], [[0., 0.], [1., 1.]])
        with self.assertRaises(ValueError):
            profile_route_surfaces([(0., 0., 1.)], [[0., 0.], [float("nan"), 1.]])
        with self.assertRaises(ValueError):
            profile_route_surfaces([(float("inf"), 0., 1.)], [[0., 0.], [1., 1.]])

    def test_preflight_uses_source_pcd_not_a_guessed_height(self):
        src = (ROOT.parents[1] / "scripts" / "n01_g4_bag_start_preflight.py")
        code = src.read_text(encoding="utf-8")
        ast.parse(code)
        self.assertIn('profile_route_surfaces(', code)
        self.assertIn('"g49_surface_height_profile.json"', code)
        self.assertIn('"ground_z_map_m": None', code)
        self.assertNotIn('create_publisher(', code)


if __name__ == "__main__":
    unittest.main()
