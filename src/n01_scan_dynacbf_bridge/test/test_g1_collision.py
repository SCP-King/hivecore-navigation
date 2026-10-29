"""Regression tests: reproduce the two-cylinder occupancy semantics without ROS."""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from n01_scan_dynacbf_bridge.g1_collision_geometry import (
    cloud_voxel_index, path_collision, path_margin_proximity,
)


class G1CollisionTest(unittest.TestCase):
    def test_index_is_floor_not_round(self):
        cells = cloud_voxel_index([(1.025, -.025, .425)])
        self.assertEqual(cells, {(20, -1, 8)})

    def test_straight_trajectory_hits_front_cylinder(self):
        voxels = cloud_voxel_index([(1.025, .025, .425)])
        path = [(0.80, 0.0, .425), (0.85, 0.0, .425), (0.90, 0.0, .425)]
        self.assertTrue(path_collision(path, voxels))
        self.assertEqual(path_collision(path, voxels)[-1]["centres"][0]["side"], "front")

    def test_off_axis_detour_clears_obstacle(self):
        voxels = cloud_voxel_index([(1.025, .025, .425)])
        path = [(0.80, 0.50, .425), (0.85, 0.50, .425), (0.90, 0.50, .425)]
        self.assertEqual(path_collision(path, voxels), [])

    def test_extra_9cm_margin_is_diagnostic_only(self):
        voxels = cloud_voxel_index([(1.025, .025, .425)])
        path = [(0.95, .08, .425), (1.00, .08, .425), (1.05, .08, .425)]
        self.assertEqual(path_collision(path, voxels), [])
        self.assertGreater(len(path_margin_proximity(path, voxels)), 0)


if __name__ == "__main__":
    unittest.main()
