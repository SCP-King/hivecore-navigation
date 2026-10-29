"""G4 PCD projection and non-SCAN route-preview safety regression."""
import pathlib
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from n01_offline_integration.pcd_goal_grid import (
    FREE, OBSTACLE, UNKNOWN, project, route, save_outputs, world_cell)


class PcdGoalPreviewTests(unittest.TestCase):
    def fixture(self):
        # Dense observed flat floor with an obstacle patch, plus passage.
        floor = [(x*.25, y*.25, 0.) for x in range(21) for y in range(21)]
        obstacles = [(2.5, y*.25, 1.) for y in range(7,14)]
        return floor + obstacles

    def test_project_and_route_avoids_inflated_obstacles(self):
        grid = project(self.fixture(), clearance=.25)
        r = route(grid, (.5, .5), (4., 4.))
        self.assertGreater(r["length_m"], 0)
        self.assertTrue(r["path_xy_m"])
        self.assertTrue(all(grid["grid"][y*grid["width"]+x] == FREE
                            for x,y in r["grid_cells"]))
        self.assertEqual(grid["resolution"], .25)
        self.assertGreater(grid["inflated_occupied_cells"], 0)

    def test_unknown_is_blocked_by_default(self):
        grid = project([(0.,0.,0.), (5.,5.,0.)], clearance=0.)
        with self.assertRaisesRegex(ValueError, "no route found"):
            route(grid, (.01,.01), (5.,5.))
        r = route(grid, (.01,.01), (5.,5.), unknown_policy="optimistic")
        self.assertGreater(r["length_m"], 0)

    def test_never_cut_occupied_diagonal_corner(self):
        pts = [(x, y, 0.) for x in range(3) for y in range(3)]
        pts += [(1.,0.,1.), (0.,1.,1.)]
        grid = project(pts,resolution=.5,clearance=0.)
        a = world_cell(grid,(0.,0.))
        b = world_cell(grid,(1.,1.))
        self.assertNotEqual(a,b)
        # The only straight escape would cut between two occupied cells.
        with self.assertRaisesRegex(ValueError, "no route found"):
            route(grid,(0.,0.),(1.,1.))

    def test_invalid_goal_and_inputs_fail_closed(self):
        grid = project(self.fixture())
        with self.assertRaisesRegex(ValueError,"outside"):
            route(grid,(-100,0),(4.,4.))
        with self.assertRaisesRegex(ValueError,"invalid"):
            project(self.fixture(), resolution=-1)
        with self.assertRaisesRegex(ValueError,"policy"):
            route(grid,(0,0),(1,1),unknown_policy="unsafe")

    def test_map_exports_and_status_not_claimed_scan(self):
        grid=project(self.fixture())
        with tempfile.TemporaryDirectory() as d:
            base=pathlib.Path(d)/"preview"
            info=save_outputs(grid,base)
            self.assertIsNone(info["path"])
            self.assertEqual(info["global_reference_algorithm"],
                             "A_STAR_ON_PROJECTED_GRID_NOT_SCAN")
            self.assertFalse(info["scan_local_optimizer_connected"])
            self.assertEqual(info["safety_authorization"],"NEVER")
            self.assertEqual(info["actuator_output"],"DISABLED")
            pgm=base.with_suffix(".pgm").read_bytes()
            self.assertTrue(pgm.startswith(b"P5\n"))
            self.assertTrue(base.with_suffix(".yaml").exists())
            self.assertTrue(base.with_suffix(".svg").exists())


if __name__ == "__main__":
    unittest.main()
