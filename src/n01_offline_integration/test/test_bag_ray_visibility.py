"""G4.2 ray-visibility conservative grid synthetic tests (pure Python)."""
import pathlib
import sys
import unittest

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from n01_offline_integration.bag_ray_visibility import (
    RayVisibilityGrid, UNKNOWN, FREE, OCCUPIED)


class BagRayVisibilityTests(unittest.TestCase):
    def new_grid(self, **kw):
        return RayVisibilityGrid([(0.,5.),(0.,5.)],resolution=.25,**kw)

    def test_no_ray_is_all_unknown(self):
        g=self.new_grid()
        m=g.project()
        self.assertEqual(m["free_cells"],0)
        self.assertEqual(m["inflated_occupied_cells"],0)
        self.assertEqual(m["unknown_cells"],m["width"]*m["height"])

    def test_repeated_horizontal_ray_observes_free_but_endpoint_blocks(self):
        g=self.new_grid(min_free_rays=3)
        for i in range(3):
            self.assertTrue(g.ray((.25,.25,1.0),(2.25,.25,1.0)))
        m=g.project()
        self.assertGreater(m["free_cells"],0)
        self.assertEqual(m["grid"][g.cell((2.25,.25)) [1]*g.width
                                    +g.cell((2.25,.25))[0]],OCCUPIED)
        self.assertEqual(m["ray_count_stats"]["rays_accepted"],3)
        self.assertTrue(m["ray_visibility_evidence_only_not_ground_clearance"])
        self.assertEqual(m["safety_authorization"],"NEVER")

    def test_vertical_laser_does_not_make_ground_free(self):
        g=self.new_grid()
        self.assertFalse(g.ray((.25,.25,1.0),(2.25,.25,.2)))
        m=g.project()
        self.assertEqual(m["free_cells"],0)
        self.assertEqual(m["ray_count_stats"]["rejected_vertical"],1)

    def test_obstacle_beats_free_and_inflation(self):
        g=self.new_grid(min_free_rays=1)
        self.assertTrue(g.ray((.25,.25,1.),(2.25,.25,1.)))
        m=g.project(static_obstacle_xy=[(1.25,.25)],clearance_m=.25)
        x,y=g.cell((1.25,.25))
        self.assertEqual(m["grid"][y*g.width+x],OCCUPIED)
        self.assertGreater(m["inflated_occupied_cells"],2)

    def test_reject_outside_and_far_and_nonfinite(self):
        g=self.new_grid(max_range_m=2.)
        self.assertFalse(g.ray((-1.,1.,1.),(1.,1.,1.)))
        self.assertFalse(g.ray((.25,.25,1.),(4.5,.25,1.)))
        self.assertFalse(g.ray((float("nan"),1.,1.),(1.,1.,1.)))
        self.assertEqual(g.stats["rays_accepted"],0)

    def test_no_ros_or_commands(self):
        p=pathlib.Path(__file__).resolve().parents[1]
        c=(p/"n01_offline_integration"/"bag_ray_visibility.py").read_text()
        self.assertNotIn("rclpy",c)
        self.assertNotIn("create_publisher",c)
        self.assertNotIn("cmd_vel",c)


if __name__=="__main__":
    unittest.main()
