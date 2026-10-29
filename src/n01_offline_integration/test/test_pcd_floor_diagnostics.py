"""G4.1 floor evidence and connectivity: research-only tests."""
import pathlib
import sys
import unittest

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from n01_offline_integration.pcd_floor_diagnostics import (
    floor_height_sweep, free_component_stats)
from n01_offline_integration.pcd_goal_grid import FREE, UNKNOWN


class PcdFloorDiagnosticsTests(unittest.TestCase):
    def test_disconnected_fragments_not_a_route(self):
        m=dict(grid=[FREE,UNKNOWN,FREE,FREE,UNKNOWN,UNKNOWN,FREE,FREE,FREE],
               width=3,height=3,resolution=.25,free_cells=6)
        s=free_component_stats(m)
        self.assertEqual(s["free_component_count_4_connected"],2)
        self.assertEqual(s["largest_free_component_cells"],5)
        self.assertEqual(s["top_free_component_sizes"],[5,1])

    def test_sweep_explicitly_does_not_select_floor(self):
        points=[(x*.25,y*.25,0.) for x in range(10) for y in range(10)]
        points.append((1.25,1.25,1.))
        s=floor_height_sweep(points,z_step=.25,clearance=.25)
        self.assertEqual(s["status"],"HEIGHT_HYPOTHESES_ONLY_NOT_FLOOR_VALIDATED")
        self.assertFalse("selected_floor_z_m" in s)
        self.assertEqual(s["actuator_output"],"DISABLED")
        row=[r for r in s["rows"] if r["floor_z_hypothesis_m"]==0.][0]
        self.assertGreater(row["largest_free_component_cells"],10)
        self.assertGreater(row["free_cells"],0)

    def test_empty_or_invalid_input_fails_closed(self):
        with self.assertRaises(ValueError):
            floor_height_sweep([])
        with self.assertRaises(ValueError):
            floor_height_sweep([(0,0,float("nan"))])
        with self.assertRaises(ValueError):
            floor_height_sweep([(0,0,0)],z_step=.001)

    def test_all_unknown_has_no_free_component(self):
        s=free_component_stats(dict(grid=[UNKNOWN]*4,width=2,height=2,
                                   resolution=.25,free_cells=0))
        self.assertEqual(s["largest_free_component_cells"],0)
        self.assertEqual(s["free_component_count_4_connected"],0)

    def test_no_publisher_or_motion_code(self):
        root=pathlib.Path(__file__).resolve().parents[1]
        code=(root/"n01_offline_integration"/"pcd_floor_diagnostics.py").read_text()
        self.assertNotIn("rclpy",code)
        self.assertNotIn("create_publisher",code)
        self.assertNotIn("cmd_vel",code)


if __name__=="__main__":
    unittest.main()
