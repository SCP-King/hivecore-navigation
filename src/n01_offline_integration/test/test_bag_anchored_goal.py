"""G4.8 start-anchored offline research path selection, no fake map poses."""
import ast
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from n01_offline_integration.bag_anchored_goal import choose_research_goal
from n01_offline_integration.pcd_bag_evidence_audit import FREE, UNKNOWN


def example_grid():
    # Two components: only the component with the actual Bag pose is valid.
    width, height = 8, 6
    values = [UNKNOWN] * (width * height)
    for x in range(1, 6):
        values[1 * width + x] = FREE
    for x in range(1, 7):
        values[4 * width + x] = FREE
    return {"width": width, "height": height, "origin": [0., 0.],
            "resolution": .25, "grid": values}


class BagAnchoredGoalTests(unittest.TestCase):
    def test_select_from_actual_bag_cell_not_largest_component(self):
        # x=.38,y=.38 is cell (1,1): smaller component than row 4.
        result = choose_research_goal(example_grid(), [.38, .38])
        self.assertEqual(result["first_bag_pose_xy_m"], [.38, .38])
        self.assertEqual(result["start_cell"], 9)
        self.assertEqual(result["goal_cell"] // 8, 1)
        self.assertGreaterEqual(result["research_path_length_m"], .5)
        self.assertIsNone(result["floor_z_map_m"])
        self.assertFalse(result["floor_verified"])
        self.assertEqual(result["actuator_output"], "DISABLED")

    def test_unknown_start_stops_without_snapping_to_nearest_free(self):
        with self.assertRaisesRegex(ValueError, "BAG_START_NOT_OBSERVED_FREE"):
            choose_research_goal(example_grid(), [.88, .88])

    def test_outside_map_stops_without_clamping(self):
        with self.assertRaisesRegex(ValueError, "BAG_START_OUTSIDE_MAP"):
            choose_research_goal(example_grid(), [-5., .4])

    def test_too_small_observed_component_is_a_blocker(self):
        m = example_grid()
        m["grid"] = [UNKNOWN] * 48
        m["grid"][9] = FREE
        with self.assertRaisesRegex(ValueError, "NO_USABLE_BAG_ANCHORED"):
            choose_research_goal(m, [.38, .38])

    def test_bag_source_extractor_does_not_guess_ground_or_issue_ros_commands(self):
        source = ROOT.parents[1] / "scripts" / "n01_g4_bag_start_preflight.py"
        code = source.read_text(encoding="utf-8")
        ast.parse(code)
        self.assertIn("HistoricalMapTf", code)
        self.assertIn("stamp_to_ns(msg.header.stamp)", code)
        self.assertIn("ground_z_estimate_selected", code)
        self.assertIn("ground_z_map_m", code)
        self.assertNotIn("create_publisher(", code)
        self.assertNotIn('create_client(', code)
        runner = (ROOT.parents[1] / "scripts" / "run_n01_g4_full_chain.sh").read_text()
        self.assertIn("N01_G4_FLOOR_Z_VERIFIED", runner)
        self.assertIn("n01_g4_bag_start_preflight.py", runner)
        self.assertIn("G4.8_XY_READY_Z_UNKNOWN", runner)
        self.assertIn('--start "${COORDS[0]}"', runner)
        self.assertNotIn("ARGS=(--research-route)", runner)


if __name__ == "__main__":
    unittest.main()
