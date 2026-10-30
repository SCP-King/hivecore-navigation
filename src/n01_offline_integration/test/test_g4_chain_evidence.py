"""Pure-Python G4.7 fail-closed order and provenance regression tests."""
import ast
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from n01_offline_integration.g4_chain_evidence import ChainEvidence
from n01_offline_integration.scan_reference_artifact import prepare_reference


def research_reference():
    cells = [[0., 0.], [.25, 0.], [.5, 0.]]
    return prepare_reference({
        "schema": "n01.offline.pcd_bag_evidence_audit.v1",
        "status": "CONFLICT_AWARE_RAY_VISIBILITY_ONLY_NOT_TRAVERSABILITY",
        "safety_authorization": "NEVER",
        "actuator_output": "DISABLED",
        "global_reference_algorithm":
            "A_STAR_ON_RAY_VISIBILITY_WITH_PCD_CONFLICT_VETO_NOT_SCAN",
        "research_path_length_m": .5,
        "research_path_xy_m": cells,
        "research_start_xy_m": cells[0],
        "research_goal_xy_m": cells[-1],
        "resolution_m": .25,
    }, ground_z=-1.)


def publish_state():
    return {"schema": "n01.offline.g4_reference_status.v1",
            "status": "UNVERIFIED_REFERENCE_SENT_TO_OFFLINE_SCAN",
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER"}


class G4ChainEvidenceTest(unittest.TestCase):
    def test_observer_rejects_old_scan_success_and_requires_every_hop(self):
        ref = research_reference()
        c = ChainEvidence(ref)
        c.native_bspline(True)  # Old mode1 path must not count
        c.g3_path()
        self.assertIn("G4_REFERENCE_NOT_OBSERVED_EXACTLY_ONCE", c.finalize()["issues"])
        c.reference_status(publish_state())
        c.initial_path("map", ref["path_xyz_m"])
        c.native_bspline(True)
        c.bridge_path()
        c.dynamic_report()
        c.g3_report()
        c.g3_path()
        out = c.finalize()
        self.assertEqual(out["result"], "G4_TO_G3_TOPIC_FLOW_OBSERVED_NOT_SAFETY_CERTIFIED")
        self.assertEqual(out["counts"]["native_valid_bspline_before_reference"], 1)
        self.assertEqual(out["counts"]["native_valid_bspline_after_reference"], 1)
        self.assertFalse(out["terrain_and_collision_verified"])
        self.assertFalse(out["g3_output_proven_to_follow_the_new_g4_route"])
        self.assertEqual(out["actuator_output"], "DISABLED")

    def test_mismatched_path_cannot_pass(self):
        ref = research_reference()
        c = ChainEvidence(ref)
        bad = [p.copy() for p in ref["path_xyz_m"]]
        bad[1][0] += .1
        c.reference_status(publish_state())
        c.initial_path("map", bad)
        c.native_bspline(True)
        self.assertEqual(c.counts["matching_initial_paths"], 0)
        self.assertEqual(c.counts["native_valid_bspline_after_reference"], 0)
        self.assertTrue(c.finalize()["issues"])

    def test_fail_closed_on_duplicate_reference_or_cmd_topic(self):
        ref = research_reference()
        c = ChainEvidence(ref)
        c.initial_path("map", ref["path_xyz_m"])
        c.initial_path("map", ref["path_xyz_m"])
        c.reference_status(publish_state())
        c.native_bspline(True)
        c.bridge_path()
        c.dynamic_report()
        c.g3_report()
        c.g3_path()
        c.cmd_vel_seen = True
        out = c.finalize()
        self.assertIn("CMD_VEL_PUBLISHER_OR_UNSAFE_REFERENCE_STATUS", out["issues"])
        self.assertIn("G4_REFERENCE_NOT_OBSERVED_EXACTLY_ONCE", out["issues"])

    def test_bad_provenance_rejected(self):
        bad = research_reference()
        bad["safety_authorization"] = "ALLOW"
        with self.assertRaises(ValueError):
            ChainEvidence(bad)

    def test_ros_observer_and_launcher_static_contract(self):
        node = (ROOT / "n01_offline_integration" / "g4_chain_observer_node.py")
        source = node.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('get_publishers_info_by_topic("/cmd_vel")', source)
        self.assertNotIn("create_publisher(", source)
        setup = (ROOT / "setup.py").read_text(encoding="utf-8")
        self.assertIn("n01_g4_chain_observer", setup)
        launch = (ROOT / "launch" / "g4_offline_scan_reference.launch.py").read_text(encoding="utf-8")
        self.assertIn("n01_g4_chain_observer", launch)
        self.assertIn("report_path", launch)


if __name__ == "__main__":
    unittest.main()
