"""G2.3 regression: no command publishers and explicit open-loop assumptions."""
import ast
from pathlib import Path
import unittest

PKG=Path(__file__).resolve().parents[1]/"n01_offline_integration"


class ReadOnlyTemporalRisk(unittest.TestCase):
    def test_only_diagnostic_publisher(self):
        code=(PKG/"path_risk_node.py").read_text(encoding="utf-8")
        mod=ast.parse(code)
        topics=[]
        for call in ast.walk(mod):
            if (isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute)
                    and call.func.attr=="create_publisher"):
                self.assertIsInstance(call.args[1],ast.Constant)
                topics.append(call.args[1].value)
        self.assertEqual(topics,["/n01/offline/path_risk"])
        self.assertIn('"ROS_DOMAIN_ID"',code)
        self.assertIn('"ROS_LOCALHOST_ONLY"',code)
        self.assertNotIn("TransformBroadcaster",code)
        self.assertNotIn("create_publisher(Twist",code)

    def test_exact_recorded_stamp_required(self):
        source=(PKG/"replay_path_sync.py").read_text(encoding="utf-8")
        self.assertIn("stamp not in self.poses",source)
        self.assertIn("stamp not in self.reports",source)
        self.assertIn('"path_header_is_historical_stamp":False',source)
        self.assertIn('"RETAINED_SCAN_PATH_GEOMETRY_UNVERIFIED"',source)
        self.assertNotIn("safety_authorization=True",source)

    def test_reference_not_promoted_to_control(self):
        code=(PKG/"path_risk_node.py").read_text(encoding="utf-8")
        self.assertIn('"LAST_RECORDED_FRAME_SNAPSHOT_NOT_NEW_EVALUATION"',code)
        self.assertIn('"NO_RECENT_EXACT_TIMESTAMP_PAIRED_FRAME"',code)
        self.assertIn("frame_event=False",code)
        core=(PKG/"path_risk_core.py").read_text(encoding="utf-8")
        self.assertIn('"safety_authorization":"NEVER"',core)
        self.assertIn('"cbf_solver_implemented":False',core)
        self.assertIn('"actuator_output":"DISABLED"',core)

    def test_observer_requires_retained_reference_witness(self):
        code=(PKG/"observer_node.py").read_text(encoding="utf-8")
        self.assertIn("no_usable_path_projection_live_or_posthoc",code)
        runner=PKG.parents[2]/"scripts"/"run_n01_bag_functional.sh"
        self.assertIn("Path received but not usable for ANY projected recorded pose",
                      runner.read_text(encoding="utf-8"))


if __name__=="__main__":
    unittest.main()
