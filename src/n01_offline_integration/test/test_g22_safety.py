"""No actuator topics or misleading safety authorization in G2.2."""
import ast
from pathlib import Path
import unittest

PACKAGE=Path(__file__).resolve().parents[1]/"n01_offline_integration"


class ReadonlyG22(unittest.TestCase):
    def test_path_risk_node_only_diagnostic(self):
        code=(PACKAGE/"path_risk_node.py").read_text(encoding="utf-8")
        program=ast.parse(code)
        publishers=[]
        for n in ast.walk(program):
            if (isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
                    and n.func.attr=="create_publisher" and len(n.args)>=2):
                self.assertIsInstance(n.args[1],ast.Constant)
                publishers.append(n.args[1].value)
        self.assertEqual(publishers,["/n01/offline/path_risk"])
        self.assertIn('"ROS_DOMAIN_ID"',code)
        self.assertIn('"ROS_LOCALHOST_ONLY"',code)
        self.assertNotIn("create_publisher(Twist",code)
        self.assertNotIn("TransformBroadcaster",code)

    def test_port_does_not_promote_preview_to_controller(self):
        code=(PACKAGE/"dynamic_port_node.py").read_text(encoding="utf-8")
        self.assertIn('"mpc_cbf_solver":"NOT_IMPLEMENTED"',code)
        self.assertIn('"actuator_output":"DISABLED"',code)
        self.assertIn('"path_risk_interface":"DIAGNOSTIC_ONLY_NO_ACTUATION"',code)

    def test_launches_are_diagnostic_only(self):
        for launch in ("bag_pipeline.launch.py","offline_pipeline.launch.py"):
            code=(PACKAGE.parents[0]/"launch"/launch).read_text(encoding="utf-8")
            self.assertIn('executable="n01_path_risk"',code)
            self.assertIn('executable="n01_dynamic_tracker"',code)
            self.assertNotIn("create_publisher(Twist",code)


if __name__=="__main__":
    unittest.main()
