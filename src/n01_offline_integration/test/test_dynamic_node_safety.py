"""Source-level command isolation for the offline moving-object prototype."""
import ast
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]/"n01_offline_integration"


class DynamicNodeSafety(unittest.TestCase):
    def test_tracker_has_only_diagnostic_publisher(self):
        file=ROOT/"dynamic_tracker_node.py"
        code=file.read_text(encoding="utf-8")
        tree=ast.parse(code)
        topics=[]
        for call in ast.walk(tree):
            if (isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute)
                    and call.func.attr=="create_publisher" and len(call.args)>=2):
                self.assertIsInstance(call.args[1],ast.Constant)
                topics.append(call.args[1].value)
        self.assertEqual(topics,["/n01/offline/dynamic_tracks"])
        self.assertIn('"ROS_DOMAIN_ID"',code)
        self.assertIn('"ROS_LOCALHOST_ONLY"',code)
        self.assertNotIn("TransformBroadcaster",code)
        self.assertNotIn("create_publisher(Twist",code)

    def test_port_is_non_actuating(self):
        code=(ROOT/"dynamic_port_node.py").read_text(encoding="utf-8")
        self.assertIn('"mpc_cbf_solver":"NOT_IMPLEMENTED"',code)
        self.assertIn('"actuator_output":"DISABLED"',code)
        self.assertIn('"kalman_prediction":"NOT_IMPLEMENTED"',code)
        self.assertIn('self.on_tracks,10',code)

    def test_existing_bag_reader_no_raw_replay(self):
        code=(ROOT/"historical_bag_node.py").read_text(encoding="utf-8")
        self.assertNotIn("create_publisher(Twist",code)
        self.assertNotIn("TransformBroadcaster",code.split('class RecordedBagReplay')[1])
        self.assertIn('"raw_tf_republished":False',code)


if __name__=="__main__":
    unittest.main()
