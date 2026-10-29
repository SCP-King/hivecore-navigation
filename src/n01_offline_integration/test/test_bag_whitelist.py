"""Source-only safety regression for historical Bag replay.

The file is reviewed structurally before ROS build: accidental raw topic
replay or controller introduction must make this test fail.
"""
import ast
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]/"n01_offline_integration"


class BagWhitelistSafety(unittest.TestCase):
    def test_only_scanner_sensor_and_goal_publishers(self):
        source=(ROOT/"historical_bag_node.py").read_text(encoding="utf-8")
        tree=ast.parse(source)
        pubs=[]
        for call in ast.walk(tree):
            if not isinstance(call,ast.Call) or not isinstance(call.func,ast.Attribute):
                continue
            if call.func.attr=="create_publisher" and len(call.args)>=2:
                topic=call.args[1]
                if isinstance(topic,ast.Constant):
                    pubs.append(topic.value)
        self.assertEqual(set(pubs), {
            "/n01/offline/bag_replay_status", "/cloud", "/body_pose",
            "/sensor_pose", "/move_base_simple/goal"
        })
        self.assertNotIn("ros2 bag play",source)
        self.assertNotIn("TransformBroadcaster(",source)
        self.assertNotIn("create_publisher(Twist",source)

    def test_requires_explicit_isolation(self):
        source=(ROOT/"historical_bag_node.py").read_text(encoding="utf-8")
        self.assertIn('"N01_HISTORICAL_BAG_ONLY"',source)
        self.assertIn('"ROS_DOMAIN_ID"',source)
        self.assertIn('"ROS_LOCALHOST_ONLY"',source)
        self.assertIn("self.stats[",source)
        self.assertIn('"strict_p0_localization_verified":False',source)

    def test_no_odom_to_map_synthetic_assumption(self):
        source=(ROOT/"historical_tf.py").read_text(encoding="utf-8")
        self.assertIn("map->base_footprint",source)
        self.assertIn("SELECTED_MOUNT_HEIGHT_M = 0.90",source)


if __name__=="__main__":
    unittest.main()
