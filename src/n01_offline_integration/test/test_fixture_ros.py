"""ROS 2 Humble type regression for offline fixture TF publishing.

This catches strict ROS generated Vector3 float setters before 75s integration.
No ROS node is started and no middleware data is published.
"""
import os
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))


class FixtureMessageTypesTest(unittest.TestCase):
    def test_mounted_tf_accepts_int_literals_as_input(self):
        from builtin_interfaces.msg import Time
        original = os.environ.get("N01_OFFLINE_FIXTURE")
        os.environ["N01_OFFLINE_FIXTURE"] = "1"
        try:
            from n01_offline_integration.fixture_node import new_tf
        finally:
            if original is None:
                os.environ.pop("N01_OFFLINE_FIXTURE", None)
            else:
                os.environ["N01_OFFLINE_FIXTURE"] = original

        stamp = Time(sec=123, nanosec=456)
        for frame, child, xyz in (
            ("base_footprint", "base_link", (0, 0, .9)),
            ("base_link", "lidar_link", (.295, 0, 0)),
            ("map", "base_footprint", (0, 0, 0)),
        ):
            msg = new_tf(stamp, frame, child, xyz)
            self.assertEqual(msg.header.frame_id, frame)
            self.assertEqual(msg.child_frame_id, child)
            self.assertEqual(msg.header.stamp, stamp)
            self.assertEqual(
                (msg.transform.translation.x, msg.transform.translation.y,
                 msg.transform.translation.z), tuple(float(value) for value in xyz))
            self.assertIsInstance(msg.transform.translation.x, float)
            self.assertIsInstance(msg.transform.translation.y, float)
            self.assertIsInstance(msg.transform.translation.z, float)
            self.assertEqual(msg.transform.rotation.w, 1.0)


if __name__ == "__main__":
    unittest.main()
