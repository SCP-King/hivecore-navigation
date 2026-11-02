"""Focused contract checks for dual SLAM backend and FAST-LIO adapter."""
import ast
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[3]

class DualSlamBackendTest(unittest.TestCase):
    def test_vendor_is_default_and_fast_lio_is_opt_in(self):
        p=ROOT/"src/n01_fast_lio/launch/slam_backend.launch.py"
        src=p.read_text()
        ast.parse(src)
        self.assertIn('DeclareLaunchArgument("backend", default_value="vendor"',src)
        self.assertIn('"fast_lio"',src)
        self.assertIn('PythonLaunchDescriptionSource(fast_launch)',src)
        self.assertIn('executable="n01_fast_lio_odometry_adapter"',src)

    def test_adapter_preserves_private_map_frame_and_never_publishes_tf(self):
        src=(ROOT/"src/n01_fast_lio/n01_fast_lio/odometry_adapter.py").read_text()
        ast.parse(src)
        self.assertIn('"/n01/fast_lio/odometry"',src)
        self.assertIn('"/n01/slam/odometry"',src)
        self.assertIn('"n01_fast_lio_map"',src)
        self.assertIn('"UNVERIFIED_PRIVATE_FRAME"',src)
        self.assertIn('"NOT_IMPLEMENTED"',src)
        self.assertNotIn('TransformBroadcaster',src)
        self.assertNotIn('create_publisher(Twist',src)
        self.assertNotIn('/cmd_vel',src)

    def test_vendor_bridge_contract_untouched_and_default(self):
        bridge=(ROOT/"src/n01_offline_integration/config/offline_bridge.yaml").read_text()
        self.assertIn('status_topic: /localization_status',bridge)
        self.assertIn('outer_value: localization',bridge)
        self.assertIn('inner_value: tracking',bridge)
        self.assertIn('slam_rsp_disabled_confirmed: false',bridge)
        pkg=(ROOT/"src/n01_fast_lio/package.xml").read_text()
        self.assertIn('<exec_depend>diagnostic_msgs</exec_depend>',pkg)
        self.assertIn('<exec_depend>nav_msgs</exec_depend>',pkg)

if __name__=="__main__":
    unittest.main()
