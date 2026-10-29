"""Static safety regression for the X11-only historical N01 demo scripts."""
import ast
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[3]
PKG=ROOT/"src"/"n01_offline_integration"/"n01_offline_integration"


class RecordedDemoSafety(unittest.TestCase):
    def test_display_node_publishes_only_markers_and_demo_paths(self):
        src=(PKG/"demo_markers_node.py").read_text(encoding="utf-8")
        parsed=ast.parse(src)
        topics=[]
        for call in ast.walk(parsed):
            if (isinstance(call,ast.Call) and
                    isinstance(call.func,ast.Attribute) and
                    call.func.attr=="create_publisher"):
                self.assertIsInstance(call.args[1],ast.Constant)
                topics.append(call.args[1].value)
        self.assertEqual(set(topics),{
            "/n01/offline/demo_nominal_path",
            "/n01/offline/demo_candidate_path",
            "/n01/offline/demo_markers"})
        self.assertEqual(len(topics),3)
        self.assertNotIn("TransformBroadcaster",src)
        self.assertNotIn("create_publisher(Twist",src)

    def test_automatic_demo_requires_isolation_and_cleanup(self):
        src=(ROOT/"scripts"/"run_n01_demo_replay.sh").read_text(encoding="utf-8")
        for required in (
            "export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1",
            "N01_HISTORICAL_BAG_ONLY=1",
            "g1_process_guard.py",
            "nonempty_ros_domain",
            "owned_process_or_ros_graph_residual",
            "g3_solver_attempts",
            "safety_authorization",
            "record_n01_demo.sh",
            "export_n01_demo_gif.py",
        ):
            self.assertIn(required,src)
        self.assertNotIn("ros2 bag play",src)
        self.assertNotIn("reset --hard",src)
        self.assertNotIn("pkill",src)

    def test_recorder_never_falls_back_to_desktop(self):
        src=(ROOT/"scripts"/"record_n01_demo.sh").read_text(encoding="utf-8")
        self.assertIn("-window_id",src)
        self.assertIn("search --onlyvisible --pid",src)
        self.assertNotIn("-i :0",src)
        self.assertIn("Refusing whole-screen capture",src)

    def test_gif_must_come_from_actual_mp4(self):
        src=(ROOT/"scripts"/"export_n01_demo_gif.py").read_text(encoding="utf-8")
        self.assertIn("No real GIF frames produced",src)
        self.assertIn("Missing/nontrivial REAL recording MP4",src)
        self.assertIn("palettegen",src)
        self.assertIn("paletteuse",src)


if __name__=="__main__":
    unittest.main()
