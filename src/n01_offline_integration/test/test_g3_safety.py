"""G3 source safety regression: offline-only candidate path is not a command."""
import ast
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
PKG=ROOT/"n01_offline_integration"


class G3OfflineOnlySafety(unittest.TestCase):
    def test_only_explicit_diagnostic_and_candidate_path_pubs(self):
        path=PKG/"g3_solver_node.py"
        src=path.read_text(encoding="utf-8")
        ast_tree=ast.parse(src)
        topics=[]
        for x in ast.walk(ast_tree):
            if (isinstance(x,ast.Call) and
                isinstance(x.func,ast.Attribute) and
                x.func.attr=="create_publisher"):
                self.assertIsInstance(x.args[1],ast.Constant)
                topics.append(x.args[1].value)
        self.assertEqual(set(topics),{
            "/n01/offline/g3_solver_report",
            "/n01/offline/g3_candidate_path"})
        self.assertEqual(len(topics),2)
        self.assertIn('"N01_HISTORICAL_BAG_ONLY"',src)
        self.assertIn('"ROS_LOCALHOST_ONLY"',src)
        self.assertIn('"ROS_DOMAIN_ID"',src)
        self.assertNotIn("create_publisher(Twist",src)
        self.assertNotIn("TransformBroadcaster",src)
        self.assertNotIn("ActionClient",src)

    def test_g3_audit_uses_filesystem_path_not_ros_path(self):
        src=(PKG/"g3_solver_node.py").read_text(encoding="utf-8")
        self.assertIn("from pathlib import Path as FilePath", src)
        self.assertIn("FilePath(report_path).with_name", src)
        self.assertIn("from nav_msgs.msg import Path, Odometry", src)

    def test_optimizer_not_chassis_velocity_or_control_loop(self):
        src=(PKG/"g3_mpc_cbf_core.py").read_text(encoding="utf-8")
        self.assertIn('p[k+1] = p[k] + dt*v[k]',src)
        self.assertIn('"safety_authorization":"NEVER"',src)
        self.assertIn('"actuator_output":"DISABLED"',src)
        self.assertIn('"real_robot_ready":False',src)
        self.assertIn('"robot_model":"PLANAR_HOLONOMIC_POINT_MASS_UNVERIFIED_FOR_HIVE2"',src)
        self.assertNotIn("rclpy",src)
        self.assertNotIn("cmd_vel",src)

    def test_bag_uses_full_g3_and_measures_numerical_solve(self):
        launch=(ROOT/"launch"/"bag_pipeline.launch.py").read_text(encoding="utf-8")
        self.assertIn('executable="n01_g3_optimizer"',launch)
        self.assertNotIn('create_publisher(Twist',launch)
        obs=(PKG/"observer_node.py").read_text(encoding="utf-8")
        self.assertIn("g3_real_bag_data_never_reached_optimizer",obs)
        self.assertIn("g3_no_candidate_path_from_real_bag",obs)
        runner=ROOT.parents[1]/"scripts"/"run_n01_bag_functional.sh"
        content=runner.read_text(encoding="utf-8")
        self.assertIn("g3_synthetic_solver_evidence.json",content)
        self.assertIn("G3 Bag data never reached optimizer",content)
        self.assertNotIn("ros2 bag play",content.split('say "5/5 Launching real Bag')[1])

    def test_synthetic_script_does_not_claim_real_obstacle(self):
        script=ROOT.parents[1]/"scripts"/"g3_offline_solver_demo.py"
        code=script.read_text(encoding="utf-8")
        self.assertIn("SYNTHETIC_KNOWN_MOTION_NOT_REAL_BAG",code)
        self.assertIn('"safety_authorization"',code)
        self.assertNotIn("rclpy",code)


if __name__=="__main__":
    unittest.main()
