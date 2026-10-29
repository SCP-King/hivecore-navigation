"""Regression for two Path types accidentally shadowing one another.

Source-only AST test runs before colcon build, without ROS messages installed.
"""
import ast
from pathlib import Path
import unittest


class ObserverPathAliases(unittest.TestCase):
    def test_pathlib_and_ros_path_distinct(self):
        source = Path(__file__).resolve().parents[1] / "n01_offline_integration" / "observer_node.py"
        module = ast.parse(source.read_text(encoding="utf-8"))
        aliases = {
            (stmt.module, name.name): name.asname
            for stmt in module.body if isinstance(stmt, ast.ImportFrom)
            for name in stmt.names
        }
        self.assertEqual(aliases.get(("pathlib", "Path")), "FilePath")
        self.assertEqual(aliases.get(("nav_msgs.msg", "Path")), "RosPath")
        self.assertNotEqual(aliases[("pathlib", "Path")], aliases[("nav_msgs.msg", "Path")])

        text = source.read_text(encoding="utf-8")
        self.assertIn("self.path = FilePath(target)", text)
        self.assertIn('self.create_subscription(RosPath,"/nav/test/global_path"', text)


if __name__ == "__main__":
    unittest.main()
