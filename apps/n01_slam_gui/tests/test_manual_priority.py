"""Exercise executor ownership callbacks without publishing to a robot."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

SOURCE = Path(__file__).resolve().parents[3] / 'src/n01_live_navigation/n01_live_navigation/slow_motion_executor.py'


class ManualPriorityTests(unittest.TestCase):
    def setUp(self):
        tree = ast.parse(SOURCE.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SlowMotionExecutor')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
                   {'_on_command_observed', '_control', '_zero'}]
        namespace = {'Twist': Mock(), 'CMD_VEL_TOPIC': '/cmd_vel', 'time': SimpleNamespace(monotonic=lambda: 10.)}
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), 'exec'), namespace)
        self.methods = namespace
        self.node = SimpleNamespace(command_publishers={}, manual_override=False, manual_source='',
            observer_ready=True, _poll_command_observer=Mock(),
            get_name=lambda: 'n01_slow_motion_executor', get_namespace=lambda: '/',
            get_publishers_info_by_topic=lambda _: [
                SimpleNamespace(endpoint_gid=[1], node_name='n01_slow_motion_executor', node_namespace='/'),
                SimpleNamespace(endpoint_gid=[2], node_name='cmd_wheel_preprocessor_node', node_namespace='/')],
            cmd_pub=Mock(), _status=Mock())

    def test_manual_zero_also_takes_priority_and_control_does_not_publish(self):
        self.methods['_on_command_observed'](self.node, '/cmd_wheel_preprocessor_node')
        self.assertTrue(self.node.manual_override)
        self.methods['_control'](self.node)
        self.node.cmd_pub.publish.assert_not_called()
        self.assertEqual(self.node._status.call_args.args[0], 'MANUAL_OVERRIDE')

    def test_idle_publisher_presence_does_not_take_control(self):
        self.assertFalse(self.node.manual_override)
        self.methods['_zero'](self.node, 'NO_PATH')
        self.node.cmd_pub.publish.assert_called_once()

    def test_zero_guard_does_not_overwrite_manual_control(self):
        self.node.manual_override = True
        self.methods['_zero'](self.node, 'DISABLED')
        self.node.cmd_pub.publish.assert_not_called()

    def test_unknown_actual_publisher_yields_conservatively(self):
        self.methods['_on_command_observed'](self.node, 'unknown_external_publisher')
        self.assertTrue(self.node.manual_override)


if __name__ == '__main__':
    unittest.main()
