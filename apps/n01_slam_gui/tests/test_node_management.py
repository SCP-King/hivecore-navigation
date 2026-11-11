"""Node categorization must not grant process-stop authority."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

GUI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(GUI))
import slam_lite_gui as g

spec = importlib.util.spec_from_file_location(
    'livox_preflight', GUI.parents[1] / 'scripts/livox_start_preflight.py')
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


class NodeManagementTests(unittest.TestCase):
    def test_execution_nodes_are_not_classified_as_planning(self):
        self.assertEqual(g.node_category('/n01_live_slow_motion_executor')[0], '底盘与执行')
        self.assertEqual(g.node_category('/robot/n01_traj_tracker')[0], '底盘与执行')
        self.assertEqual(g.node_category('/scan_planner_node')[0], '导航规划')
        self.assertEqual(g.node_category('/n01_localization_adapter')[0], '适配器')
        self.assertEqual(g.node_category('/unknown')[0], '其他')

    def test_livox_process_matching_keeps_external_driver_protected(self):
        row = dict(pid=991022, ppid=1, pgid=991022,
                   cmd='/opt/hivecore/lib/livox_ros_driver2/livox_ros_driver2_node')
        plan = g.node_kill_plan('livox_lidar_publisher', [row])
        self.assertEqual(plan['killable'], [])
        self.assertEqual(plan['protected'], [row])

    def test_driver_preflight_reuses_single_compatible_source(self):
        pc2 = SimpleNamespace(topic_type='sensor_msgs/msg/PointCloud2')
        custom = SimpleNamespace(topic_type='livox_ros_driver2/msg/CustomMsg')
        self.assertEqual(preflight.decision([pc2], 1)[0], 0)
        self.assertEqual(preflight.decision([], 0)[0], 10)
        for endpoints, names in [([pc2, pc2], 2), ([custom], 1), ([], 1), ([pc2], 2)]:
            self.assertEqual(preflight.decision(endpoints, names)[0], 2)


if __name__ == '__main__':
    unittest.main()
