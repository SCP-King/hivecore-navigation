import importlib.util
import unittest
from pathlib import Path
from unittest.mock import patch


SCRIPT = Path(__file__).with_name("n01_stop_live_stack.py")
SPEC = importlib.util.spec_from_file_location("n01_stop_live_stack", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ScopedN01CleanupTests(unittest.TestCase):
    def test_matches_navigation_executable(self):
        with patch.object(MODULE, "process_cmdline", return_value=[
            "/home/demo/lyy/hivecore-navigation/install/n01_live_navigation/lib/"
            "n01_live_navigation/n01_live_scan_bridge"
        ]):
            self.assertEqual(MODULE.is_managed_process(1234), (True, "n01_live_scan_bridge"))

    def test_matches_scan_planner_but_not_other_planners(self):
        with patch.object(MODULE, "process_cmdline", return_value=[
            "/home/demo/lyy/hivecore-navigation/install/scan_planner/lib/"
            "scan_planner/scan_planner_node"
        ]):
            self.assertEqual(MODULE.is_managed_process(1234), (True, "scan_planner_node"))
        with patch.object(MODULE, "process_cmdline", return_value=[
            "/opt/ros/humble/lib/nav2_controller/controller_server"
        ]):
            self.assertEqual(MODULE.is_managed_process(1234), (False, None))

    def test_never_matches_independent_localization_adapter(self):
        with patch.object(MODULE, "process_cmdline", return_value=[
            "/home/demo/lyy/hivecore-navigation/install/n01_live_navigation/lib/"
            "n01_live_navigation/n01_localization_adapter"
        ]):
            self.assertEqual(MODULE.is_managed_process(1234), (False, None))


    def test_process_cmdline_splits_procfs_nul_delimiters(self):
        with patch.object(MODULE.Path, "read_bytes", return_value=b"/usr/bin/demo\\x00--flag\\x00"):
            self.assertEqual(MODULE.process_cmdline(1234), ["/usr/bin/demo", "--flag"])

    def test_proc_stat_extracts_process_group(self):
        stat_line = "123 (ros2 launch) S 1 123 123 0 -1 0 0 0 0 0 0 0 0 0 20 0 1 0 0"
        with patch.object(MODULE.Path, "read_text", return_value=stat_line):
            parsed = MODULE.proc_stat(123)
        self.assertEqual(parsed["state"], "S")
        self.assertEqual(parsed["pgrp"], 123)
        self.assertEqual(parsed["session"], 123)

    def test_matches_only_n01_rviz_config(self):
        with patch.object(MODULE, "process_cmdline", return_value=[
            "/opt/ros/humble/lib/rviz2/rviz2", "-d",
            "/home/demo/lyy/hivecore-navigation/install/share/"
            "n01_live_navigation/rviz/n01_live_navigation.rviz"
        ]):
            self.assertEqual(MODULE.is_managed_process(1234), (True, "N-01 RViz"))


if __name__ == "__main__":
    unittest.main()
