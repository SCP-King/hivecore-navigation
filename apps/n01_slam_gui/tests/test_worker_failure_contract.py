#!/usr/bin/env python3
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "slam_lite_gui.py").read_text(encoding="utf-8")


class WorkerFailureContractTests(unittest.TestCase):
    def test_worker_entrypoint_has_top_level_exception_boundary(self):
        self.assertIn("def _run(self) -> None:", SOURCE)
        self.assertIn("self._run_impl()", SOURCE)
        self.assertIn("ROS Worker 异常退出", SOURCE)

    def test_ready_only_after_executor_registration(self):
        self.assertIn("executor.add_node(node)", SOURCE)
        self.assertIn("ROS Worker 已连接：executor 已接管 GUI 节点", SOURCE)
        self.assertLess(SOURCE.index("executor.add_node(node)"),
                        SOURCE.index("ROS Worker 已连接：executor 已接管 GUI 节点"))

    def test_dead_worker_is_reported_to_ui(self):
        self.assertIn("WORKER.thread is not None and not WORKER.thread.is_alive()", SOURCE)
        self.assertIn("ROS Worker 线程已退出", SOURCE)


if __name__ == "__main__":
    unittest.main(verbosity=2)
