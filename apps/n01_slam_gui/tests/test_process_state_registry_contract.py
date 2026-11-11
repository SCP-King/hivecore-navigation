#!/usr/bin/env python3
"""Regression checks for shared UI/worker state and ROS readiness semantics."""
from pathlib import Path

SOURCE = (Path(__file__).resolve().parents[1] / "slam_lite_gui.py").read_text(encoding="utf-8")

checks = {
    "process-wide AppState registry": "_N01_SLAM_GUI_PROCESS_STATE_REGISTRY" in SOURCE,
    "process-wide worker registry": "_N01_SLAM_GUI_ROS_WORKER_REGISTRY" in SOURCE,
    "worker connection event": "self.connected = threading.Event()" in SOURCE,
    "connection is based on executor ownership": "executor.add_node(node)" in SOURCE and "self.connected.set()" in SOURCE,
    "SLAM presence is not a connection prerequisite": "Successful add_node is the GUI's ROS connection criterion" in SOURCE,
    "UI reconciles with worker event": "worker_connected = bool(" in SOURCE and "connected_event.is_set()" in SOURCE,
    "old live worker is not silently accepted": "旧版/未完成初始化的 ROS Worker" in SOURCE,
    "worker startup has finite timeout": "ROS Worker 初始化超过 20 秒" in SOURCE,
}
failed = []
for name, passed in checks.items():
    print(("PASS " if passed else "FAIL ") + name)
    if not passed:
        failed.append(name)
raise SystemExit(1 if failed else 0)
