#!/usr/bin/env python3
"""Contract checks for mapping + bag recording as one lifecycle."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "slam_lite_gui.py").read_text()
launcher = (ROOT / "run_slam_lite_gui.sh").read_text()
bag_topic_block = source.split("_DEFAULT_BAG_TOPICS = (", 1)[1].split(")", 1)[0]

topics = [
    "/livox/lidar", "/livox/imu", "/livox/lidar/geo_filtered",
    "/imu/data", "/imu/rpy/filtered", "/odom",
    "/tf", "/tf_static", "/robot_pose", "/mapping_status", "/cloud_map",
]
checks = {
    "bag topic set": all(t in source for t in topics),
    "bag dir/path": "MAPPING_BAG_DIR" in source and 'bag_{stamp}' in source,
    "start mapping starts bag": "if mode == \"mapping\":" in source
                                 and "if not self.start_bag():" in source,
    "bag failure blocks desk": "建图 bag 录制启动失败；不会继续启动建图 desk" in source,
    "desk failure stops bag": "建图 desk 启动失败，停止同时启动的 bag 录制" in source,
    "stop motion and bag before desk": (
        '("slow_motion", "bag", "desk", "rviz", "nav_rviz", "foxglove")' in source),
    "mapping close saves first": "_close_mapping_session" in source
                                and "PROCS.deactivate_mapping" in source,
    "mapping close stops bag": 'await asyncio.to_thread(PROCS.stop, "bag")' in source,
    "stop desk mapping path": "if mode == \"mapping\":" in source
                              and "建图已保存，bag 已停止" in source,
    "desk exit stops orphan bag": "同步停止 bag 录制，避免孤儿录包" in source,
    "shutdown saves mapping and stops bag": "退出前建图 PCD 已保存" in source
                                            and 'PROCS.stop("bag")' in source,
    "bag env preserved in launcher": "N01_MAPPING_BAG_DIR N01_MAPPING_BAG_TOPICS" in launcher,
    "no actuator topic recorded": "/cmd_vel" not in bag_topic_block,
}
failed = []
for name, ok in checks.items():
    print(("PASS " if ok else "FAIL ") + name)
    if not ok:
        failed.append(name)
raise SystemExit(1 if failed else 0)
