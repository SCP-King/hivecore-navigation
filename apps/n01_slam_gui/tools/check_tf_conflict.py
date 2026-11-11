#!/usr/bin/env python3
"""check_tf_conflict.py — 只读检查 SLAM 与运动栈的 base_footprint→base_link 冲突。

    source /opt/ros/humble/setup.bash
    python3 tools/check_tf_conflict.py [采样秒数，默认 5]

输出：
  · /tf_static 与 /tf 上 base_footprint→base_link 出现过的所有 z 值及次数
    （同时出现 0.30 和 0.90 = 两套 URDF 在抢同一条边）
  · /robot_description、/tf_static、/tf 的发布节点
  · base_link→lidar_link 的数值（两栈应一致，用于区分“冲突只发生在 footprint 高度”）

只读：只创建临时节点订阅图数据，不发 TF、不改任何文件。
"""

import os
import sys
import tempfile
import time

os.environ.setdefault("ROS_LOG_DIR",
                      os.path.join(tempfile.gettempdir(), "n01_tf_probe_log"))
os.makedirs(os.environ["ROS_LOG_DIR"], exist_ok=True)

import rclpy                                     # noqa: E402
from rclpy.duration import Duration              # noqa: E402
from rclpy.node import Node                      # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)
from tf2_msgs.msg import TFMessage               # noqa: E402

WATCH = [
    ("base_footprint", "base_link"),
    ("base_link", "lidar_link"),
    ("base_link", "imu_link"),
]

STATIC_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
DYNAMIC_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)


def short(node_name: str, ns: str) -> str:
    ns = (ns or "/").rstrip("/")
    return f"{ns}/{node_name}" if ns not in ("", "/") else f"/{node_name}"


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = Node(f"n01_tf_probe_{os.getpid()}")

    seen = {}   # (topic, parent, child) -> {z: count}

    def record(topic):
        def _cb(msg: TFMessage) -> None:
            for tr in msg.transforms:
                key = (topic, tr.header.frame_id.lstrip("/"),
                       tr.child_frame_id.lstrip("/"))
                if (key[1], key[2]) not in WATCH:
                    continue
                z = round(float(tr.transform.translation.z), 4)
                seen.setdefault((key[1], key[2], topic), {}).setdefault(z, 0)
                seen[(key[1], key[2], topic)][z] += 1
        return _cb

    node.create_subscription(TFMessage, "/tf_static", record("/tf_static"),
                             STATIC_QOS)
    node.create_subscription(TFMessage, "/tf", record("/tf"), DYNAMIC_QOS)

    publishers = {}
    for topic in ("/robot_description", "/tf_static", "/tf"):
        try:
            infos = node.get_publishers_info_by_topic(topic)
            publishers[topic] = sorted(
                {short(i.node_name, i.node_namespace) for i in infos})
        except Exception as exc:                 # noqa: BLE001
            publishers[topic] = [f"查询失败: {exc}"]

    print(f"采样 {seconds:.0f}s（Ctrl+C 提前结束）…")
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("\n========== 发布节点 ==========")
    for topic, names in publishers.items():
        print(f"  {topic:20s} {len(names)} 个发布者")
        for n in names:
            print(f"      - {n}")

    print("\n========== base_footprint→base_link 采样值 ==========")
    key = ("base_footprint", "base_link")
    rows = [(topic, zc) for (p, c, topic), zc in seen.items()
            if (p, c) == key]
    if not rows:
        print("  未收到任何 base_footprint→base_link 变换")
    for topic, zc in sorted(rows):
        detail = "、".join(f"z={z} × {n}" for z, n in sorted(zc.items()))
        print(f"  {topic:12s} {detail}")
    zs = sorted({z for _, zc in rows for z in zc})
    if len(zs) > 1:
        print(f"  ⚠ 同一条边出现多个 z 值 {zs} → 两套 URDF 同时发布同一 joint")
    elif zs:
        print(f"  ✓ 只有单一 z={zs[0]}（另一套栈当前没有在发）")

    print("\n========== 其它同名边 ==========")
    for p, c in WATCH[1:]:
        for (pp, cc, topic), zc in sorted(seen.items()):
            if (pp, cc) != (p, c):
                continue
            detail = "、".join(f"z={z} × {n}" for z, n in sorted(zc.items()))
            print(f"  {p}→{c:12s} [{topic}] {detail}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
