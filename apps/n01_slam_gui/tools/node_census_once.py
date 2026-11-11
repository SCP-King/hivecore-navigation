#!/usr/bin/env python3
"""node_census_once.py — 只读的一次性节点普查（rclpy 图 API，不走 ros2 CLI）。

为什么不用 `ros2 node list`：
  该命令依赖 ros2 daemon；CLI 环境异常时可能只输出 traceback，导致节点统计失真。
  本工具直接通过 rclpy 图 API 读取当前 ROS_DOMAIN_ID 的发现结果。
  本脚本直接 import rclpy 查 DDS 图，不启子进程、不需要 daemon。

用法：
    source /opt/ros/humble/setup.bash
    python3 tools/node_census_once.py

全程只读：只调用图查询（get_node_names_and_namespaces /
get_publishers_info_by_topic），不发信号、不启动/停止任何东西。
"""

import os
import sys
import tempfile
import time

KEY_TOPICS = [
    "/imu/data",
    "/livox/imu",
    "/livox/lidar/geo_filtered",
    "/loc_status",
    "/robot_pose",
    "/localization_map",
    "/cloud_map",
]

# 与 GUI 的 NODE_EXEC_HINTS 对齐：节点名 → 可执行文件片段
EXEC_HINTS = {
    "complementary_filter_gain_node": ("complementary_filter_node",),
    "lidar_cloud_preprocess_node": ("lidar_cloud_preprocess_node",),
    "robot_state_publisher": ("mount_tf_publisher_node", "robot_state_publisher"),
    "static_tf_publisher_node": ("static_tf_publisher_node",),
    "localization_lite_node": ("localization_lite_node",),
    "mapping_lite_node": ("mapping_lite_node",),
}

READ_ONLY_DENY = ("/opt/hivecore", "livox", "chassis", "controller_manager",
                  "hive2_leg", "legs_", "joy_", "battery", "wheel", "ethercat",
                  "motor", "lpms", "person_follow")


def ps_snapshot():
    out = os.popen("ps -eo pid=,ppid=,pgid=,etime=,cmd=").read()
    rows = []
    for line in out.splitlines():
        parts = line.split(None, 4)
        if len(parts) < 5:
            continue
        try:
            rows.append({"pid": int(parts[0]), "ppid": int(parts[1]),
                         "pgid": int(parts[2]), "etime": parts[3],
                         "cmd": parts[4]})
        except ValueError:
            continue
    return rows


def tree_tag(cmd):
    if "/opt/hivecore" in cmd:
        return "甲方栈 /opt/hivecore"
    if "/opt/slam3d_release" in cmd:
        return "甲方 SLAM3D Lite 综合包"
    return "其他"


def protected(cmd):
    return any(t in cmd for t in READ_ONLY_DENY)


def main():
    # rclpy 默认把日志写到 ~/.ros/log，只读文件系统上会 init 失败
    os.environ.setdefault("ROS_LOG_DIR",
                          os.path.join(tempfile.gettempdir(), "n01_census_log"))
    os.makedirs(os.environ["ROS_LOG_DIR"], exist_ok=True)
    try:
        import rclpy
        from rclpy.node import Node
    except Exception as exc:  # noqa: BLE001
        print(f"rclpy 导入失败: {exc!r}")
        print("请先 source /opt/ros/humble/setup.bash，再 source SLAM3D Lite runtime")
        return 2

    rclpy.init()
    node = Node(f"n01_census_probe_{os.getpid()}")
    try:
        time.sleep(1.5)                      # 给 DDS 发现一点时间
        pairs = node.get_node_names_and_namespaces()
        counts = {}
        for name, ns in pairs:
            key = (ns.rstrip("/") + "/" + name) if ns not in ("", "/") else "/" + name
            counts[key] = counts.get(key, 0) + 1

        print("========== 1. 全部节点 ==========")
        for k in sorted(counts):
            mark = "  ← 重名" if counts[k] > 1 else ""
            print(f"  {counts[k]} × {k}{mark}")
        dups = sorted(k for k, c in counts.items() if c > 1)
        print(f"  合计 {len(counts)} 个唯一节点名，{len(pairs)} 个实例，"
              f"重名 {len(dups)} 组")

        print("\n========== 2. 关键话题发布者 ==========")
        for topic in KEY_TOPICS:
            try:
                infos = node.get_publishers_info_by_topic(topic)
            except Exception as exc:  # noqa: BLE001
                print(f"  {topic:32s} 查询失败: {exc}")
                continue
            names = sorted({(i.node_namespace.rstrip("/") + "/" + i.node_name)
                            if i.node_namespace not in ("", "/") else "/" + i.node_name
                            for i in infos})
            flag = "  ← 多源" if len(names) > 1 else ""
            print(f"  {topic:32s} {len(names)} 个发布者{flag}")
            for n in names:
                print(f"      - {n}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\n========== 3. 宿主进程（含孤儿标记）==========")
    snap = ps_snapshot()
    me = os.getpid()
    hits = []
    for r in snap:
        if r["pid"] == me:
            continue
        for node_name, hints in EXEC_HINTS.items():
            if any(h in r["cmd"] for h in hints):
                hits.append((node_name, r))
                break
    if not hits:
        print("  （没有找到 desk 链进程）")
    for node_name, r in sorted(hits, key=lambda x: x[1]["pid"]):
        orphan = "  ← 孤儿（父进程已死）" if r["ppid"] == 1 else ""
        prot = "  [受保护]" if protected(r["cmd"]) else ""
        print(f"  {node_name:30s} pid {r['pid']:<8} ppid {r['ppid']:<8} "
              f"pgid {r['pgid']:<8} up {r['etime']:<10} {tree_tag(r['cmd'])}"
              f"{orphan}{prot}")
        print(f"      {r['cmd'][:150]}")

    # 白名单外的重名节点（例如 N01 导航栈）：按节点短名宽松匹配，只展示不停止。
    # 目的是回答“这两个同名实例分别是哪个 pid / 哪次启动”。
    print("\n========== 3b. 重名节点的宿主进程（白名单外，仅展示）==========")
    known = {n for n, _ in hits}
    extra = 0
    for name in dups:
        if name.lstrip("/") in known:
            continue
        base = name.rstrip("/").rsplit("/", 1)[-1]
        if len(base) < 5:
            continue
        procs = [r for r in snap if r["pid"] != me and base in r["cmd"]]
        if not procs:
            continue
        extra += 1
        print(f"  {name}  ×{counts[name]}")
        for r in procs:
            orphan = "  ← 孤儿（父进程已死）" if r["ppid"] == 1 else ""
            prot = "  [受保护]" if protected(r["cmd"]) else ""
            print(f"      pid {r['pid']:<8} ppid {r['ppid']:<8} pgid {r['pgid']:<8} "
                  f"up {r['etime']:<10} {tree_tag(r['cmd'])}{orphan}{prot}")
            print(f"          {r['cmd'][:150]}")
    if not extra:
        print("  （没有需要额外定位的重名节点）")

    orphans = [(n, r) for n, r in hits if r["ppid"] == 1 and not protected(r["cmd"])]
    print("\n========== 4. 结论 ==========")
    if dups:
        print(f"  重名 {len(dups)} 组：{', '.join(dups)}")
        print("  同一 pgid = 同一次 launch 起的两份（脚本/参数重复）；"
              "不同 pgid = 两次独立启动。")
    if orphans:
        print("  可清理的孤儿（可直接 kill，不会触发 respawn）：")
        for n, r in orphans:
            print(f"    pid {r['pid']}  {n}")
    if not dups and not orphans:
        print("  节点无重名、无孤儿残留。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
