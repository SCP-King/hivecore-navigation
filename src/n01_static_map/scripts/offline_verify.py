#!/usr/bin/env python3
# 离线验证:目标 PCD 静态地图 vs 对应 bag 的结构化对应性。
# 纯只读:不启动任何 ROS 节点、不发布、不回放、不改任何数据。
#
# 用法(src/n01_static_map 目录下):
#   python3 -I scripts/offline_verify.py \
#       --pcd /home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd \
#       --bag /home/demo/lyy/slam_data/bags/bag_20261027_225100
#
# 依赖:/opt/ros/humble 已 source(用 rclpy 官方 CDR 反序列化);sqlite3, numpy。
import argparse
import collections
import sqlite3
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 2)[0] + "/n01_static_map")
from pcd_xyz import parse_pcd_xyz  # noqa: E402

try:
    from rclpy.serialization import deserialize_message
    import tf2_msgs.msg
    import sensor_msgs.msg
    import nav_msgs.msg
    HAVE_RCL = True
except Exception as e:  # pragma: no cover
    print(f"[warn] rclpy 反序列化不可用({e}),回退到手工 CDR")
    HAVE_RCL = False


def cdropts(topic_row):
    # topic_row = (id, name, type, serialization_format, offered_qos_profiles)
    q = topic_row[4] or ""
    return f"reliability={'reliable' if 'reliability: 1' in q else 'best_effort' if 'reliability: 2' in q else '?'}"


def summarize_cloud(msg):
    """PointCloud2 -> (n_pts, frame, xmin,xmax,ymin,ymax,zmin,zmax) 或 None"""
    if not HAVE_RCL:
        return None
    w, h = msg.width, msg.height
    if w == 0 or h == 0:
        return None
    ps = msg.point_step
    n = w * h
    xoff = yoff = zoff = None
    for f in msg.fields:
        if f.name == "x" and f.offset is not None:
            xoff = f.offset
        elif f.name == "y":
            yoff = f.offset
        elif f.name == "z":
            zoff = f.offset
    if any(o is None for o in (xoff, yoff, zoff)):
        return None
    x = np.ndarray((n,), dtype="<f4", buffer=msg.data, offset=xoff, strides=(ps,))
    y = np.ndarray((n,), dtype="<f4", buffer=msg.data, offset=yoff, strides=(ps,))
    z = np.ndarray((n,), dtype="<f4", buffer=msg.data, offset=zoff, strides=(ps,))
    good = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    if not good.any():
        return (0, msg.header.frame_id, None, None, None, None, None, None)
    g = good
    return (int(g.sum()), msg.header.frame_id,
            float(x[g].min()), float(x[g].max()), float(y[g].min()),
            float(y[g].max()), float(z[g].min()), float(z[g].max()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pcd", required=True, help="目标静态地图 PCD 绝对路径")
    ap.add_argument("--bag", required=True, help="目标 bag 目录 (含 *.db3)")
    args = ap.parse_args()

    # ---- PCD ----
    xs, ys, zs = parse_pcd_xyz(args.pcd)
    print("=" * 78)
    print("P1 目标 PCD")
    print(f"  file      : {args.pcd}")
    print(f"  pts       : {xs.size}   (全部 finite)")
    print(f"  bounds    : x[{xs.min():.2f},{xs.max():.2f}] "
          f"y[{ys.min():.2f},{ys.max():.2f}] z[{zs.min():.2f},{zs.max():.2f}]")

    # ---- bag ----
    import glob, os
    dbfiles = sorted(glob.glob(os.path.join(args.bag, "*.db3")))
    if not dbfiles:
        print("[error] 未找到 *.db3"); sys.exit(1)
    con = sqlite3.connect(dbfiles[0])
    cur = con.cursor()
    meta = dict(cur.execute("select * from metadata").fetchall())
    start_ns = int(meta.get("starting_time", next(cur.execute(
        "select coalesce(min(timestamp),0) from messages"))[0] or 0))
    print("\nP2 目标 bag 元数据 (rosbag2 sqlite3)")
    for k in ("bagfile_information",):
        print(f"  {k}: {dict(meta).get(k, '(n/a)')}")
    rec = cur.execute("select count(*), min(timestamp), max(timestamp) from messages").fetchone()
    print(f"  messages  : {rec[0]}")
    print(f"  span      : {(rec[2]-rec[1])/1e9:.1f} s  "
          f"start≈{start_ns/1e9:.3f}  end≈{rec[2]/1e9:.3f}")
    rows = cur.execute(
        "select id,name,type,serialization_format,offered_qos_profiles "
        "from topics order by name").fetchall()
    t2id = {r[1]: r[0] for r in rows}
    print("  topics:")
    for r in rows:
        cnt = cur.execute("select count(*) from messages where topic_id=?", (r[0],)).fetchone()[0]
        print(f"    {r[1]:<30} {r[2]:<44} n={cnt:>7}  {cdropts(r)}")

    # ---- PCD vs /cloud_map ----
    print("\nP3 目标 PCD vs bag 内 /cloud_map 对应性 (仅结构化,非逐点)")
    cl = t2id.get("/cloud_map")
    if cl:
        first = last = None
        bounds_first = bounds_last = None
        n_first = n_last = None
        for ts, d in cur.execute(
                "select timestamp,data from messages where topic_id=? order by timestamp",
                (cl,)):
            msg = deserialize_message(d, sensor_msgs.msg.PointCloud2) if HAVE_RCL else None
            if HAVE_RCL:
                s = summarize_cloud(msg)
            else:
                s = None
            if first is None:
                first = ts; n_first, bounds_first = (s[0], s) if s else (None, None)
            last = ts; n_last, bounds_last = (s[0], s) if s else (None, None)
        print(f"  /cloud_map msgs: first@{(first-start_ns)/1e9:.1f}s pts={n_first} "
              f"bounds={bounds_first[2:5] if bounds_first else None}")
        print(f"                  last @{(last-start_ns)/1e9:.1f}s pts={n_last} "
              f"bounds={bounds_last[2:5] if bounds_last else None}")
        if bounds_last:
            print(f"  PCD vs last /cloud_map x: [{bounds_last[2]:.2f},{bounds_last[4]:.2f}] vs "
                  f"[{xs.min():.2f},{xs.max():.2f}] (PCD 应覆盖 >= last cloud_map 范围)")
    else:
        print("  [warn] bag 无 /cloud_map 话题")

    # ---- TF 完整性 ----
    print("\nP4 TF 链完整性 (N-01 map_localized 依赖 map<-lidar_link)")
    tf = t2id.get("/tf"); tfs = t2id.get("/tf_static")
    edges = collections.Counter()
    for tid in (tf, tfs):
        if not tid:
            continue
        for _, d in cur.execute(
                "select timestamp,data from messages where topic_id=?", (tid,)):
            t = deserialize_message(d, tf2_msgs.msg.TFMessage)
            for tr in t.transforms:
                edges[(tr.header.frame_id, tr.child_frame_id)] += 1
    for k in sorted(edges):
        print(f"    {k[0]:>16} -> {k[1]:<16} {edges[k]:6d}")
    need = [("map", "base_footprint"), ("base_footprint", "base_link"),
            ("base_link", "lidar_link")]
    missing = [k for k in need if edges.get(k, 0) == 0]
    print("  结论:", "TF 链完整 ✓" if not missing else f"缺失边 {missing} ✗")

    # ---- /mapping_status 状态采样 (如最终会话正确关闭) ----
    mstat = t2id.get("/mapping_status")
    if mstat:
        cnt = cur.execute("select count(*) from messages where topic_id=?",
                          (mstat,)).fetchone()[0]
        print(f"\nP5 /mapping_status: {cnt} 条 (前3条信息位:)", end=" ")
        for ts, d in cur.execute(
                "select timestamp,data from messages where topic_id=? order by timestamp limit 3",
                (mstat,)):
            # 不确定消息字段,仅作 count 展示
            print(f"[{(ts-start_ns)/1e9:.1f}s]", end=" ")
        print()
    con.close()

    print("\n校验阶段结束:以上均为只读静态验证,未启动任何 ROS 节点/未回放/未发布。")


if __name__ == "__main__":
    main()