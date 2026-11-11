#!/usr/bin/env python3
"""test_lifecycle.py — GUI 节点生命周期 / 孤儿清理的回归测试（不需要 ROS 图）。

    source /opt/ros/humble/setup.bash
    source /opt/slam3d_release/setup_runtime.sh
    python3 tests/test_lifecycle.py

覆盖：
  · ps 快照带 etime/pgid；_fmt_proc 格式
  · host_procs 的白名单命中 / 宽松匹配 / 短名不猜
  · collect_unmanaged_residue 的过滤规则（ppid、保护名单、本进程组、范围）
  · 未托管残留仅只读展示，不提供自动清理接口
  · start_desk 在前置链残留 / 普查过期时拒绝，skip_rsp 放行地盘 RSP
全程不发 ROS 请求、不启动 GUI。
"""

import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import slam_lite_gui as g  # noqa: E402

fails = []


def check(name, cond, detail=""):
    print(("PASS  " if cond else "FAIL  ") + name + (f"  {detail}" if detail and not cond else ""))
    if not cond:
        fails.append(name)


def drain():
    out = []
    while True:
        try:
            out.append(g.STATE.log_q.get_nowait())
        except Exception:
            return out


# 1. ps 快照带 etime/pgid
snap = g._ps_snapshot()
me = next((r for r in snap if r["pid"] == os.getpid()), None)
check("ps has etime", bool(me and me.get("etime")), me)
check("ps has pgid", bool(me and me.get("pgid")), me)

# 2. 格式化
check("fmt_proc", g._fmt_proc({"pid": 5, "pgid": 5, "etime": "01:02"})
      == "pid 5（pgid 5，up 01:02）")
check("fmt_proc minimal", g._fmt_proc({"pid": 5}) == "pid 5")

# 3. host_procs：白名单命中 / 宽松匹配 / 短名跳过
known, is_known = g.host_procs("complementary_filter_gain_node",
                               [{"pid": 991011, "ppid": 2, "pgid": 991011, "etime": "1m",
                                 "cmd": "/opt/x/complementary_filter_node --ros-args"}])
check("known hint matched", is_known and [p["pid"] for p in known] == [991011], known)
avail, is_known = g.host_procs("runtime_supervisor",
                               [{"pid": 12, "ppid": 2, "pgid": 12, "etime": "1m",
                                 "cmd": "python3 -m runtime_supervisor"}])
check("fallback matched", (not is_known) and [p["pid"] for p in avail] == [12], avail)
avail, _ = g.host_procs("tf", [{"pid": 13, "ppid": 2, "pgid": 13, "etime": "1m",
                              "cmd": "python3 /tmp/tf"}])
check("short name not guessed", avail == [], avail)
avail, _ = g.host_procs("n01/live_navigation/scan_bridge",
                        [{"pid": 14, "ppid": 2, "pgid": 14, "etime": "3m",
                          "cmd": "python3 .../scan_bridge --ros-args"}])
check("namespaced fallback", [p["pid"] for p in avail] == [14], avail)

# 4. 孤儿过滤
my_pgid = os.getpgid(0)
syn = [
    {"pid": 101, "ppid": 1, "pgid": 101, "etime": "2m", "cmd": "x/complementary_filter_node"},
    {"pid": 102, "ppid": 2, "pgid": 101, "etime": "2m", "cmd": "x/complementary_filter_node"},
    {"pid": 103, "ppid": 1, "pgid": 103, "etime": "2m", "cmd": "/opt/hivecore/x/mount_tf_publisher_node"},
    {"pid": 104, "ppid": 1, "pgid": 104, "etime": "2m", "cmd": "python3 unrelated_thing"},
    {"pid": 105, "ppid": 1, "pgid": my_pgid, "etime": "2m", "cmd": "x/localization_lite_node"},
    {"pid": 106, "ppid": 1, "pgid": 106, "etime": "2m", "cmd": "ros2 launch pkg localization_lite_desk.launch.py"},
]
got = sorted(r["pid"] for r in g.collect_unmanaged_residue(syn))
check("orphan filter", got == [101, 106], got)

# 5. 未托管残留仅用于只读展示，不存在自动 kill 接口
check("unmanaged residue is read-only", callable(g.collect_unmanaged_residue)
      and not hasattr(g, "cleanup_orphans"))

# 6. /proc starttime is available for this live process and guards PID reuse
check("proc starttime available", g._proc_start_ticks(os.getpid()) is not None)

# 7. 前置链残留 → 拒绝启动
mgr = g.ProcManager()
spawned = []
mgr._spawn = lambda *a, **k: spawned.append(a) or True
with g.STATE.lock:
    g.STATE.loc_node_up = g.STATE.map_node_up = False
    g.STATE.census_at = time.monotonic()
    g.STATE.node_rows = [{"name": "/complementary_filter_gain_node", "count": 1,
                          "state": "正常", "killable": True}]
drain()
ok = mgr.start_desk("loc", skip_rsp=True)
logs = drain()
check("residue blocks start", ok is False and not spawned, (ok, spawned))
check("residue log says 拒绝", any("拒绝启动" in x for x in logs), logs)

# 8. skip_rsp=True 时，图里的地盘 RSP 不算残留
with g.STATE.lock:
    g.STATE.loc_node_up = g.STATE.map_node_up = False
    g.STATE.census_at = time.monotonic()
    g.STATE.node_rows = [{"name": "/robot_state_publisher", "count": 1,
                          "state": "正常", "killable": False}]
drain()
spawned.clear()
ok = mgr.start_desk("loc", skip_rsp=True)
check("chassis RSP allowed", ok is True and len(spawned) == 1, (ok, spawned))

# 9. skip_rsp=False 时同一张图 → 拒绝（TF 冲突）
with g.STATE.lock:
    g.STATE.loc_node_up = g.STATE.map_node_up = False
    g.STATE.census_at = time.monotonic()
    g.STATE.node_rows = [{"name": "/robot_state_publisher", "count": 1,
                          "state": "正常", "killable": False}]
drain()
spawned.clear()
ok = mgr.start_desk("loc", skip_rsp=False)
logs = drain()
check("RSP conflict blocked", ok is False and not spawned
      and any("robot_state_publisher" in x for x in logs), (ok, spawned, logs))

# 10. 普查过期 → 拒绝（看不到图里已有的 desk 时不能起）
with g.STATE.lock:
    g.STATE.loc_node_up = g.STATE.map_node_up = False
    g.STATE.census_at = 0.0
    g.STATE.node_rows = []
drain()
spawned.clear()
ok = mgr.start_desk("loc", skip_rsp=True)
logs = drain()
check("stale census refuses", ok is False and not spawned
      and any("普查" in x for x in logs), (ok, logs))

# 11. FAST-LIO2 定位必须走仓库内 nav bridge，并把参考 PCD 显式传给 launch
fast_map = "/tmp/n01_gui_fast_lio_reference.pcd"
with open(fast_map, "w", encoding="utf-8") as fh:
    fh.write("# test fixture\n")
with g.STATE.lock:
    g.STATE.census_at = time.monotonic()
    g.STATE.node_rows = []
spawned.clear()
ok = mgr.start_fast_lio("loc", fast_map, "/tmp/n01_gui_fast_lio_out.pcd")
cmd = spawned[0][1] if spawned else []
check("FAST-LIO localization uses unified bridge",
      ok is True and any("start_fast_lio_gui_backend.sh" in part for part in cmd)
      and "localization" in cmd
      and f"saved_map_path:={fast_map}" in cmd, (ok, cmd))
spawned.clear()
ok = mgr.start_fast_lio_rviz()
rviz_cmd = spawned[0][1] if spawned else []
check("FAST-LIO RViz uses repository profile",
      ok is True and rviz_cmd[:2] == ["rviz2", "-d"]
      and rviz_cmd[-1].endswith("/src/n01_fast_lio/rviz/fast_lio.rviz"),
      (ok, rviz_cmd))
rviz_profile = ""
if rviz_cmd:
    with open(rviz_cmd[-1], encoding="utf-8") as fh:
        rviz_profile = fh.read()
check("FAST-LIO RViz exposes manual initial pose",
      "rviz_default_plugins/SetInitialPose" in rviz_profile
      and "Value: /initialpose" in rviz_profile
      and "Fixed Frame: map" in rviz_profile
      and "Value: /n01/fast_lio/reference_map" in rviz_profile)
spawned.clear()
mgr.start_bag = lambda: True
ok = mgr.start_fast_lio("realtime", "", "/tmp/n01_gui_realtime_map.pcd")
realtime_cmd = spawned[0][1] if spawned else []
check("FAST-LIO realtime uses live navigation bridge",
      ok is True and "realtime" in realtime_cmd
      and any("start_fast_lio_gui_backend.sh" in part for part in realtime_cmd),
      (ok, realtime_cmd))

with g.STATE.lock:
    g.STATE.loc_node_up = g.STATE.map_node_up = False
    g.STATE.census_at = time.monotonic()
    g.STATE.node_rows = []

print()
if fails:
    print("FAILED:", fails)
    sys.exit(1)
print("ALL PASS")
