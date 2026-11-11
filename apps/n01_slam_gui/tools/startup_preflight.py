#!/usr/bin/env python3
"""Safe startup census for the unified N-01 GUI.

Two-stage policy:
1) refresh the local ROS CLI daemon and census the ROS graph;
2) if an N-01 navigation stack is present, ask the repository's scoped
   lifecycle entrypoint to stop it, then census again.

Only the navigation repository's own lifecycle manager may stop N-01 navigation.
Lite SLAM, localization adapter, Livox, IMU, chassis and controller processes are
reported/protected. Ambiguous processes are never killed by this preflight.
"""
from __future__ import annotations

import collections
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAV_HELPER = ROOT / "tools" / "navigation_lifecycle.py"
# Nodes that only the navigation launch can own. /n01_live_scan_bridge is
# deliberately NOT here: it is started by the localization data path
# (n01_localization_stack.launch.py: desk -> adapter -> bridge) while
# n01_navigation.sh passes start_scan_bridge:=false.  It is resolved
# dynamically in classify_scan_bridge() instead of being assumed to be a
# navigation residual; scripts/n01_navigation.sh's managed_nodes_absent()
# excludes it for the same reason.
NAV_MARKERS = {
    "/n01_navigation_path",
    "/n01_static_map",
    "/n01_live_runtime_supervisor",
    "/n01_live_safety_supervisor",
    "/n01_live_dynamic_obstacle_tracker",
    "/n01_hive2_trajectory_monitor",
    "/n01_live_scan_goal_gate",
    "/scan_planner_node",
}
SCAN_BRIDGE = "/n01_live_scan_bridge"
# Any of these proves the localization data path is running, and therefore
# that the bridge node belongs to it (they are started by the same launch).
LOCALIZATION_STACK_MARKERS = {
    "/n01_localization_adapter",
    "/localization_lite_node",
    "/mapping_lite_node",
}


def classify_scan_bridge(nodes: list[str], active_pid: int | None) -> tuple[str, str]:
    """Resolve the scan bridge's owner from the live graph.

    Returns (state, detail) where state is one of:
      "absent"    bridge not in the graph
      "nav"       a managed navigation launch is active; the navigation launch
                  owns it (start_scan_bridge defaults to true)
      "upstream"  the localization data path is running; the bridge is its
                  documented output (desk -> adapter -> /cloud + /body_pose)
      "orphan"    no owner can be proven; refuse rather than start on top of it
    """
    if SCAN_BRIDGE not in nodes:
        return "absent", ""
    if active_pid is not None:
        return "nav", f"managed navigation PID={active_pid}"
    loc = sorted(set(nodes) & LOCALIZATION_STACK_MARKERS)
    if loc:
        return "upstream", ", ".join(loc)
    return "orphan", "no localization stack node and no managed navigation PID"
PROTECTED_MARKERS = {
    "/localization_lite_node",
    "/mapping_lite_node",
    "/n01_localization_adapter",
    "/livox_lidar_publisher",
    "/lpms_ig1_calib",
    "/lpms_imu_broadcaster",
    "/chassis_controller",
    "/hive2_leg_controller_node",
    "/controller_manager",
    "/ethercat_io_m0",
}


def run(args: list[str], timeout: float = 10.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, timeout=timeout, check=False,
                          env=os.environ.copy())


def census() -> list[str]:
    # Stop only the local CLI discovery daemon, never ROS application nodes.
    try:
        run(["ros2", "daemon", "stop"], timeout=5.0)
    except (OSError, subprocess.TimeoutExpired):
        pass
    cp = run(["ros2", "node", "list"], timeout=12.0)
    if cp.returncode != 0:
        raise RuntimeError(f"ros2 node list failed ({cp.returncode}):\n{cp.stdout[-2000:]}")
    return [line.strip() for line in cp.stdout.splitlines() if line.strip().startswith("/")]


def active_managed_navigation_pid() -> int | None:
    """Return the PID only when the lifecycle PID file points to the expected launch."""
    runtime = Path(os.environ.get("N01_RUNTIME_DIR", "~/lyy/n01_runtime")).expanduser()
    pid_file = runtime / "runs" / "navigation" / "live_scan_launch.pid"
    try:
        pid = int(pid_file.read_text().strip())
        cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\\0", b" ").decode("utf-8", "replace")
        stat_raw = Path(f"/proc/{pid}/stat").read_text()
        stat_fields = stat_raw.rsplit(") ", 1)[1].split()
        pgrp, session = int(stat_fields[2]), int(stat_fields[3])
    except (OSError, ValueError, IndexError):
        return None
    expected = ("ros2", "launch", "n01_live_navigation", "live_scan.launch.py")
    if pgrp == pid and session == pid and all(token in cmdline for token in expected):
        return pid
    return None


def stop_scoped_navigation(nodes: list[str]) -> int:
    found = sorted(set(nodes) & NAV_MARKERS)
    if not found:
        return 0
    active_pid = active_managed_navigation_pid()
    if active_pid is not None:
        print(f"发现 N-01 导航节点，但 PID {active_pid} 与生命周期 PID 文件一致；"
              "判定为正在运行的受管实例，不作为残留清理。", flush=True)
        return 0
    print("发现无有效生命周期主 PID 的 N-01 导航节点：", ", ".join(found), flush=True)
    print("仅调用导航仓库的受管 stop 入口；不触碰 SLAM/传感器/底盘。", flush=True)
    cp = run([sys.executable, str(NAV_HELPER), "stop"], timeout=55.0)
    print(cp.stdout, end="" if cp.stdout.endswith("\n") else "\n", flush=True)
    if cp.returncode != 0:
        print("ERROR: N-01 导航受管清理失败；拒绝启动 GUI 管理流程。", file=sys.stderr)
        return cp.returncode
    return 0


def main() -> int:
    print("===== N-01 GUI STARTUP PREFLIGHT: STAGE 1 / CENSUS =====", flush=True)
    try:
        nodes = census()
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        print(f"ERROR: ROS 节点普查失败：{exc}", file=sys.stderr)
        return 20

    counts = collections.Counter(nodes)
    duplicates = sorted(name for name, count in counts.items() if count > 1)
    protected = sorted(set(nodes) & PROTECTED_MARKERS)
    if protected:
        print("受保护的上游/SLAM 节点（保留，不自动停止）：", ", ".join(protected), flush=True)
    bridge_state, bridge_detail = classify_scan_bridge(
        nodes, active_managed_navigation_pid())
    if bridge_state == "upstream":
        print(f"/n01_live_scan_bridge 归属定位数据链（{bridge_detail}），"
              "按上游保留，不作为导航残留。", flush=True)
    elif bridge_state == "orphan":
        print(f"/n01_live_scan_bridge 无法证明归属（{bridge_detail}）；"
              "导航 stop 不会清理它，见最终普查。", flush=True)
    if duplicates:
        print("WARNING: ROS 图存在重名节点：", ", ".join(duplicates), flush=True)

    print("===== STAGE 2 / SCOPED CLEANUP =====", flush=True)
    rc = stop_scoped_navigation(nodes)
    if rc:
        return rc

    print("===== FINAL CENSUS =====", flush=True)
    try:
        final_nodes = census()
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        print(f"ERROR: 清理后复查失败：{exc}", file=sys.stderr)
        return 21

    remaining = sorted(set(final_nodes) & NAV_MARKERS)
    active_pid = active_managed_navigation_pid()
    if remaining and active_pid is None:
        print("ERROR: 以下 N-01 导航节点没有有效的受管 launch PID，拒绝启动：", file=sys.stderr)
        for name in remaining:
            print(f"  {name}", file=sys.stderr)
        return 22
    if remaining and active_pid is not None:
        print(f"保留正在运行的受管 N-01 导航实例 PID={active_pid}："
              + ", ".join(remaining), flush=True)

    bridge_state, bridge_detail = classify_scan_bridge(final_nodes, active_pid)
    if bridge_state == "orphan":
        print("ERROR: /n01_live_scan_bridge 在 ROS 图上，但无法证明归属："
              f"（{bridge_detail}）。", file=sys.stderr)
        print("       它是定位数据链（n01_localization_stack.launch.py）的输出节点；"
              "导航生命周期不管它，因此 stop 清不掉。", file=sys.stderr)
        print("       请先在有持久终端的窗口停掉定位数据链/desk，再重新启动 GUI；"
              "不要直接杀厂商 SLAM、Livox、IMU 或底盘进程。", file=sys.stderr)
        return 23
    if bridge_state == "upstream":
        print(f"保留定位数据链拥有的 /n01_live_scan_bridge（上游证据：{bridge_detail}）；"
              "它不属于导航生命周期，不清理。", flush=True)

    protected = sorted(set(final_nodes) & PROTECTED_MARKERS)
    print(f"PASS: N-01 导航预检通过；当前 ROS 图共 {len(final_nodes)} 个节点。", flush=True)
    if bridge_state == "upstream":
        print("受保护节点仍正常保留（不会被 GUI 清理）：/n01_live_scan_bridge, "
              + ", ".join(protected), flush=True)
    elif protected:
        print("受保护节点仍正常保留（不会被 GUI 清理）：", ", ".join(protected), flush=True)
    print("未能证明归属的其他 ROS 节点只报告，不执行 kill。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
