#!/usr/bin/env python3
"""Stop one N-01 live navigation process group and any narrowly matched orphans.

The live launch is started under setsid, so its PID is also its private process
group/session ID. Group signalling is the primary lifecycle mechanism; exact
executable matching is a fallback for legacy launches whose parent was killed.
"""
import argparse
import os
import signal
import sys
import time
from pathlib import Path

MANAGED_EXECUTABLES = {
    "static_map_node",
    "n01_live_scan_bridge",
    "n01_live_runtime_supervisor",
    "n01_live_safety_supervisor",
    "n01_live_dynamic_obstacle_tracker",
    "n01_hive2_trajectory_monitor",
    "n01_live_goal_gate",
    "scan_planner_node",
}
LAUNCH_SCRIPT = "live_scan.launch.py"
RVIZ_CONFIG = "n01_live_navigation.rviz"


def process_cmdline(pid):
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
        return [part.decode(errors="replace") for part in raw.split(bytes([0])) if part]
    except (OSError, ProcessLookupError):
        return []


def proc_stat(pid):
    try:
        raw = Path(f"/proc/{pid}/stat").read_text()
        # comm may contain spaces or parentheses; fields after the final ')'
        # start with state, ppid, pgrp, session.
        tail = raw[raw.rfind(")") + 2:].split()
        return {"state": tail[0], "pgrp": int(tail[2]), "session": int(tail[3])}
    except (OSError, ValueError, IndexError):
        return None


def is_managed_process(pid):
    argv = process_cmdline(pid)
    if not argv:
        return False, None
    basenames = {Path(arg).name for arg in argv}
    if "launch" in argv and "n01_live_navigation" in argv and LAUNCH_SCRIPT in argv:
        return True, "n01 live launch parent"
    matched = sorted(MANAGED_EXECUTABLES.intersection(basenames))
    if matched:
        return True, matched[0]
    if "rviz2" in basenames and any(RVIZ_CONFIG in arg for arg in argv):
        return True, "N-01 RViz"
    return False, None


def discover():
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        matched, role = is_managed_process(pid)
        if matched and pid != os.getpid():
            found.append((pid, role, process_cmdline(pid)))
    return sorted(found)


def group_members(pgid):
    members = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        stat = proc_stat(pid)
        if stat and stat["pgrp"] == pgid and stat["state"] != "Z":
            members.append(pid)
    return members


def signal_group(pgid, sig, label):
    members = group_members(pgid)
    if not members:
        return []
    try:
        os.killpg(pgid, sig)
        print(f"{label}: process group {pgid}, PIDs {members}")
    except ProcessLookupError:
        return []
    except PermissionError:
        print(f"ERROR: permission denied signalling process group {pgid}", file=sys.stderr)
        return members
    return members


def wait_group_gone(pgid, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        members = group_members(pgid)
        if not members:
            return []
        time.sleep(0.2)
    return group_members(pgid)


def wait_pids_gone(pids, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        alive = []
        for pid in pids:
            stat = proc_stat(pid)
            if stat and stat["state"] != "Z":
                alive.append(pid)
        if not alive:
            return []
        time.sleep(0.2)
    return [pid for pid in pids if (proc_stat(pid) or {}).get("state") != "Z"]


def stop_group(launch_pid):
    stat = proc_stat(launch_pid)
    if not stat:
        print(f"Recorded launch PID {launch_pid} is already absent.")
        return []
    pgid = stat["pgrp"]
    # setsid launch contract: the launch PID must own a private process group.
    if pgid != launch_pid:
        raise RuntimeError(
            f"PID {launch_pid} is not its own process-group leader (PGID={pgid}); "
            "refusing to signal an unrelated process group."
        )
    signal_group(pgid, signal.SIGINT, "SIGINT")
    remaining = wait_group_gone(pgid, 8.0)
    if remaining:
        signal_group(pgid, signal.SIGTERM, "SIGTERM")
        remaining = wait_group_gone(pgid, 3.0)
    if remaining:
        signal_group(pgid, signal.SIGKILL, "SIGKILL (isolated N-01 process group only)")
        remaining = wait_group_gone(pgid, 2.0)
    return remaining


def stop_legacy_orphans():
    targets = discover()
    if not targets:
        print("No legacy N-01 navigation orphan processes found.")
        return []
    print("Legacy N-01 processes outside the managed launch group:")
    for pid, role, argv in targets:
        print(f"  PID {pid}: {role}: {' '.join(argv)[:220]}")
    pids = [item[0] for item in targets]
    for sig, label, wait_s in (
        (signal.SIGINT, "SIGINT", 5.0),
        (signal.SIGTERM, "SIGTERM", 2.0),
        (signal.SIGKILL, "SIGKILL (exact N-01 executable match only)", 2.0),
    ):
        current = [item for item in discover() if item[0] in pids]
        for pid, role, _ in current:
            try:
                os.kill(pid, sig)
                print(f"{label}: PID {pid} ({role})")
            except ProcessLookupError:
                pass
            except PermissionError:
                print(f"ERROR: permission denied for PID {pid}", file=sys.stderr)
        pids = wait_pids_gone(pids, wait_s)
        if not pids:
            break
    return pids


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--launch-pid", type=int,
                        help="PID returned by setsid ros2 launch; its PGID must equal PID")
    args = parser.parse_args()

    print("===== N-01 navigation lifecycle cleanup =====")
    print("Protected: independently managed Lite SLAM, localization adapter, Livox, IMU and robot drivers.")
    remaining = []
    if args.launch_pid:
        remaining = stop_group(args.launch_pid)
        if remaining:
            print("ERROR: managed launch process group remains: " +
                  ", ".join(map(str, remaining)), file=sys.stderr)
            return 2

    # Compatibility path for launch parents killed before the process-group
    # lifecycle was installed. The independent adapter is deliberately excluded.
    orphans = stop_legacy_orphans()
    if orphans:
        print("ERROR: N-01 navigation processes remain: " +
              ", ".join(map(str, orphans)), file=sys.stderr)
        return 2

    print("PASS: N-01 navigation process group and legacy orphans are stopped.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(3)
