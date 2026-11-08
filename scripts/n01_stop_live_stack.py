#!/usr/bin/env python3
"""Stop only N-01 live navigation launch/node processes; preserve SLAM and sensors.

This is intentionally process-scoped rather than ROS-domain-scoped. It never
matches or signals the independent n01_localization_adapter, vendor SLAM,
Livox, IMU, robot drivers, or arbitrary ROS nodes.
"""
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
        return [part.decode(errors="replace") for part in raw.split(b"\0") if part]
    except (OSError, ProcessLookupError):
        return []


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


def wait_gone(pids, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        alive = []
        for pid in pids:
            try:
                os.kill(pid, 0)
                # Treat zombies as exited: they no longer own ROS resources.
                stat = Path(f"/proc/{pid}/stat").read_text().split()
                if len(stat) > 2 and stat[2] == "Z":
                    continue
                alive.append(pid)
            except ProcessLookupError:
                pass
            except (PermissionError, OSError):
                alive.append(pid)
        if not alive:
            return []
        time.sleep(0.2)
    return alive


def signal_stage(processes, sig, label):
    alive = []
    for pid, role, _ in processes:
        try:
            os.kill(pid, sig)
            print(f"{label}: PID {pid} ({role})")
            alive.append(pid)
        except ProcessLookupError:
            pass
        except PermissionError:
            print(f"ERROR: no permission to signal PID {pid} ({role})", file=sys.stderr)
            alive.append(pid)
    return alive


def main():
    print("===== N-01 scoped process cleanup =====")
    print("Protected: Lite SLAM, localization adapter, Livox, IMU and robot drivers.")
    targets = discover()
    if not targets:
        print("No matching N-01 live navigation processes found.")
        return 0

    print("Matched processes:")
    for pid, role, argv in targets:
        print(f"  PID {pid}: {role}: {' '.join(argv)[:260]}")

    pids = [item[0] for item in targets]
    signal_stage(targets, signal.SIGINT, "SIGINT")
    remaining = wait_gone(pids, 8.0)
    if remaining:
        current = [item for item in discover() if item[0] in remaining]
        signal_stage(current, signal.SIGTERM, "SIGTERM")
        remaining = wait_gone(remaining, 3.0)
    if remaining:
        current = [item for item in discover() if item[0] in remaining]
        if current:
            signal_stage(current, signal.SIGKILL, "SIGKILL (N-01 managed processes only)")
        remaining = wait_gone(remaining, 2.0)

    if remaining:
        print("ERROR: these N-01 processes could not be stopped: " +
              ", ".join(map(str, remaining)), file=sys.stderr)
        return 2
    print("PASS: all matched N-01 live navigation processes exited.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
