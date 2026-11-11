#!/usr/bin/env python3
"""Stop only Hive2 chassis processes; never touch SLAM/navigation/sensors."""
import os
import signal
import time
from pathlib import Path

NAMES = {
    "steer_drive_node",
    "cmd_wheel_preprocessor_node",
    "person_follow_controller_node",
    "rt_ros2_control_node",
    "control_mode_manager.py",
    "spawn_controller.py",
    "robot_state_publisher",
    "hive2_driver.launch.py",
    "hive2_chassis.launch.py",
}


def cmdline(pid):
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return []
    return [p.decode(errors="replace") for p in raw.split(b"\0") if p]


def matched(pid):
    argv = cmdline(pid)
    if not argv:
        return False
    bases = {Path(a).name for a in argv}
    if bases & NAMES:
        return True
    # ros2 launch hive2_driver ... / ros2 run ... is matched by its args.
    return any("hive2_driver.launch.py" in a or "hive2_chassis.launch.py" in a
               for a in argv)


def alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def main():
    me = os.getpid()
    targets = [int(p.name) for p in Path("/proc").iterdir()
               if p.name.isdigit() and int(p.name) != me and matched(int(p.name))]
    if not targets:
        print("No Hive2 chassis processes found.")
        return 0
    print("Stopping Hive2 chassis PIDs:", targets)
    for pid in targets:
        try:
            os.kill(pid, signal.SIGINT)
        except OSError:
            pass
    for _ in range(30):
        targets = [p for p in targets if alive(p)]
        if not targets:
            break
        time.sleep(0.2)
    for pid in targets:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    time.sleep(1.0)
    targets = [p for p in targets if alive(p)]
    for pid in targets:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    print("PASS: Hive2 chassis process cleanup complete." if not targets else
          f"WARN: still alive {targets}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
