#!/usr/bin/env python3
"""Pure-Python lifecycle tests for desk_guard; no ROS imports required."""
import importlib.util
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("desk_guard", ROOT / "tools" / "desk_guard.py")
guard = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(guard)


def start_ticks(pid: int) -> int:
    raw = Path(f"/proc/{pid}/stat").read_text()
    return int(raw.rsplit(") ", 1)[1].split()[19])


class DeskGuardTests(unittest.TestCase):
    def test_process_identity_matches_live_pid(self):
        identity = guard.process_identity(os.getpid())
        self.assertIsNotNone(identity)
        self.assertNotEqual(identity[0], "Z")
        self.assertEqual(identity[1], start_ticks(os.getpid()))

    def test_group_members_excludes_zombies_and_self(self):
        members = guard.group_members(os.getpgrp())
        self.assertNotIn(os.getpid(), members)
        self.assertTrue(all(pid > 0 for pid in members))

    def test_guard_stops_child_when_original_gui_pid_disappears(self):
        gui = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
        guard_proc = None
        child_pid_file = Path(tempfile.mkstemp(prefix="n01-guard-child-")[1])
        child_pid_file.unlink(missing_ok=True)
        try:
            gui_ticks = start_ticks(gui.pid)
            child_code = (
                "import os,time,pathlib; "
                f"pathlib.Path({str(child_pid_file)!r}).write_text(str(os.getpid())); "
                "time.sleep(30)"
            )
            guard_proc = subprocess.Popen(
                [sys.executable, str(ROOT / "tools" / "desk_guard.py"),
                 str(gui.pid), str(gui_ticks), sys.executable, "-c", child_code],
                start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True,
            )
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and not child_pid_file.exists():
                time.sleep(0.05)
            self.assertTrue(child_pid_file.exists(), "guard child did not start")
            child_pid = int(child_pid_file.read_text())
            gui.terminate()
            gui.wait(timeout=3)
            guard_proc.wait(timeout=12)
            if guard_proc.stdout:
                guard_proc.stdout.close()
            if guard_proc.stderr:
                guard_proc.stderr.close()
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline and guard._pid_exists_active(child_pid):
                time.sleep(0.05)
            self.assertFalse(guard._pid_exists_active(child_pid), "guard left its child alive")
        finally:
            if gui.poll() is None:
                gui.kill(); gui.wait()
            if guard_proc and guard_proc.poll() is None:
                guard_proc.kill(); guard_proc.wait()
            if guard_proc:
                if guard_proc.stdout and not guard_proc.stdout.closed:
                    guard_proc.stdout.close()
                if guard_proc.stderr and not guard_proc.stderr.closed:
                    guard_proc.stderr.close()
            child_pid_file.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
