"""Linux regression: G1 process cleanup must never kill an unrelated process."""
import os
from pathlib import Path
import subprocess
import sys
import time
import unittest

from g1_process_guard import owned_processes


class ProcessGuardTest(unittest.TestCase):
    def test_run_marker_scopes_process_ownership(self):
        token = "g1_scan_test_%s_%s" % (os.getpid(), time.time_ns())
        marked_env = dict(os.environ, N01_G1_RUN_TOKEN=token)
        # Keep the outer runner token, if present, so interruption cleanup
        # can still find this unrelated-to-the-test-token mock process.
        unmarked_env = dict(os.environ)
        self.assertNotEqual(unmarked_env.get("N01_G1_RUN_TOKEN"), token)
        marked = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(20)"],
            env=marked_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        other = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(20)"],
            env=unmarked_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            for _ in range(30):
                if any(p["pid"] == marked.pid for p in owned_processes(token)):
                    break
                time.sleep(.05)
            owned = owned_processes(token)
            self.assertIn(marked.pid, [p["pid"] for p in owned])
            self.assertNotIn(other.pid, [p["pid"] for p in owned])
            self.assertTrue(all(p["cmdline"] for p in owned))
            self.assertIsNone(marked.poll())
            self.assertIsNone(other.poll())
            subprocess.run(
                [sys.executable, str(Path(__file__).with_name("g1_process_guard.py")),
                 "signal", "--signal", "TERM"],
                env=marked_env, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, check=True, timeout=5)
            marked.wait(timeout=3)
            self.assertIsNotNone(marked.returncode)
            self.assertIsNone(other.poll())
        finally:
            for child in (marked, other):
                if child.poll() is None:
                    child.terminate()
                try:
                    child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=2)


if __name__ == "__main__":
    unittest.main()
