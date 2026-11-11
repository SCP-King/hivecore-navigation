#!/usr/bin/env python3
"""Navigation lifecycle wrapper tests; no ROS graph or robot required."""
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "tools" / "navigation_lifecycle.py"


class NavigationLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self.tmp.name)
        scripts = self.repo / "scripts"
        scripts.mkdir()
        self.calls = self.repo / "calls.txt"
        entry = scripts / "n01_navigation.sh"
        entry.write_text(
            "#!/usr/bin/env bash\n"
            f"echo \"$1\" >> {self.calls}\n"
            "exit 0\n"
        )
        entry.chmod(0o755)

    def tearDown(self):
        self.tmp.cleanup()

    def run_helper(self, action):
        return subprocess.run([sys.executable, str(HELPER), action, "--repo", str(self.repo)],
                              text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              timeout=10, check=False)

    def test_start_runs_stop_then_status_then_start(self):
        cp = self.run_helper("start")
        self.assertEqual(cp.returncode, 0, cp.stdout)
        self.assertEqual(self.calls.read_text().splitlines(), ["stop", "status", "start"])

    def test_stop_only_calls_managed_stop(self):
        cp = self.run_helper("stop")
        self.assertEqual(cp.returncode, 0, cp.stdout)
        self.assertEqual(self.calls.read_text().splitlines(), ["stop"])

    def test_missing_entry_fails_closed(self):
        (self.repo / "scripts" / "n01_navigation.sh").unlink()
        cp = self.run_helper("start")
        self.assertNotEqual(cp.returncode, 0)
        self.assertIn("not found", cp.stdout)


if __name__ == "__main__":
    unittest.main(verbosity=2)
