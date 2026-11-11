#!/usr/bin/env python3
"""Safe wrapper for the N-01 navigation repository lifecycle entrypoint.

This tool deliberately never scans and kills arbitrary ROS processes. It delegates
ownership-aware start/stop/status to hivecore-navigation/scripts/n01_navigation.sh.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


def main() -> int:
    default_repo = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop", "status"))
    parser.add_argument("--repo", default=os.environ.get(
        "N01_NAV_REPO", str(default_repo)))
    args = parser.parse_args()
    repo = Path(args.repo).expanduser().resolve()
    entry = repo / "scripts" / "n01_navigation.sh"
    if not entry.is_file():
        print(f"ERROR: navigation lifecycle entry not found: {entry}", file=sys.stderr)
        return 2

    def invoke(action: str, timeout: int) -> int:
        print(f"===== N01 NAVIGATION {action.upper()} =====", flush=True)
        cp = subprocess.run(["bash", str(entry), action], cwd=repo, timeout=timeout,
                            env=os.environ.copy(), check=False)
        return cp.returncode

    try:
        if args.action == "start":
            # Stop only the N-01 stack owned by the navigation lifecycle manager.
            rc = invoke("stop", 45)
            if rc:
                print(f"ERROR: pre-start managed cleanup failed ({rc}); refusing duplicate start.")
                return rc
            rc = invoke("status", 20)
            if rc:
                print(f"ERROR: post-cleanup status check failed ({rc}); refusing start.")
                return rc
            return invoke("start", 90)
        return invoke(args.action, 45 if args.action == "stop" else 20)
    except subprocess.TimeoutExpired as exc:
        print(f"ERROR: lifecycle command timed out: {exc}", file=sys.stderr)
        # subprocess.run 只杀直接子进程；bash n01_navigation.sh 可能仍在启动
        # ros2 launch（其子进程已在独立会话）。用仓库自己的 stop 兜底清理，
        # 避免超时后留下半启动的导航栈。
        try:
            subprocess.run(["bash", str(entry), "stop"], cwd=repo,
                           env=os.environ.copy(), timeout=100, check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            print("INFO: best-effort stop cleanup finished after timeout.", file=sys.stderr)
        except subprocess.TimeoutExpired:
            print("ERROR: cleanup stop also timed out; run status manually.",
                  file=sys.stderr)
        return 124


if __name__ == "__main__":
    raise SystemExit(main())
