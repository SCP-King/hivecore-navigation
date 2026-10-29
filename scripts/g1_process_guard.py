#!/usr/bin/env python3
"""Find and signal ONLY processes spawned by one N-01 offline test run.

Ownership uses an exact unique environment token inherited by spawned children,
not process names, system-wide pkill, or ROS domain alone. Linux /proc required.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys

MARKER = "N01_G1_RUN_TOKEN"


def process_info(pid, token):
    # Do not signal the guard itself OR its invoking test runner shell.
    # The marker is exported by the parent runner; killing it mid-cleanup
    # would abort the archive, reaping and final ROS-graph verification.
    if pid in (os.getpid(), os.getppid()) or not str(pid).isdigit():
        return None
    root = Path("/proc") / str(pid)
    try:
        if root.stat().st_uid != os.geteuid():
            return None
        env = (root / "environ").read_bytes().split(b"\0")
        target = (MARKER + "=" + token).encode("utf-8")
        if target not in env:
            return None
        raw = (root / "stat").read_text(encoding="utf-8")
        tail = raw.rsplit(") ", 1)[1].split()
        state = tail[0]
        # /proc/PID/stat field 22 is starttime (tail index 19).
        starttime = tail[19]
        argv = (root / "cmdline").read_bytes().replace(b"\0", b" ").decode(
            "utf-8", errors="replace").strip()[:400]
        return {
            "pid": pid, "ppid": int(tail[1]), "state": state,
            "pgid": int(tail[2]), "sid": int(tail[3]),
            "starttime": starttime, "cmdline": argv
        }
    except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError, ValueError, OSError):
        return None


def owned_processes(token):
    results = []
    with os.scandir("/proc") as entries:
        for item in entries:
            if item.name.isdigit():
                record = process_info(int(item.name), token)
                if record is not None:
                    results.append(record)
    return sorted(results, key=lambda record: record["pid"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["list", "signal", "verify"])
    parser.add_argument("--signal", dest="signame", choices=["INT", "TERM", "KILL"])
    args = parser.parse_args()
    token = os.environ.get(MARKER, "")
    if not token.startswith("g1_scan_") or len(token) < 20:
        print("Refusing to inspect/kill: invalid per-run marker", file=sys.stderr)
        return 2
    if not Path("/proc/self/environ").exists():
        print("Linux /proc is required", file=sys.stderr)
        return 2
    found = owned_processes(token)
    if args.action == "verify":
        print(json.dumps({"marker": token, "remaining": found}, indent=2))
        return 1 if found else 0
    if args.action == "list":
        print(json.dumps({"marker": token, "processes": found}, indent=2))
        return 0
    if not args.signame:
        print("--signal required", file=sys.stderr)
        return 2
    sig = getattr(signal, "SIG" + args.signame)
    reported = []
    for record in found:
        # Recheck ownership and starttime immediately before signalling a PID.
        fresh = process_info(record["pid"], token)
        if fresh is None or fresh["starttime"] != record["starttime"]:
            continue
        try:
            os.kill(record["pid"], sig)
            reported.append({"pid": record["pid"], "signal": args.signame})
        except ProcessLookupError:
            pass
        except PermissionError as exc:
            reported.append({"pid": record["pid"], "error": str(exc)})
    print(json.dumps({"marker": token, "signals": reported}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
