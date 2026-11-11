#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPYCACHEPREFIX="${TMPDIR:-/tmp}/n01_gui_pycache_${UID}"
mkdir -p "$PYTHONPYCACHEPREFIX"
echo "===== 1. PYTHON SYNTAX ====="
python3 -m py_compile "$ROOT/slam_lite_gui.py" "$ROOT"/tools/*.py "$ROOT"/tests/*.py
echo "===== 2. SHELL SYNTAX ====="
bash -n "$ROOT/run_slam_lite_gui.sh" "$ROOT"/tools/*.sh
echo "===== 3. DESK GUARD TESTS ====="
python3 "$ROOT/tests/test_desk_guard.py"
echo "===== 4. NAVIGATION LIFECYCLE TESTS ====="
python3 "$ROOT/tests/test_navigation_lifecycle.py"
echo "===== 5. WORKER FAILURE-STATE TESTS ====="
python3 "$ROOT/tests/test_worker_failure_contract.py"

echo "===== 6. WORKER / CLEAN ENV CONTRACT ====="
python3 "$ROOT/tests/test_worker_contract.py"
echo "===== 7. STARTUP PREFLIGHT TESTS ====="
python3 "$ROOT/tests/test_startup_preflight.py"
echo "===== 8. SAFE CLEANUP CONTRACT ====="
if grep -RniE 'killall[[:space:]]+.*(ros|python)|pkill[[:space:]]+-f[[:space:]].*(ros2|localization_lite|mapping_lite)' \
    "$ROOT/slam_lite_gui.py" "$ROOT/tools"; then
  echo "ERROR: broad process-kill command found" >&2
  exit 1
fi
echo "PASS: no global ROS kill command found"
echo "===== ALL OFFLINE CHECKS PASSED ====="
