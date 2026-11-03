#!/usr/bin/env bash
# One-command, read-only live-stack observer. Starts only the Python audit
# subscriber; it never publishes goals, calls services/actions, writes params,
# restarts nodes, or sends actuator commands.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
RUNTIME_ROOT="${N01_RUNTIME_ROOT:-$HOME/lyy/n01_runtime}"
REPORT_DIR="$RUNTIME_ROOT/runs/live_stack_preflight"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="$REPORT_DIR/n01_live_stack_preflight_$STAMP.json"
DURATION="${1:-10}"

mkdir -p "$REPORT_DIR"
# shellcheck disable=SC1091
source "$ROOT/scripts/lib/ros_env.sh"

write_error_report() {
  local status="$1"
  local message="$2"
  STATUS="$status" MESSAGE="$message" REPORT="$REPORT" python3 - <<'PY'
import json
import os
with open(os.environ["REPORT"], "w", encoding="utf-8") as stream:
    json.dump({
        "schema": "n01.live_stack_preflight_wrapper_error.v1",
        "status": os.environ["STATUS"],
        "error": os.environ["MESSAGE"],
        "read_only": True,
        "actuator_output": "DISABLED",
        "execution_authorized": False,
    }, stream, indent=2, ensure_ascii=False, sort_keys=True)
    stream.write("\n")
PY
}

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  write_error_report "ROS_HUMBLE_SETUP_MISSING" "/opt/ros/humble/setup.bash is not readable"
  echo "REPORT=$REPORT"
  cat "$REPORT"
  exit 2
fi
if ! n01_source_setup /opt/ros/humble/setup.bash; then
  write_error_report "ROS_HUMBLE_SETUP_FAILED" "Failed to source /opt/ros/humble/setup.bash"
  echo "REPORT=$REPORT"
  cat "$REPORT"
  exit 2
fi
if [[ ! -r "$ROOT/install/setup.bash" ]]; then
  write_error_report "WORKSPACE_SETUP_MISSING" "$ROOT/install/setup.bash is not readable"
  echo "REPORT=$REPORT"
  cat "$REPORT"
  exit 2
fi
if ! n01_source_setup "$ROOT/install/setup.bash"; then
  write_error_report "WORKSPACE_SETUP_FAILED" "Failed to source workspace install/setup.bash"
  echo "REPORT=$REPORT"
  cat "$REPORT"
  exit 2
fi

# ROS 2 Humble on the target Ubuntu environment uses the system Python.
ROS_PYTHON="${N01_ROS_PYTHON:-/usr/bin/python3}"
if [[ ! -x "$ROS_PYTHON" ]]; then
  ROS_PYTHON="$(command -v python3 2>/dev/null || true)"
fi
if [[ -z "$ROS_PYTHON" || ! -x "$ROS_PYTHON" ]]; then
  write_error_report "ROS_PYTHON_NOT_FOUND" "No executable Python interpreter was found"
  echo "REPORT=$REPORT"
  cat "$REPORT"
  exit 2
fi

echo "N-01 read-only live-stack preflight"
echo "workspace: $ROOT"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-unset}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default}"
echo "ROS Python: $ROS_PYTHON"
echo "duration: $DURATION s"
echo "REPORT=$REPORT"
echo "READ_ONLY: subscriber only; no goals, services/actions, parameter writes, restarts or actuator commands"

if "$ROS_PYTHON" "$ROOT/scripts/n01_live_stack_preflight.py" \
    --duration "$DURATION" > "$REPORT" 2>&1; then
  rc=0
else
  rc=$?
fi

echo
echo "===== structured report ====="
cat "$REPORT"
echo "PREFLIGHT_EXIT_CODE=$rc"
exit "$rc"
