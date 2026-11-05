#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/lib/ros_env.sh"

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

cat > "$TMP_DIR/optional_unset.sh" <<'EOF'
# Deliberately reproduces the ROS/colcon setup pattern that fails under set -u.
if [[ -n "$AMENT_TRACE_SETUP_FILES" ]]; then
  :
fi
export N01_ROS_ENV_COMPAT_TEST=loaded
EOF

unset AMENT_TRACE_SETUP_FILES || true
set -u
n01_source_setup "$TMP_DIR/optional_unset.sh"
[[ "$-" == *u* ]]
[[ "$N01_ROS_ENV_COMPAT_TEST" == "loaded" ]]
echo "PASS: nounset restored after successful setup source"

cat > "$TMP_DIR/failing_setup.sh" <<'EOF'
return 23
EOF
if n01_source_setup "$TMP_DIR/failing_setup.sh"; then
  echo "FAIL: source failure was swallowed" >&2
  exit 1
else
  source_rc=$?
fi
[[ "$source_rc" -eq 23 ]]
[[ "$-" == *u* ]]
echo "PASS: source exit status preserved and nounset restored"

set +u
unset N01_ROS_ENV_COMPAT_TEST
n01_source_setup "$TMP_DIR/optional_unset.sh"
[[ "$-" != *u* ]]
[[ "$N01_ROS_ENV_COMPAT_TEST" == "loaded" ]]
echo "PASS: caller without nounset remains without nounset"

mkdir -p "$TMP_DIR/ws/install/scan_planner" "$TMP_DIR/vendor/lib"
export AMENT_PREFIX_PATH="$TMP_DIR/ws/install/scan_planner:/opt/ros/humble:$TMP_DIR/vendor"
export COLCON_PREFIX_PATH="$TMP_DIR/ws/install:/opt/ros/humble:$TMP_DIR/vendor"
export CMAKE_PREFIX_PATH="$TMP_DIR/ws/install/scan_planner:$TMP_DIR/vendor"
export LD_LIBRARY_PATH="$TMP_DIR/ws/install/scan_planner/lib:$TMP_DIR/vendor/lib"
export PYTHONPATH="$TMP_DIR/ws/install/n01_live_navigation/lib/python3.10/site-packages:$TMP_DIR/vendor/python"
n01_strip_workspace_prefixes "$TMP_DIR/ws"
case ":${AMENT_PREFIX_PATH:-}:" in *":$TMP_DIR/ws/install"*) echo "FAIL: AMENT self-prefix retained" >&2; exit 1;; esac
case ":${COLCON_PREFIX_PATH:-}:" in *":$TMP_DIR/ws/install"*) echo "FAIL: COLCON self-prefix retained" >&2; exit 1;; esac
case ":${CMAKE_PREFIX_PATH:-}:" in *":$TMP_DIR/ws/install"*) echo "FAIL: CMAKE self-prefix retained" >&2; exit 1;; esac
case ":${LD_LIBRARY_PATH:-}:" in *":$TMP_DIR/ws/install"*) echo "FAIL: LD_LIBRARY self-prefix retained" >&2; exit 1;; esac
case ":${PYTHONPATH:-}:" in *":$TMP_DIR/ws/install"*) echo "FAIL: PYTHON self-prefix retained" >&2; exit 1;; esac
[[ "$AMENT_PREFIX_PATH" == "/opt/ros/humble:$TMP_DIR/vendor" ]]
[[ "$COLCON_PREFIX_PATH" == "/opt/ros/humble:$TMP_DIR/vendor" ]]
[[ "$CMAKE_PREFIX_PATH" == "$TMP_DIR/vendor" ]]
[[ "$LD_LIBRARY_PATH" == "$TMP_DIR/vendor/lib" ]]
[[ "$PYTHONPATH" == "$TMP_DIR/vendor/python" ]]
echo "PASS: only this workspace's install prefixes are removed; vendor/ROS paths survive"

echo "ROS environment compatibility tests: PASS"
