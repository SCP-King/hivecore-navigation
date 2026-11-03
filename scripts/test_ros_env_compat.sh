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

echo "ROS environment compatibility tests: PASS"
