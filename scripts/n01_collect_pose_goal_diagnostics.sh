#!/usr/bin/env bash
# Backward-compatible entry point.
# The comprehensive planning-readiness audit supersedes this focused diagnostic.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec bash "$SCRIPT_DIR/n01_collect_planning_readiness_audit.sh" "$@"
