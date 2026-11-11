"""Pure path-source selection for the slow-motion executor.

No ROS imports: this module is plain functions over plain data so it can be
unit-tested without rclpy.  ``select_source`` decides which path the executor
follows on a given control cycle and returns a fully-validated candidate or
None.  All safety gates (estop / dec-stop / pose / readiness / CBF) live in the
executor, not here; this module only answers "which path and validated prefix".
"""
from __future__ import annotations

import math
from typing import Optional


def select_source(*, path_source: str, now: float,
                  path, path_rx: float, path_timeout: float,
                  path_status, path_status_rx: float,
                  path_valid_at: float, grace_s: float,
                  global_path, global_path_rx: float, global_timeout_s: float,
                  pose) -> Optional[tuple]:
    """Choose the followable path for one cycle.

    Returns (source, poses, prefix, path_stamp_ns) or None, where:
      source  "unified" | "unified_grace" | "global"
      poses   the full Path.poses list chosen (not yet prefix-sliced)
      prefix  number of leading poses allowed as the validated execution prefix
      path_stamp_ns  the header stamp of the chosen path

    ``path_source`` semantics:
      "unified"  only the live validated local prefix (strict; stops on gaps)
      "global"   only the planner's /planning/global_path while fresh
      "auto"     unified first; keep the last validated prefix during a short
                 replanning gap (grace_s); otherwise fall back to global while
                 fresh.  Exhausted validated prefixes skip to global so the
                 robot is not held at a replanning boundary.
    """
    now = float(now)
    path_timeout = float(path_timeout)
    global_timeout_s = float(global_timeout_s)

    def _prefix_ok(status) -> Optional[int]:
        try:
            prefix = int(status.get('validated_prefix_points', 0))
        except (TypeError, ValueError):
            return None
        if path is None or not 2 <= prefix <= len(path.poses):
            return None
        return prefix

    def _exhausted(poses, prefix) -> bool:
        if pose is None or prefix is None or prefix >= len(poses):
            return False
        pe = poses[prefix - 1].pose.position
        return math.hypot(pe.x - pose[0], pe.y - pose[1]) < 0.20

    def _unified(freshness_required: bool,
                 allow_exhausted: bool) -> Optional[tuple]:
        if (path is None or path_status is None or len(path.poses) < 2
                or now - path_rx > path_timeout):
            return None
        if freshness_required and now - path_status_rx > 0.6:
            return None
        if path_status.get('valid') is not True:
            return None
        stamp = path.header.stamp.sec * 10 ** 9 + path.header.stamp.nanosec
        if path_status.get('path_stamp_ns') != stamp:
            return None
        prefix = _prefix_ok(path_status)
        if prefix is None:
            return None
        if not allow_exhausted and _exhausted(path.poses, prefix):
            return None
        return ("unified", path.poses, prefix, stamp)

    def _global() -> Optional[tuple]:
        if (global_path is None or now - global_path_rx > global_timeout_s
                or global_path.header.frame_id.lstrip('/') != 'map'
                or len(global_path.poses) < 2):
            return None
        gstamp = (global_path.header.stamp.sec * 10 ** 9
                  + global_path.header.stamp.nanosec)
        return ("global", global_path.poses, len(global_path.poses), gstamp)

    if path_source == "unified":
        return _unified(freshness_required=True, allow_exhausted=True)
    if path_source == "global":
        return _global()

    # auto
    u = _unified(freshness_required=True, allow_exhausted=False)
    if u is not None:
        return u
    # grace: keep the last validated prefix across a short replan gap
    if (path_valid_at > 0.0 and now - path_valid_at <= grace_s
            and path is not None and path_status is not None
            and now - path_rx <= path_timeout * 2.0
            and path_status.get('valid') is True):
        stamp = path.header.stamp.sec * 10 ** 9 + path.header.stamp.nanosec
        if path_status.get('path_stamp_ns') == stamp:
            prefix = _prefix_ok(path_status)
            if prefix is not None and not _exhausted(path.poses, prefix):
                return ("unified_grace", path.poses, prefix, stamp)
    return _global()