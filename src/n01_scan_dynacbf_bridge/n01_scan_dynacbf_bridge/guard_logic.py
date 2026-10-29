"""Fail-closed validation for isolated test command output. Not a hardware E-stop."""
import math


def checked_command(armed, bridge_valid, move_allowed, values,
                    bridge_age, move_age, plan_age,
                    timeout=0.5, max_v=0.5, max_omega=0.3):
    if not armed:
        return 0.0, 0.0, 'DISARMED'
    if not bridge_valid or not move_allowed:
        return 0.0, 0.0, 'NOT_AUTHORIZED'
    if not all(math.isfinite(v) and 0.0 <= v <= timeout for v in (bridge_age, move_age, plan_age)):
        return 0.0, 0.0, 'STALE'
    if values is None or len(values) < 2:
        return 0.0, 0.0, 'NO_PLAN'
    try:
        v, w = float(values[0]), float(values[1])
    except (TypeError, ValueError):
        return 0.0, 0.0, 'INVALID_PLAN'
    if not math.isfinite(v) or not math.isfinite(w):
        return 0.0, 0.0, 'INVALID_PLAN'
    if not (0.0 <= v <= max_v and abs(w) <= max_omega):
        return 0.0, 0.0, 'OUT_OF_BOUNDS'
    return v, w, 'OK'
