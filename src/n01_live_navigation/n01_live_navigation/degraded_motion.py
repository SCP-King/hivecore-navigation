"""Short continuity window backed by fresh measured/predicted control inputs."""
import math
TRANSIENT_BLOCKERS=frozenset({
    'live_localization_or_scan_bridge_not_ready','global_robot_pose_missing_or_stale',
    'scan_motion_compensation_not_ready','body_pose_missing_or_stale'})


def continuity_allowed(health, gap_s, path_fresh, already_moving):
    if (not isinstance(health,dict) or not path_fresh or not already_moving
            or not math.isfinite(gap_s) or not 0<=gap_s<=.4):return False
    blockers=health.get('blockers')
    if not isinstance(blockers,list) or not blockers or not set(blockers)<=TRANSIENT_BLOCKERS:return False
    b=health.get('bridge',{})
    if not isinstance(b,dict):return False
    p=b.get('pose_prediction',{})
    if not isinstance(p,dict) or not p or p.get('fault'):return False
    for key in ('odom_receive_fresh','body_pose_live_fresh','sensor_extrinsic_ready','cloud_output_fresh'):
        if b.get(key) is not True:return False
    age=b.get('cloud_output_age_s')
    return isinstance(age,(float,int)) and math.isfinite(age) and 0<=age<=.35


def degraded_speed_limit(reviewed_speed, spin_threshold):
    # A limit cannot increase the reviewed speed. Below native translation
    # threshold, scaling further would silently turn slowdown into a stop.
    return min(reviewed_speed,max(spin_threshold+.002,reviewed_speed*.5))


def reviewed_path_continuity(health, path_fresh):
    """Current evidence replaces aggregate READY and time-since-last-green.

    No cached green state is used. The caller still checks pose, command
    ownership, estop, path/goal binding, collision review and native wheel limits.
    """
    if not isinstance(health,dict) or not path_fresh:return False
    blockers=health.get('blockers')
    if not isinstance(blockers,list) or not set(blockers)<=TRANSIENT_BLOCKERS|{'global_robot_pose_anchor_outside_scan_time_budget'}:
        return False
    b=health.get('bridge',{})
    if not isinstance(b,dict):return False
    p=b.get('pose_prediction',{})
    if not isinstance(p,dict) or not p or p.get('fault'):return False
    for key in ('odom_receive_fresh','body_pose_live_fresh','sensor_extrinsic_ready','cloud_output_fresh'):
        if b.get(key) is not True:return False
    age=b.get('cloud_output_age_s')
    return isinstance(age,(float,int)) and math.isfinite(age) and 0<=age<=.35
