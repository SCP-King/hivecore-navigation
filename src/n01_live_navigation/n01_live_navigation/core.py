"""Pure helpers for the read-only live Hive2 -> SCAN adapter."""
import math


def localization_is_tracking(msg):
    """Check the real Hivecore LocalizationStatus contract, not node-name heuristics."""
    required = (
        "config_context_map_loaded",
        "control_signal_outer_role",
        "control_signal_session_mode",
        "registration_output_pose_published",
        "registration_output_tf_published",
        "registration_output_fitness_evaluation_ok",
        "registration_output_tracking_compose_ok",
    )
    if any(not hasattr(msg, name) for name in required):
        return False, "localization_status_contract_mismatch"
    if not bool(msg.config_context_map_loaded):
        return False, "localization_map_not_loaded"
    if str(msg.control_signal_outer_role).lower() != "localization":
        return False, "localization_outer_role_not_localization"
    if str(msg.control_signal_session_mode).lower() != "tracking":
        return False, "localization_not_tracking"
    if not bool(msg.registration_output_pose_published):
        return False, "localization_pose_not_published"
    if not bool(msg.registration_output_tf_published):
        return False, "localization_tf_not_published"
    if not bool(msg.registration_output_fitness_evaluation_ok):
        return False, "localization_fitness_not_accepted"
    if not bool(msg.registration_output_tracking_compose_ok):
        return False, "localization_tracking_compose_not_ok"
    # IMU sync is reported independently. It is deliberately not a hard gate:
    # the vendor currently publishes tracking poses with this flag false.
    return True, "tracking_ready"


def rotate_translate(point, translation, quaternion):
    """Apply a normalized xyzw rigid transform to one XYZ point."""
    x, y, z = (float(v) for v in point)
    tx, ty, tz = (float(v) for v in translation)
    qx, qy, qz, qw = (float(v) for v in quaternion)
    qn = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(qn) or qn < 1e-9:
        raise ValueError("invalid_transform_quaternion")
    qx, qy, qz, qw = qx/qn, qy/qn, qz/qn, qw/qn
    # q * p * conjugate(q), expanded to avoid temporary quaternion objects.
    ix = qw*x + qy*z - qz*y
    iy = qw*y + qz*x - qx*z
    iz = qw*z + qx*y - qy*x
    iw = -qx*x - qy*y - qz*z
    rx = ix*qw + iw*(-qx) + iy*(-qz) - iz*(-qy)
    ry = iy*qw + iw*(-qy) + iz*(-qx) - ix*(-qz)
    rz = iz*qw + iw*(-qz) + ix*(-qy) - iy*(-qx)
    out = (rx+tx, ry+ty, rz+tz)
    if not all(math.isfinite(v) for v in out):
        raise ValueError("nonfinite_transformed_point")
    return out


def estimate_velocity(previous, current, max_speed=3.0):
    """Finite-difference map-frame velocity; return zero and invalid on bad dt."""
    if previous is None:
        return (0.0, 0.0, 0.0), False
    p_xyz, p_stamp = previous
    c_xyz, c_stamp = current
    dt = float(c_stamp) - float(p_stamp)
    if not math.isfinite(dt) or dt < 0.02 or dt > 0.5:
        return (0.0, 0.0, 0.0), False
    vel = tuple((float(c_xyz[i])-float(p_xyz[i]))/dt for i in range(3))
    speed = math.sqrt(sum(v*v for v in vel))
    if not all(math.isfinite(v) for v in vel) or speed > max_speed:
        return (0.0, 0.0, 0.0), False
    return vel, True


def goal_is_valid(goal, *, live_ready, static_ready):
    """Pure fail-closed goal contract; goal is any object with ROS-like fields."""
    if not live_ready:
        return False, "live_localization_or_cloud_not_ready"
    if not static_ready:
        return False, "native_scan_static_pcd_not_ready"
    if str(goal.header.frame_id).lstrip("/") != "map":
        return False, "goal_frame_must_be_map"
    p = goal.pose.position
    if not all(math.isfinite(float(v)) for v in (p.x, p.y, p.z)):
        return False, "goal_nonfinite"
    if float(p.z) < -0.1:
        return False, "goal_z_below_scan_ground_guard"
    return True, "accepted"
