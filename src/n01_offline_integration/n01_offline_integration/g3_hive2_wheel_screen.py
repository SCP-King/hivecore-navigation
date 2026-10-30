"""G3.3: source-audited Hive2 motion-limit NECESSARY-condition screen.

Only XY positions and the assumed discrete time grid are available. This
cannot reconstruct the robot's yaw, current steering angles, wheel speeds,
braking, terrain contact or footprint collisions; it NEVER certifies 4WS
feasibility. It has NO ROS, control SDK or hardware imports.
"""
import math

SCHEMA = "n01.offline.g33_hive2_xy_motion_limit_screen.v1"
SOURCE_SHA256 = "3f19abe0a65e02b331a0d51c18ec0503278a4d9a6b942332b47c4f7d434584f0"
# Audited both exact-copy hive2_driver/config/robot_motion.yaml files.
CHASSIS_MAX_LINEAR_SPEED_MPS = 2.0
CHASSIS_MAX_LINEAR_ACCEL_MPS2 = 1.0
CHASSIS_NOMINAL_LINEAR_ACCEL_MPS2 = .6
# N-01 maximum speed requirement and repository SCAN planning settings.
NAV_REQUIREMENT_MAX_SPEED_MPS = 1.0
SCAN_MAX_SPEED_MPS = .75
SCAN_MAX_ACCEL_MPS2 = .5
# G3 uses zero initial velocity in its solver's finite-control candidate model.
# Historical Bag pose contains no trustworthy measured twist.
CANDIDATE_INITIAL_VELOCITY_ASSUMPTION = "G3_SOLVER_ZERO_NOT_MEASURED"
UNVERIFIED = (
    "four_wheel_steering_angle_allocation",
    "wheel_speed_and_steering_rate",
    "robot_yaw_and_angular_kinematics",
    "measured_initial_velocity_and_braking",
    "fixed_pitch_joint_feedback",
    "whole_body_collision_and_ground_support",
)


def _base(status):
    return {
        "schema": SCHEMA,
        "status": status,
        "scope": "DISCRETE_XY_TRANSLATIONAL_NECESSARY_CONDITIONS_ONLY",
        "parameter_reference_sha256": SOURCE_SHA256,
        "chassis_speed_limit_mps": CHASSIS_MAX_LINEAR_SPEED_MPS,
        "chassis_max_acceleration_mps2": CHASSIS_MAX_LINEAR_ACCEL_MPS2,
        "chassis_nominal_acceleration_mps2": CHASSIS_NOMINAL_LINEAR_ACCEL_MPS2,
        "navigation_speed_requirement_mps": NAV_REQUIREMENT_MAX_SPEED_MPS,
        "scan_speed_limit_mps": SCAN_MAX_SPEED_MPS,
        "scan_acceleration_limit_mps2": SCAN_MAX_ACCEL_MPS2,
        "g3_initial_velocity_assumption": CANDIDATE_INITIAL_VELOCITY_ASSUMPTION,
        "actual_initial_velocity_verified": False,
        "wheel_steering_feasibility_verified": False,
        "robot_model_execution_feasible": False,
        "unverified_requirements": list(UNVERIFIED),
        "physical_safety_authorization": "NEVER",
        "actuator_output": "DISABLED",
        "speed_max_mps": None,
        "accelerations_max_mps2": None,
        "chassis_config_limits_exceeded": None,
        "scan_planning_limits_exceeded": None,
        "navigation_requirement_speed_exceeded": None,
        "samples": 0,
        "assumed_dt_s": None,
    }


def screen_xy_candidate(candidate_xy, dt_s):
    """Check discrete, modeled speeds and XY acceleration norms.

    The first acceleration uses the G3 optimizer's ASSUMED zero initial
    velocity, not a measured robot twist. Even a within-limits result
    remains INCONCLUSIVE because steering/yaw/wheel state is unavailable.
    """
    if not isinstance(candidate_xy, (list, tuple)) or not (
            2 <= len(candidate_xy) <= 13):
        return _base("NO_BOUNDED_XY_CANDIDATE")
    if (type(dt_s) not in (int, float) or not math.isfinite(dt_s) or
            not (.1 <= dt_s <= .5)):
        return _base("INVALID_DISCRETE_TIME")
    points = []
    for p in candidate_xy:
        if (not isinstance(p, (tuple, list)) or len(p) != 2 or
                any(type(x) not in (int, float) or not math.isfinite(x)
                    for x in p)):
            return _base("INVALID_XY_CANDIDATE")
        points.append((float(p[0]), float(p[1])))
    if any(abs(x) > 100000 or abs(y) > 100000 for x, y in points):
        return _base("INVALID_XY_CANDIDATE")
    velocities = [
        ((b[0] - a[0]) / dt_s, (b[1] - a[1]) / dt_s)
        for a, b in zip(points, points[1:])
    ]
    speeds = [math.hypot(*v) for v in velocities]
    # Includes the first acceleration relative to G3's ZERO-MOTION model.
    # This term must not be confused with measured chassis acceleration.
    prior = (0., 0.)
    accelerations = []
    for v in velocities:
        accelerations.append(
            math.hypot(v[0] - prior[0], v[1] - prior[1]) / dt_s)
        prior = v
    max_speed = max(speeds)
    max_acc = max(accelerations)
    # G3 stores XY points rounded to 4 decimals; cover quantization noise.
    eps_speed = .001
    eps_accel = .005
    chassis_over = (
        max_speed > CHASSIS_MAX_LINEAR_SPEED_MPS + eps_speed or
        max_acc > CHASSIS_MAX_LINEAR_ACCEL_MPS2 + eps_accel)
    scan_over = (
        max_speed > SCAN_MAX_SPEED_MPS + eps_speed or
        max_acc > SCAN_MAX_ACCEL_MPS2 + eps_accel)
    nav_speed_over = max_speed > NAV_REQUIREMENT_MAX_SPEED_MPS + eps_speed
    result = _base(
        "CONFIGURED_CHASSIS_TRANSLATION_EXCEEDED" if chassis_over else
        "SCAN_TRANSLATIONAL_PLANNING_BOUNDS_EXCEEDED" if scan_over else
        "TRANSLATIONAL_BOUNDS_WITHIN_CONFIG_4WS_UNVERIFIED")
    result.update({
        "samples": len(points),
        "assumed_dt_s": dt_s,
        "speed_max_mps": round(max_speed, 5),
        "accelerations_max_mps2": round(max_acc, 5),
        "chassis_config_limits_exceeded": chassis_over,
        "scan_planning_limits_exceeded": scan_over,
        "navigation_requirement_speed_exceeded": nav_speed_over,
        "modeled_velocity_mps": [[round(v[0], 5), round(v[1], 5)]
                                 for v in velocities],
        "modeled_acceleration_norm_mps2": [round(a, 5) for a in accelerations],
        "first_acceleration_measured": False,
        "configuration_only_not_measured_motion": True,
        "insufficient_for_4ws_feasibility": True,
    })
    return result


def screen_g3_result(result):
    """Read-only G3 post-solve diagnostics; no filtering to control outputs."""
    if not isinstance(result, dict) or not result.get("candidate_xy_m"):
        return _base("G3_HAS_NO_CANDIDATE")
    return screen_xy_candidate(
        result.get("candidate_xy_m"),
        result.get("discretization_dt_s"))
