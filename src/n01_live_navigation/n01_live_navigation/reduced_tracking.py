"""Two-DOF path tracking reference; wheel allocation stays in the native chassis."""
import math


def braking_speed_limit(distance,speed,deceleration,margin=.08,reaction_s=.6):
    if (not all(math.isfinite(v)for v in (distance,speed,deceleration,margin,reaction_s))
            or min(distance,speed,margin,reaction_s)<0 or deceleration<=0):
        raise ValueError('invalid braking envelope')
    return math.sqrt(2*deceleration*max(0.,distance-margin-speed*reaction_s))


def translation_velocity(speed, heading_error):
    """Body-frame velocity toward the lookahead; no commanded body rotation."""
    if not math.isfinite(speed) or not math.isfinite(heading_error) or speed<0:
        raise ValueError('invalid_translation_reference')
    return speed*math.cos(heading_error),speed*math.sin(heading_error)


def prefix_requires_extension(position,prefix_end,goal,has_suffix):
    # A validated prefix already entering the arrival ball is sufficient to
    # stop at the requested goal tolerance; a duplicate tiny suffix is not a
    # reason to halt 20cm early.
    return (has_suffix and math.hypot(prefix_end[0]-goal[0],prefix_end[1]-goal[1])>.08
            and math.hypot(position[0]-prefix_end[0],position[1]-prefix_end[1])<.20)


def reduced_yaw_rate(speed, heading_error, target_distance, max_yaw,
                     spin_threshold=.01, max_curvature=1.):
    values=(speed,heading_error,target_distance,max_yaw,spin_threshold,max_curvature)
    if not all(math.isfinite(v) for v in values) or min(speed,max_yaw,spin_threshold,max_curvature)<0:
        raise ValueError('invalid_reduced_tracking_input')
    # Native IK snaps sub-threshold translation to zero. Never pair that with
    # nonzero yaw: it would request a spin while the speed ramp is starting.
    if speed < spin_threshold:
        return 0.
    curvature=2.*math.sin(heading_error)/max(.25,target_distance)
    curvature=max(-max_curvature,min(max_curvature,curvature))
    return max(-max_yaw,min(max_yaw,speed*curvature))
