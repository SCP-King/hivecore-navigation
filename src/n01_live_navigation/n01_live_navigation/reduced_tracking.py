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
                     spin_threshold=.01, max_curvature=None):
    values=(speed,heading_error,target_distance,max_yaw,spin_threshold)
    if max_curvature is not None:values=(*values,max_curvature)
    if not all(math.isfinite(v) for v in values) or (speed<0 or max_yaw<0 or spin_threshold<0 or (max_curvature is not None and max_curvature<0)):
        raise ValueError('invalid_reduced_tracking_input')
    # Native IK snaps sub-threshold translation to zero. Never pair that with
    # nonzero yaw: it would request a spin while the speed ramp is starting.
    if speed < spin_threshold:
        return 0.
    curvature=2.*math.sin(heading_error)/max(.25,target_distance)
    if max_curvature is not None:
        curvature=max(-max_curvature,min(max_curvature,curvature))
    return max(-max_yaw,min(max_yaw,speed*curvature))


def forward_speed_factor(heading_error):
    """Continuous slowdown without a zero-speed hole before realignment."""
    if not math.isfinite(heading_error):raise ValueError('invalid_heading_error')
    return max(0.,math.cos(heading_error))
