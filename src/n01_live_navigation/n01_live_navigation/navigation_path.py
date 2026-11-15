"""Splice the currently validated local segment into a forward global suffix."""
import math


def execution_prefix(points, position, max_length=1.0):
    """Return a near-term prefix of the collision-checked candidate, without
    replacing the reference goal or accepting an unverified future segment.
    """
    if len(points) < 3 or not math.isfinite(max_length) or max_length < .3:
        raise ValueError('invalid_execution_preview')
    if any(not all(math.isfinite(float(v)) for v in point) for point in points):
        raise ValueError('nonfinite_execution_preview')
    start = min(range(len(points)), key=lambda i: math.dist(points[i][:2], position[:2]))
    if math.dist(points[start][:2], position[:2]) > .3:
        raise ValueError('candidate_too_far_from_robot')
    selected = [points[start]]
    length = 0.
    for point in points[start+1:]:
        step = math.dist(selected[-1][:2], point[:2])
        if length+step > max_length:
            break
        selected.append(point)
        length += step
    if len(selected) < 3:
        raise ValueError('execution_preview_too_short')
    return selected


def splice_path(reference, local, join_tolerance=.30, reference_tail_only=False):
    if len(reference) < 2 or len(local) < 2:
        raise ValueError('missing_path')
    if any(not all(math.isfinite(v) for v in p[:2]) for p in [*reference, *local]):
        raise ValueError('nonfinite_path')
    start = min(range(len(reference)), key=lambda i: math.dist(reference[i][:2], local[0][:2]))
    end = min(range(start, len(reference)), key=lambda i: math.dist(reference[i][:2], local[-1][:2]))
    if not reference_tail_only and math.dist(reference[end][:2], local[-1][:2]) > join_tolerance:
        raise ValueError('local_global_join_gap')
    suffix = reference[end+1:]
    if not suffix and math.dist(reference[-1][:2], local[-1][:2]) > 1e-6:
        suffix = [reference[-1]]
    near_goal = math.dist(reference[-1][:2], local[-1][:2]) <= .08
    if near_goal:
        # Preserve the requested endpoint without a micron-scale backwards
        # suffix whose heading is dominated by spline sampling noise. The
        # executor still treats any suffix as unvalidated.
        suffix = [reference[-1]] if math.dist(reference[-1][:2], local[-1][:2]) > 1e-6 else []
    if suffix and not near_goal and not reference_tail_only:
        dx, dy = local[-1][0]-local[-2][0], local[-1][1]-local[-2][1]
        tx, ty = suffix[0][0]-local[-1][0], suffix[0][1]-local[-1][1]
        if math.hypot(dx,dy) > 1e-6 and math.hypot(tx,ty) > 1e-6:
            if (dx*tx+dy*ty)/(math.hypot(dx,dy)*math.hypot(tx,ty)) < .5:
                raise ValueError('join_heading_discontinuity')
    return [*local, *suffix], len(local)
