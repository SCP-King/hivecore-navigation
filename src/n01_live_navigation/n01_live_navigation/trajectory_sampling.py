"""Geometric prefix selection for offline model tests; no execution permission."""
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

