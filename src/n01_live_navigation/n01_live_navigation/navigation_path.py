"""Splice the currently validated local segment into a forward global suffix."""
import math


def splice_path(reference, local, join_tolerance=.30):
    if len(reference) < 2 or len(local) < 2:
        raise ValueError('missing_path')
    if any(not all(math.isfinite(v) for v in p[:2]) for p in [*reference, *local]):
        raise ValueError('nonfinite_path')
    start = min(range(len(reference)), key=lambda i: math.dist(reference[i][:2], local[0][:2]))
    end = min(range(start, len(reference)), key=lambda i: math.dist(reference[i][:2], local[-1][:2]))
    if math.dist(reference[end][:2], local[-1][:2]) > join_tolerance:
        raise ValueError('local_global_join_gap')
    suffix = reference[end+1:]
    if suffix:
        dx, dy = local[-1][0]-local[-2][0], local[-1][1]-local[-2][1]
        tx, ty = suffix[0][0]-local[-1][0], suffix[0][1]-local[-1][1]
        if math.hypot(dx,dy) > 1e-6 and math.hypot(tx,ty) > 1e-6:
            if (dx*tx+dy*ty)/(math.hypot(dx,dy)*math.hypot(tx,ty)) < .5:
                raise ValueError('join_heading_discontinuity')
    return [*local, *suffix], len(local)
