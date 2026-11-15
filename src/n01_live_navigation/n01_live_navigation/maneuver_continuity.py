"""Retain geometry only after a fresh scene and native transition review.

A cached route is never a cached drive permit. Goal identity, progress, current
pose, footprint, environment and steering must be checked on every reuse.
"""
import math
from .heading_search import revalidate_cached, SearchResult, wrap
from .maneuver_planner import Maneuver, swept_turn


def retained_maneuver(cache, goal_id, start, goal, collision, raw_collision,
                      capability, forward_check):
    if not cache or cache['goal_id'] != goal_id:
        return None
    if cache['rotation']:
        # Lock the selected turn heading while executing it. Recomputing the
        # bearing from pose noise must not move the yaw target every 200 ms.
        target=cache['rotation'][-1][2]
        if abs(wrap(target-start[2]))<math.radians(3):
            return None  # Require a new forward plan from measured heading.
        if math.dist(start[:2],cache['rotation'][0][:2])>.04 or raw_collision is None:
            return None
        rotation=swept_turn(start,target,collision)
        if not rotation or not swept_turn(start,target,raw_collision):return None
        forward=cache['forward']
        if not collision.free(forward) or not capability.segment_allowed(forward):return None
        return Maneuver(SearchResult(forward,'revalidated_forward_preview',0,0.),rotation,'rotate_then_replan_forward')
    tail=revalidate_cached(cache['forward'],start,collision)
    if not tail or not capability.segment_allowed(tail):return None
    remaining=sum(math.dist(a[:2],b[:2])for a,b in zip(tail,tail[1:]))
    # Extend a short local prefix early, but do not replan merely because a
    # complete route is naturally becoming short near its final goal.
    # A short but safe prefix remains executable while its extension is computed.
    if not forward_check(tail):return None
    return Maneuver(SearchResult(tail,'revalidated_current_forward_path',0,0.),[], 'forward_heading_path')
