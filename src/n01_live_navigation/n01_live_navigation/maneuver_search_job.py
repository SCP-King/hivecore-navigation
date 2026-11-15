"""One bounded non-actuating search job, with fresh-scene admission on return."""
import math
from concurrent.futures import ThreadPoolExecutor
from .heading_search import revalidate_cached,wrap
from .maneuver_planner import plan_maneuver,swept_turn


def poll_maneuver(node,goal_id,start,target,collision,raw,capability):
    future=getattr(node,'maneuver_future',None)
    if future is not None:
        if not future.done():return None
        node.maneuver_future=None
        result=future.result()
        if node.maneuver_future_goal!=goal_id:return None
        if not result.forward.states:return result
        if result.rotation:
            if math.dist(start[:2],result.rotation[0][:2])>.04 or raw is None:return None
            rotation=swept_turn(start,result.rotation[-1][2],collision)
            if not rotation or not swept_turn(start,result.rotation[-1][2],raw):return None
            if not collision.free(result.forward.states) or not capability.segment_allowed(result.forward.states):return None
            result.rotation=rotation
        else:
            tail=revalidate_cached(result.forward.states,start,collision)
            if not tail or not capability.segment_allowed(tail):return None
            result.forward.states=tail
        return result
    if not hasattr(node,'maneuver_pool'):
        node.maneuver_pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='n01_local_search')
    if getattr(node,'maneuver_budget_goal',None)!=goal_id:
        node.maneuver_budget_goal=goal_id;node.maneuver_attempt=0
    budget=(2.,4.,8.)[min(node.maneuver_attempt,2)]
    node.maneuver_attempt+=1
    node.maneuver_future_goal=goal_id
    # All arguments are snapshot geometry; worker cannot publish ROS messages
    # or access live node state. New callbacks remain free to ingest sensors.
    node.maneuver_future=node.maneuver_pool.submit(plan_maneuver,start,target,collision,raw,
                                                 budget_s=budget,capability=capability)
    return None
