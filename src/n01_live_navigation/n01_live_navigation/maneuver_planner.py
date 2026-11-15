"""Forward curves plus a checked in-place turn. Never authorizes a virtual drive.

The forward preview after a turn is conditional. Only the rotation prefix can
be executed until a new measured heading produces a new forward-only review.
"""
import math,time
from dataclasses import dataclass
import numpy as np
from .heading_search import search,SearchResult,wrap

@dataclass
class Maneuver:
    forward: SearchResult
    rotation: list
    reason: str


def swept_turn(start,target,collision):
    angle=wrap(target-start[2]);steps=max(2,math.ceil(abs(angle)/math.radians(2)))
    yaws=np.linspace(start[2],start[2]+angle,steps+1)
    states=[(start[0],start[1],float(y),0.)for y in yaws]
    radius=collision.radius
    rho=(np.linalg.norm(collision.polygon,axis=1).max()if collision.polygon is not None else .75)
    try:
        collision.radius+=float(rho)*2*math.sin(abs(angle)/steps/4)
        return states if collision.free(states)else []
    finally:collision.radius=radius


def plan_maneuver(start,goal,collision,turn_collision=None,budget_s=.12):
    began=time.monotonic();desired=math.atan2(goal[1]-start[1],goal[0]-start[0])
    angle=abs(wrap(desired-start[2]));direct=None
    if angle<=math.radians(35):
        direct=search(start,goal,collision,budget_s=min(.04,budget_s))
        if direct.states:return Maneuver(direct,[],'forward_heading_path')
    if turn_collision is not None:
        for offset in (0.,math.radians(15),-math.radians(15),math.radians(30),-math.radians(30)):
            left=budget_s-(time.monotonic()-began)
            if left<=0:break
            target=wrap(desired+offset)
            if abs(wrap(target-start[2]))<math.radians(3):continue
            rotation=swept_turn(start,target,collision)
            if not rotation or not swept_turn(start,target,turn_collision):continue
            forward=search((start[0],start[1],target,0.),goal,collision,budget_s=left)
            if forward.states:return Maneuver(forward,rotation,'rotate_then_replan_forward')
    left=budget_s-(time.monotonic()-began)
    if direct is None and left>0:
        direct=search(start,goal,collision,budget_s=left)
        if direct.states:return Maneuver(direct,[],'forward_heading_path')
    return Maneuver(direct or SearchResult([],'no_safe_maneuver',0,(time.monotonic()-began)*1000),[],
                    'rotation_perception_unavailable'if turn_collision is None and angle>math.radians(35)
                    else'no_collision_checked_turn_or_forward_route')
