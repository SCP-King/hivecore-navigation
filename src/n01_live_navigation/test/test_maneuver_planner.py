import math
import numpy as np
from n01_live_navigation.heading_search import FootprintCollision
from n01_live_navigation.maneuver_planner import plan_maneuver,swept_turn

P=np.array([[-.4,-.15],[.4,-.15],[.4,.15],[-.4,.15]])
def scene(points=()):return FootprintCollision(np.array(points).reshape(-1,3),[0,0,0],.6,[[-4,-4],[4,4]],footprint=P)

def test_rear_goal_generates_turn_then_conditional_forward_path():
    p=plan_maneuver((0,0,0,0),(-1,0),scene(),scene(),budget_s=.5)
    assert p.rotation and p.forward.states
    assert abs(p.rotation[-1][2])==math.pi
    assert all(s[:2]==(0,0)for s in p.rotation)
    assert p.forward.states[0][2]==p.rotation[-1][2]
    assert np.linalg.norm(np.array(p.forward.states[-1][:2])-[-1,0])<.06

def test_front_goal_drives_with_curvature_without_unneeded_spin():
    p=plan_maneuver((0,0,0,0),(1.2,.2),scene(),scene(),budget_s=.5)
    assert p.forward.states and not p.rotation

def test_turn_cannot_use_filtered_front_scene_to_ignore_rear_obstacle():
    raw=scene([[-.30,.28,.4]])
    assert raw.free([[0,0,0]])
    assert not swept_turn((0,0,0,0),math.pi,raw)
    p=plan_maneuver((0,0,0,0),(-1,0),scene(),raw,budget_s=.1)
    assert not p.rotation

def test_missing_full_azimuth_scene_does_not_authorize_rear_rotation():
    p=plan_maneuver((0,0,0,0),(-1,0),scene(),None,budget_s=.01)
    assert not p.rotation and p.reason=='rotation_perception_unavailable'
