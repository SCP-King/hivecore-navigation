import numpy as np
from types import SimpleNamespace as S
from n01_live_navigation.maneuver_continuity import retained_maneuver
from n01_live_navigation.heading_search import FootprintCollision


def scene(points=()):
    return FootprintCollision(np.asarray(points).reshape(-1,3),[0,0,0],.6,[[-4,-4],[4,4]],
        footprint=np.array([[-.4,-.2],[.4,-.2],[.4,.2],[-.4,.2]]))

def fixture():
    states=[(i*.02,0.,0.,0.)for i in range(101)]
    return dict(goal_id=1,forward=states,rotation=[])


def retain(cache,**kw):
    args=dict(goal_id=1,start=(.2,0.,0.,0.),goal=(2.,0.),collision=scene(),raw_collision=scene(),
              capability=S(segment_allowed=lambda _:True),forward_check=lambda _:True)
    args.update(kw)
    return retained_maneuver(cache,**args)


def test_valid_path_survives_repeated_updates_without_geometry_changes():
    cache=fixture()
    for i in range(1,50):
        m=retain(cache,start=(i*.02,0.,0.,0.))
        assert m and m.forward.states==cache['forward'][i:]


def test_goal_change_new_obstacle_and_native_transition_reject_reuse():
    c=fixture()
    assert retain(c,goal_id=2) is None
    assert retain(c,collision=scene([[1.,0.,.3]])) is None
    assert retain(c,forward_check=lambda _:False) is None
    assert retain(c,capability=S(segment_allowed=lambda _:False)) is None


def test_short_safe_prefix_is_retained_while_extension_is_computed():
    c=fixture()
    assert retain(c,start=(1.7,0.,0.,0.)) is not None
    assert retain(c,start=(1.7,0.,0.,0.),goal=(3.,0.)) is not None


def test_rotation_target_is_locked_and_raw_scene_remains_required():
    c=fixture();c['rotation']=[(0.,0.,0.,0.),(0.,0.,1.,0.)]
    m=retain(c,start=(0.,0.,.2,0.))
    assert m and abs(m.rotation[-1][2]-1.)<1e-9
    assert retain(c,start=(0.,0.,.2,0.),raw_collision=None) is None
    assert retain(c,start=(0.,0.,.99,0.)) is None
