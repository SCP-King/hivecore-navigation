import math
import numpy as np
from n01_live_navigation.heading_search import FootprintCollision, primitive, search
from n01_live_navigation.rotation_guard import rotation_arc_clear


def collision(points=()):
    return FootprintCollision(np.array(points).reshape((-1, 3)), [0,0,0], .775, [[-4,-4],[4,4]])


def test_forward_straight_and_smooth_turn_reach_without_snapping():
    for goal in [(1.,0.), (1.8,.6)]:
        r=search((0,0,0,0),goal,collision(),budget_s=1.)
        assert r.states, r.reason
        s=np.array(r.states)
        assert np.linalg.norm(s[-1,:2]-goal)<=.1
        assert np.max(np.abs(s[:,3]))<=1.
        assert np.max(np.abs(np.diff(s[:,3])))<=.040001
        step=np.diff(s[:,:2],axis=0)
        heading=np.column_stack((np.cos(s[:-1,2]),np.sin(s[:-1,2])))
        assert np.all(np.sum(step*heading,axis=1)>0)


def test_collision_uses_orientation_and_rejects_wall_and_outside_window():
    c=collision([[.7,0,.5]])
    assert c.free([[0,0,math.pi/2]])
    assert not c.free([[0,0,math.pi]])
    wall=[[.9,y,.5] for y in np.linspace(-4,4,100)]
    r=search((0,0,0,0),(2,0),collision(wall),budget_s=.15)
    assert not r.states
    assert not collision().free([[3.9,0,0]])


def test_bounded_search_returns_no_partial_motion_on_timeout_or_rear_goal():
    assert not search((0,0,0,0),(-1,0),collision(),budget_s=0).states
    assert not search((0,0,0,0),(-1,0),collision(),max_expansions=1).states


def test_partial_rotation_does_not_use_full_circle_but_checks_swept_body():
    # Rear disk swings below robot during a positive small turn. Obstacle above
    # it is inside old 0.8 m circle but outside this actual swept footprint.
    assert rotation_arc_clear([[0,.70,.5]],(0,0,.775),(0,0,0),0,.2,min_observations=0)
    assert not rotation_arc_clear([[-.43,-.38,.5]],(0,0,.775),(0,0,0),0,.5,min_observations=0)
    # A low box is not removed by the previous 6 cm floor threshold.
    assert not rotation_arc_clear([[0,0,.05]],(0,0,.775),(0,0,0),0,.2,min_observations=0)


def test_display_recipe_matches_actual_collision_radius():
    from n01_live_navigation.heading_search import footprint_description
    c=collision()
    recipe=footprint_description()
    assert recipe['physical_radius_m']==.30
    assert recipe['environment_margin_m']==.06
    assert abs(c.radius-recipe['collision_query_radius_m'])<1e-10
    assert abs(recipe['radius_m']-.30)<1e-10


def test_environment_six_centimeters_is_counted_once_in_collision_query():
    c=collision([[-.1,.35,.5]])
    assert not c.free([[0,0,0]])
    c=collision([[-.1,.40,.5]])
    assert c.free([[0,0,0]])  # Would be blocked if 6cm were added twice.


def test_slew_equivalent_primitives_are_collision_checked_once():
    from types import SimpleNamespace as S
    from n01_live_navigation.heading_search import search,primitive
    import numpy as np
    class Record:
        def free(self,states):return True
        def free_batch(self,segments):
            self.segments=segments
            return np.zeros(len(segments),bool)
    scene=Record();choices=(-50.,-7.,-3.,-1.,-.5,0.,.5,1.,3.,7.,50.)
    cap=S(max_curvature=100.,choices=choices,segment_allowed=lambda s:True)
    search((0.,0.,0.,0.),(1.,0.),scene,budget_s=.5,capability=cap)
    key=lambda s:tuple(np.asarray(s).round(10).ravel())
    assert {key(s)for s in scene.segments}=={key(primitive((0.,0.,0.,0.),k))for k in choices}
    assert len(scene.segments)<len(choices)
