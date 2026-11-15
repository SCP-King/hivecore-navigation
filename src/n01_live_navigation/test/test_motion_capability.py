import math
import numpy as np
from n01_live_navigation.motion_capability import ForwardCapability
from n01_live_navigation.hive2_kinematics import Hive2Geometry
from n01_live_navigation.heading_search import primitive, search

class EmptyScene:
    def free(self,s):return True
    def free_batch(self,s):return np.ones(len(s),bool)


def test_observed_tight_turns_pass_native_domain():
    c=ForwardCapability(Hive2Geometry(),[0.]*4,.03)
    assert c.accepts(1/.69) and c.accepts(-1/.89)
    assert not c.accepts(c.max_curvature+1.)
    # This checks geometry, not a promise about wheel transition time.
    assert any(abs(k)>1. for k in c.choices)


def test_search_can_start_from_real_tight_measured_turn():
    c=ForwardCapability(Hive2Geometry(),[0.]*4,.03)
    start=(0.,0.,0.,1.4)
    end=primitive(start,1.4,length=.4)[-1]
    r=search(start,end[:2],EmptyScene(),budget_s=.5,capability=c)
    assert r.states
    assert all(c.accepts(s[3])for s in r.states)


def test_physical_steering_limit_is_still_enforced():
    c=ForwardCapability(Hive2Geometry(controller_steering_limit=.1),[0.]*4,.03)
    assert c.accepts(0.)
    assert not c.accepts(1.4)
