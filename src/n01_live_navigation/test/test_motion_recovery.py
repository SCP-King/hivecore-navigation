import math
from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
import pytest
from n01_live_navigation.angular_travel import AngularTravel
from n01_live_navigation.heading_search import FootprintCollision, primitive, revalidate_cached
from n01_live_navigation.slow_motion_executor import SlowMotionExecutor


def test_measured_angular_travel_and_discontinuity_are_explicit():
    a=AngularTravel()
    for i in range(51):a.update(10**9+i*20_000_000,.08)
    assert abs(a.angle-.08)<1e-10
    assert a.segment==0
    with pytest.raises(ValueError):a.update(2*10**9,.08)
    a.update(2_200_000_000,.08)
    assert a.segment==1 and a.angle==0
    with pytest.raises(ValueError):a.update(2_220_000_000,float('nan'))


def test_reuse_checks_new_obstacles_and_heading_without_moving_old_geometry():
    start=(0,0,0,0);states=[start]+primitive(start,0,length=1.)
    c=FootprintCollision([], [0,0,0], .775,[[-3,-3],[3,3]])
    assert revalidate_cached(states,(.01,0,0,0),c)
    assert revalidate_cached(states,(.1,.10,0,0),c) is None
    assert revalidate_cached(states,(0,0,.1,0),c) is None
    blocked=FootprintCollision([[.8,0,.5]], [0,0,0], .775,[[-3,-3],[3,3]])
    assert revalidate_cached(states,start,blocked) is None
    np.testing.assert_array_equal(states[0],start)


def test_rotation_mismatch_stops_then_enters_bounded_replanning(monkeypatch):
    from n01_live_navigation import rotation_task
    monkeypatch.setattr(rotation_task,'control',lambda n,t:setattr(n,'state','ROTATION_POSE_MISMATCH'))
    n=S(path_turn_active=True,turn_stalled_since=None,rotation_complete=False,
        get_clock=lambda:S(now=lambda:S(nanoseconds=123)),_request_turn_replan=Mock())
    assert SlowMotionExecutor._face_global_path(n,10.)
    assert n.path_turn_active
    assert SlowMotionExecutor._face_global_path(n,12.1)
    assert not n.path_turn_active and n.turn_recovery_until==42.1
    assert n.turn_measurement_after_ns==123
    n._request_turn_replan.assert_called_once()
