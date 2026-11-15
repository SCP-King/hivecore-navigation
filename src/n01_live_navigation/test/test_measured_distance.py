import math
from n01_live_navigation.measured_distance import MeasuredDistance
from n01_live_navigation.reduced_tracking import braking_speed_limit

def test_stamped_twist_distance_does_not_use_doubled_pose():
    d=MeasuredDistance()
    for i in range(101):d.update(i*20_000_000,.2,0.)
    assert abs(d.total-.4)<1e-9
    d.update(2_000_000_000,.2,0.)
    assert abs(d.total-.4)<1e-9

def test_gap_is_not_silently_integrated():
    d=MeasuredDistance();d.update(0,.2,0.);d.update(200_000_000,.2,0.)
    assert d.gap and d.total==0.

def test_arrival_braking_allows_native_translation_before_arrival_ball():
    limit=braking_speed_limit(.081,.012,.04,margin=.04)
    assert limit>.012
    assert .012*.6+.012**2/(2*.04)<.04
