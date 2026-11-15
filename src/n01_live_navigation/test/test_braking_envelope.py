import pytest
from n01_live_navigation.reduced_tracking import braking_speed_limit

def test_high_speed_is_limited_by_short_validated_path():
 assert braking_speed_limit(1.,.8,.04)<.2
 assert braking_speed_limit(.2,.8,.04)==0
 assert braking_speed_limit(10.,.8,.04)>.8

def test_invalid_braking_geometry_rejected():
 for values in [(1.,float('nan'),.04),(1.,.8,0),(-1.,.8,.04)]:
  with pytest.raises(ValueError):braking_speed_limit(*values)
