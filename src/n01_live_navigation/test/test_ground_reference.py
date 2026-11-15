import math
import numpy as np
import pytest
from n01_live_navigation.core import compose_transform
from n01_live_navigation.ground_reference import project_footprint


def test_negative_map_ground_is_not_zero_or_nominal_body_offset():
    pose, relative = project_footprint((-3.4177, 1.80395, .00165), (0,0,0,1),
                                     (-.0014784564,-.0146615688,-.8739077899))
    assert pose[0][2] == pytest.approx(-.8953036065)
    assert relative[0][2] == pytest.approx(-.8969536065)


def test_squat_changes_body_to_floor_but_not_map_floor():
    for height in (.6, .775, .95):
        pose, relative = project_footprint((1,2,-.89+height),(0,0,0,1),(0,0,-.89))
        assert pose[0][2] == -.89
        assert relative[0][2] == pytest.approx(-height)


def test_projection_chain_preserves_tilted_body_and_levels_footprint():
    # Combined body roll/pitch/yaw, with nonzero world position and floor slope.
    q=np.array([.1,.15,.3,.9]);q/=np.linalg.norm(q)
    p=[2.,-1.,.2]
    ground, relative=project_footprint(p,q,(.01,-.02,-.8))
    reconstructed=compose_transform(p,q,*relative)
    np.testing.assert_allclose(reconstructed[0],ground[0],atol=1e-9)
    np.testing.assert_allclose(reconstructed[1],ground[1],atol=1e-9)
    assert ground[0][2] == pytest.approx(-.76)
    assert ground[1][:2] == [0,0]


@pytest.mark.parametrize('p,q,plane', [([0,0,float('nan')],[0,0,0,1],[0,0,0]),
                                      ([0,0,0],[0,0,0,0],[0,0,0]),
                                      ([0,0,0],[0,math.sqrt(.5),0,math.sqrt(.5)],[0,0,0])])
def test_invalid_or_undefined_geometry_is_rejected(p,q,plane):
    with pytest.raises(ValueError):project_footprint(p,q,plane)
