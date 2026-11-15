import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ground_model import estimate_ground


def surface(a=-.007, b=-.016, c=-.89):
    x, y = np.meshgrid(np.linspace(-2.5, 2.5, 90), np.linspace(-2.5, 2.5, 90))
    rng = np.random.default_rng(9)
    z = a*x+b*y+c+rng.normal(0, .006, x.shape)
    return np.column_stack((x.ravel(), y.ravel(), z.ravel()))


def test_floor_with_tilt_and_dense_furniture_does_not_use_pose_height():
    floor = surface()
    furniture = floor.copy()
    furniture[:, 2] += .75
    p = estimate_ground(np.vstack((floor, furniture, furniture)), (0, 0))
    assert abs(p.c+.89) < .01
    assert abs(p.a+.007) < .003
    assert abs(p.b+.016) < .003
    assert abs(p.height(2, -1)-(-.89-.014+.016)) < .01


def test_sparse_floor_fails_instead_of_guessing_height():
    with pytest.raises(ValueError):
        estimate_ground(np.zeros((10, 3)), (0, 0))


def test_narrow_strip_and_excessive_tilt_are_rejected():
    p = surface()
    with pytest.raises(ValueError):
        estimate_ground(p[np.abs(p[:, 1]) < .1], (0, 0))
    with pytest.raises(ValueError):
        estimate_ground(surface(a=.10, b=0), (0, 0))


def test_ceiling_only_cells_cannot_outvote_sparse_visible_floor():
    # Reproduces the real PCD failure: many XY cells contain only ceiling.
    x,y=np.meshgrid(np.linspace(-5,5,101), np.linspace(-5,5,101))
    ceiling=np.column_stack((x.ravel(), y.ravel(), np.full(x.size,2.)))
    x,y=np.meshgrid(np.linspace(-5,-2,70), np.linspace(-2,2,80))
    floor=np.column_stack((x.ravel(), y.ravel(), -.92-.006*y.ravel()))
    points=np.vstack((ceiling,floor))
    # Without a below-body search, the dominant ceiling can even be accepted
    # as a broad horizontal surface rather than yielding an explicit failure.
    assert estimate_ground(points,(0,0),6.).c > 1.5
    for body_z in (-.15,.1):
        g=estimate_ground(points,(0,0),6.,body_z=body_z)
        assert abs(g.c+.92)<.002
        assert abs(g.b+.006)<.001


def test_bad_body_search_height_never_generates_a_floor_fallback():
    with pytest.raises(ValueError, match='候选点不足'):
        estimate_ground(surface(),(0,0),6.,body_z=-3.)
    with pytest.raises(ValueError, match='高度无效'):
        estimate_ground(surface(),(0,0),6.,body_z=float('nan'))


def test_broad_floor_with_minority_support_amid_scattered_low_returns():
    rng=np.random.default_rng(42)
    x,y=np.meshgrid(np.linspace(-3,-.3,8),np.linspace(-2,2,6))
    floor=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-.9)))
    x,y=np.meshgrid(np.linspace(.3,3,10),np.linspace(-2,2,10))
    clutter=np.column_stack((x.ravel(),y.ravel(),rng.uniform(-.8,-.1,x.size)))
    points=np.repeat(np.vstack((floor,clutter)),3,axis=0)
    g=estimate_ground(points,(0,0),6.,body_z=.1)
    assert abs(g.c+.9)<.025
    assert abs(g.a)<.02 and abs(g.b)<.02


def test_two_broad_separate_floor_levels_are_rejected():
    x,y=np.meshgrid(np.linspace(-3,-.3,10),np.linspace(-2,2,10))
    lower=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-.9)))
    higher=lower.copy();higher[:,0]*=-1;higher[:,2]=-.6
    with pytest.raises(ValueError,match='两个平面'):
        estimate_ground(np.vstack((lower,higher)),(0,0),6.,body_z=.1)


def test_sparse_visible_floor_uses_measured_model_hint_without_global_ratio():
    x,y=np.meshgrid(np.linspace(-2,2,5),np.linspace(-2,2,4))
    floor=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-.9)))
    x,y=np.meshgrid(np.linspace(-3,3,16),np.linspace(-3,3,16))
    table=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-.44)))
    p=estimate_ground(np.vstack((floor,table)),(0,0),6.,body_z=-.15,height_hint=(-.92,.12))
    assert abs(p.c+.9)<.005
    assert p.support==20


def test_height_hint_alone_is_not_a_ground_measurement():
    x,y=np.meshgrid(np.linspace(-3,3,20),np.linspace(-3,3,20))
    table=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-.44)))
    with pytest.raises(ValueError,match='回波不足'):
        estimate_ground(table,(0,0),6.,body_z=-.15,height_hint=(-.92,.12))
