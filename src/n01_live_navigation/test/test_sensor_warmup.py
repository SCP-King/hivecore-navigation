import numpy as np
from n01_live_navigation.sensor_warmup import stable_window


def window(scale=1.):
    return [(9_500_000_000+i*5_000_000,(0.,0.,scale)) for i in range(100)]


def test_requires_stable_fresh_window_and_accepts_input_unit_scale():
    for scale in (1.,9.81):
        assert stable_window(window(scale),[9_700_000_000,9_800_000_000,9_900_000_000],10_000_000_000)
    assert not stable_window(window()[:50],[10_000_000_000]*3,10_000_000_000)
    assert not stable_window(window(),[9_000_000_000]*3,10_000_000_000)
    assert not stable_window(window(),[10_000_000_000]*3,12_000_000_000)


def test_vibration_invalid_or_future_measurements_do_not_initialize_map():
    moving=[(t,(.4*(-1)**i,0.,1.)) for i,(t,_) in enumerate(window())]
    assert not stable_window(moving,[10_000_000_000]*3,10_000_000_000)
    bad=window();bad[-1]=(bad[-1][0],(np.nan,0.,1.))
    assert not stable_window(bad,[10_000_000_000]*3,10_000_000_000)
    assert not stable_window(window(),[10_000_000_000]*3,9_000_000_000)
