import numpy as np
from n01_live_navigation.core import four_wheel_steer_screen


def test_retiming_recomputes_native_motion_and_does_not_mutate_path():
    t=np.linspace(0,1.5,120)
    points=np.column_stack((np.sin(t),1-np.cos(t),np.zeros_like(t)))
    original=points.copy()
    result=four_wheel_steer_screen(points,max_speed=.05,allow_retime=True)
    assert result['recommended_speed_mps']<=.05
    assert result['recommended_speed_mps']>=.012
    np.testing.assert_array_equal(points,original)
    if result['necessary_conditions_pass']:
        assert result['max_abs_steering_rate_radps']<=1.570001


def test_rate_only_failure_is_retimed_and_reviewed_speed_is_lower(monkeypatch):
    import n01_live_navigation.hive2_kinematics as native
    def screen(twists,intervals,q,geometry):
        n=len(twists)
        return np.zeros((n,4)),np.zeros((n,4)),np.full((n,4),100*np.max(np.abs(twists[:,0])))
    monkeypatch.setattr(native,'screen_sequence',screen)
    points=np.column_stack((np.linspace(0,1,80),np.zeros((80,2))))
    result=four_wheel_steer_screen(points,max_speed=.05,allow_retime=True)
    assert result['retiming_applied'] and result['necessary_conditions_pass']
    assert .012<=result['recommended_speed_mps']<.05
    assert result['max_abs_steering_rate_radps']<1.57


def test_sharp_geometry_is_not_accepted_by_merely_reducing_speed():
    t=np.linspace(0,1,100)
    points=np.column_stack((.1*np.sin(t),.1*(1-np.cos(t)),np.zeros_like(t)))
    result=four_wheel_steer_screen(points,max_speed=.05,allow_retime=True)
    assert not result['necessary_conditions_pass']
    assert any(f['reason']=='path_curvature_requires_stop_and_turn' for f in result['kinematic_failures'])
