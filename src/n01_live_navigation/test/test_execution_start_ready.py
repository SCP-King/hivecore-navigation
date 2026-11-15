from types import SimpleNamespace as S
from n01_live_navigation.slow_motion_executor import SlowMotionExecutor

def node():
    return S(observer_ready=True,manual_override=False,estop=False,dec_stop=0,
        pose=(0,0,0),pose_rx=10.,pose_stamp_age_at_rx=.02,steering_feedback=([0.]*4,[0.]*4),
        steering_feedback_rx=10.,odom_rx=10.,readiness_rx=10.,readiness_timeout=2.,
        readiness=dict(navigation_ready=True,blockers=[]),path=object(),path_rx=10.,path_status_rx=10.,
        expected_goal=(1.,0.),path_status=dict(valid=True,execution_review=dict(candidate_review_ready=True,goal_xy=[1.,0.])))

def test_backend_arming_uses_its_own_current_inputs_not_gui_cache():
    n=node()
    assert SlowMotionExecutor._execution_start_ready(n,10.1)
    assert not SlowMotionExecutor._execution_start_ready(n,10.5)
    n=node();n.manual_override=True
    assert not SlowMotionExecutor._execution_start_ready(n,10.1)
    n=node();n.path_status['execution_review']['goal_xy']=[2.,0.]
    assert not SlowMotionExecutor._execution_start_ready(n,10.1)
