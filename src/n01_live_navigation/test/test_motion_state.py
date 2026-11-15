from n01_live_navigation.motion_state import MotionStateMachine,Phase


def test_motion_phases_and_terminal_states_are_explicit():
    f=MotionStateMachine()
    for status,phase in [('MOVING',Phase.TRACKING),('ALIGN_PATH',Phase.TURNING),
                         ('REPLANNING_AFTER_ROTATION_BLOCKED',Phase.REPLANNING),
                         ('MOVING',Phase.TRACKING),('VERIFYING_ARRIVAL',Phase.VERIFYING),
                         ('ARRIVED',Phase.ARRIVED)]:
        assert f.observe(status,10.)==phase
    assert f.observe('MOVING',11.)==Phase.ARRIVED


def test_phase_dwell_time_is_not_reset_by_heartbeats():
    f=MotionStateMachine();f.observe('NO_PATH',10.)
    f.observe('NO_PATH',12.)
    assert f.entered==10. and f.transitions==1
    assert f.observe('RECOVERY_FAILED',40.)==Phase.FAILED
    assert f.observe('MOVING',41.)==Phase.FAILED
