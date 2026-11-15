from copy import deepcopy
from n01_live_navigation.degraded_motion import continuity_allowed,degraded_speed_limit
from n01_live_navigation.feedback_continuity import SteeringRateGuard

def health():
    return dict(blockers=['live_localization_or_scan_bridge_not_ready','scan_motion_compensation_not_ready'],
        bridge=dict(odom_receive_fresh=True,body_pose_live_fresh=True,sensor_extrinsic_ready=True,
        cloud_output_fresh=True,cloud_output_age_s=.2,pose_prediction=dict(fault='')))

def test_transient_health_gap_degrades_only_running_fresh_reviewed_motion():
    h=health()
    assert continuity_allowed(h,.2,True,True)
    assert not continuity_allowed(h,.401,True,True)
    assert not continuity_allowed(h,.2,False,True)
    assert not continuity_allowed(h,.2,True,False)

def test_hard_failures_and_stale_inputs_never_use_grace():
    for k in ['odom_receive_fresh','body_pose_live_fresh','sensor_extrinsic_ready','cloud_output_fresh']:
        h=health();h['bridge'][k]=False
        assert not continuity_allowed(h,.1,True,True)
    h=health();h['blockers'].append('localization_status_missing_or_stale')
    assert not continuity_allowed(h,.1,True,True)
    h=health();h['bridge']['pose_prediction']['fault']='innovation_limit'
    assert not continuity_allowed(h,.1,True,True)
    h=health();h['bridge']['cloud_output_age_s']=.36
    assert not continuity_allowed(h,.1,True,True)

def test_degraded_speed_respects_review_and_native_minimum():
    assert degraded_speed_limit(.03,.01)==.015
    assert degraded_speed_limit(.015,.01)==.012
    assert degraded_speed_limit(.008,.01)==.008

def test_soft_spike_degrades_then_recovers_but_hard_stops():
    g=SteeringRateGuard()
    assert g.update([1.6]*4,0.) and g.degraded
    assert g.update([1.2]*4,.05) and not g.degraded
    assert not g.update([3.15]*4,.1)
    assert g.blocked and g.degraded


def test_current_execution_evidence_does_not_depend_on_last_green_or_already_moving():
    from n01_live_navigation.degraded_motion import reviewed_path_continuity
    h=health();h['navigation_ready']=False
    h['blockers'].append('global_robot_pose_anchor_outside_scan_time_budget')
    assert reviewed_path_continuity(h,True)
    # Each subsequent check must use fresh current evidence, with no arbitrary
    # 400ms-from-last-green expiry or minimum-speed condition.
    for _ in range(50):assert reviewed_path_continuity(h,True)
    assert not reviewed_path_continuity(h,False)
    h['bridge']['cloud_output_fresh']=False
    assert not reviewed_path_continuity(h,True)


def test_unknown_blockers_and_prediction_fault_still_stop_execution():
    from n01_live_navigation.degraded_motion import reviewed_path_continuity
    h=health();h['blockers'].append('localization_status_missing_or_stale')
    assert not reviewed_path_continuity(h,True)
    h=health();h['bridge']['pose_prediction']['fault']='budget_exceeded'
    assert not reviewed_path_continuity(h,True)
