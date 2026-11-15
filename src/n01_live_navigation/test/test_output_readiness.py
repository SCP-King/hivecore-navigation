from n01_live_navigation.output_readiness import compensated_output_ready


def valid(**changes):
    args=dict(snapshot=dict(scan_stamp_ns=100,anchor_stamp_ns=90,extrinsic_verified=True),
        processed_stamp=100,output_current=True,pose_fresh=True,odom_fresh=True,prediction_fault='')
    args.update(changes)
    return compensated_output_ready(**args)


def test_pending_next_scan_cannot_invalidate_successful_current_output():
    snapshot=dict(scan_stamp_ns=100,anchor_stamp_ns=90,extrinsic_verified=True)
    attempt=dict(scan_stamp_ns=110,anchor_stamp_ns=95)
    assert valid(snapshot=snapshot)  # independent of subsequent failed attempts
    assert not valid(snapshot=attempt)
    assert valid(snapshot=snapshot)


def test_stale_output_missing_feedback_and_prediction_fault_revoke_immediately():
    for key in ('output_current','pose_fresh','odom_fresh'):
        assert not valid(**{key:False})
    assert not valid(prediction_fault='prediction_budget_exceeded')
    assert not valid(snapshot={})
    assert not valid(snapshot=dict(scan_stamp_ns=100,anchor_stamp_ns=90,extrinsic_verified=False))
