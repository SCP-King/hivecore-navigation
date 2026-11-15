import pytest
from types import SimpleNamespace as S
from n01_live_navigation.pose_policy import control_pose_fresh,tracking_mode
from n01_live_navigation.delayed_pose import DelayedPoseFusion
from n01_live_navigation.live_scan_bridge import LiveScanBridge


def test_three_second_global_gap_can_continue_with_measured_odometry():
    f=DelayedPoseFusion(horizon=5.,travel_limit=.12,turn_limit=.20)
    anchor=((0.,0.,0.),(0.,0.,0.,1.))
    f.accept(10**9,anchor,anchor,10**9,anchor)
    estimate=f.pose(4*10**9,((.09,0.,0.),anchor[1]))
    assert estimate[0][0]==pytest.approx(.09)
    bridge=dict(status='LIVE_SCAN_INPUTS_READY',body_pose_live_fresh=True,
        robot_pose_stamp_age_s=3.,robot_pose_raw_receive_fresh=False,pose_prediction={'fault':''})
    assert tracking_mode(bridge)=='PREDICTED_TRACKING'
    assert control_pose_fresh(.1,.05)
    assert not control_pose_fresh(3.,.05)  # Never restamp a held raw pose.
    for stamp,position in [(7*10**9,.09),(4*10**9,.13)]:
        with pytest.raises(ValueError):f.pose(stamp,((position,0.,0.),anchor[1]))
    bridge['pose_prediction']['fault']='correction_jump'
    assert tracking_mode(bridge)=='WAITING'


def test_status_silence_tolerated_only_with_live_odom_and_valid_anchor(monkeypatch):
    import n01_live_navigation.live_scan_bridge as module
    monkeypatch.setattr(module.time,'monotonic',lambda:4.)
    n=S(status_ok=True,status_rx_monotonic=1.,status_stamp_ns=10**9,max_status_age=1.,
        max_pose_stamp_age=5.,pose_fusion=S(anchor=True,fault=''),pose_samples=[(10**9,)],
        twist_odometry=S(segment_start=10**9),_odom_rx_is_fresh=lambda:True,
        get_clock=lambda:S(now=lambda:S(nanoseconds=4*10**9)))
    assert LiveScanBridge._status_is_fresh(n)
    n.status_ok=False
    assert not LiveScanBridge._status_is_fresh(n)
    n.status_ok=True;n._odom_rx_is_fresh=lambda:False
    assert not LiveScanBridge._status_is_fresh(n)
