import math
from types import SimpleNamespace as S
from unittest.mock import Mock
from n01_live_navigation.slow_motion_executor import SlowMotionExecutor


def node(target):
    path=S(poses=[S(pose=S(position=S(x=0.,y=0.))),S(pose=S(position=S(x=target[0],y=target[1])))])
    return S(path_turn_active=False,global_path=path,global_path_rx=10.,pose=(0,0,0),
             expected_goal=target,_zero=Mock())


def test_turn_requires_fresh_current_goal_and_does_not_send_translation():
    n=node((-1,0))
    assert SlowMotionExecutor._face_global_path(n,10.1)
    assert n.path_turn_active and abs(n.target_yaw)==math.pi
    n._zero.assert_called_once()
    n=node((1,0));assert not SlowMotionExecutor._face_global_path(n,10.1)
    n=node((-1,0));assert not SlowMotionExecutor._face_global_path(n,12.)
    n=node((-1,0));n.expected_goal=(1,0)
    assert not SlowMotionExecutor._face_global_path(n,10.1)


def test_blocked_rotation_replans_same_goal_with_bounded_retries():
    from types import SimpleNamespace as S
    from builtin_interfaces.msg import Time
    n=S(requested_goal_xy=(1.,2.),expected_goal=None,turn_replan_count=0,turn_replan_at=0.,
        get_clock=lambda:S(now=lambda:S(to_msg=lambda:Time(sec=10))),replan_goal_pub=Mock(),_zero=Mock())
    for t in (10.,11.,18.,19.,26.,34.):
        SlowMotionExecutor._request_turn_replan(n,t)
    assert n.replan_goal_pub.publish.call_count==3
    msg=n.replan_goal_pub.publish.call_args.args[0]
    assert msg.header.frame_id=='map' and msg.pose.position.x==1. and msg.pose.position.y==2.
    assert msg.pose.position.z==0.


def test_recovery_suppresses_immediate_repeat_rotation_and_missing_goal_fails():
    n=node((-1,0));n.turn_recovery_until=40.
    assert not SlowMotionExecutor._face_global_path(n,10.)
    n.turn_recovery_until=0.;n.turn_suppressed_until=40.
    assert not SlowMotionExecutor._face_global_path(n,10.)
    n=S(requested_goal_xy=None,expected_goal=None,_zero=Mock())
    SlowMotionExecutor._request_turn_replan(n,10.)
    assert n.finished and n.completion_state=='RECOVERY_FAILED'


def test_reviewed_forward_curve_takes_priority_over_global_spin_request():
    n=node((-1,0))
    n.feasibility={'path_model':'forward_se2_curvature_lattice','candidate_review_ready':True}
    n.feasibility_rx=10.;n.path=n.global_path;n.path_rx=10.
    assert not SlowMotionExecutor._face_global_path(n,10.1)
    assert not n.path_turn_active
    n.feasibility_rx=8.
    assert SlowMotionExecutor._face_global_path(n,10.1)


def test_planned_rotation_prefix_is_executed_without_translation_permission():
    from builtin_interfaces.msg import Time
    n=node((-1,0));n.require_maneuver_review=True;n.requested_goal_xy=(-1.,0.)
    n.path=n.global_path;n.path.header=S(stamp=Time(sec=10));n.path_status_rx=10.
    n.path_status={'valid':True,'path_stamp_ns':10*10**9,'execution_review':{
      'execution_stage':'rotation','candidate_review_ready':True,'goal_xy':[-1.,0.],
      'stages':[{'origin_xy':[0.,0.],'target_yaw_rad':math.pi}]}}
    assert SlowMotionExecutor._face_global_path(n,10.1)
    assert n.path_turn_active and n.target_yaw==math.pi
    n=node((-1,0));n.require_maneuver_review=True
    assert not SlowMotionExecutor._face_global_path(n,10.1) # unreviewed global hint never spins
