import json,math,time
from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from n01_live_navigation import maneuver_review as m


def fixture(target):
    now=time.monotonic();ns=100_000_000_000
    path=Path();path.header.frame_id='map'
    for f in np.linspace(0,1,24):
        p=PoseStamped();p.pose.position.x=float(target[0]*f);p.pose.position.y=float(target[1]*f);path.poses.append(p)
    body=Odometry();body.header.frame_id='map';body.pose.pose.position.z=.6;body.pose.pose.orientation.w=1.
    return S(global_reference=path,accepted_goal_xy=target,accepted_goal_ns=ns,global_reference_rx=now,
        get_clock=lambda:S(now=lambda:S(nanoseconds=ns,to_msg=lambda:Time(sec=100))),
        body_pose=body,body_pose_stamp_ns=ns,body_pose_rx=now,
        obstacles=np.empty((0,3)),obstacles_rx=now,obstacles_stamp_ns=ns,ground_plane=[0,0,0],plane_rx=now,
        map_window=[[-4,-4],[4,4]],measured_twist=(0.,0.),odom_rx=now,joint_rx=now,joint_stamp_ns=ns,
        _pitch_reference_match=lambda:True,_yaw_from_quaternion=lambda q:0.,
        footprint_client=S(get=lambda:np.array([[-.4,-.15],[.4,-.15],[.4,.15],[-.4,.15]]),description=lambda:{}),
        joint_positions={f'{leg}_ankle_yaw':0. for leg in ['fl','fr','rl','rr']},
        native_model_sync=S(geometry=lambda:S(controller_steering_limit=math.pi)),
        execution_preview=1.,max_speed=.03,max_accel=.04,path_pub=Mock(),diag_pub=Mock(),_clear_candidate_visualization=Mock())


def test_rotation_plan_authorizes_only_stationary_prefix(monkeypatch):
    n=fixture((-1.,0.))
    monkeypatch.setattr(m,'live_points',lambda n:np.empty((0,3)))
    monkeypatch.setattr(m,'inverse',lambda *a,**k:(np.zeros(4),np.zeros(4)))
    assert m.publish_guide_maneuver(n)
    review=json.loads(n.diag_pub.publish.call_args.args[0].data);path=n.path_pub.publish.call_args.args[0]
    assert review['candidate_review_ready'] and review['execution_stage']=='rotation'
    prefix=review['validated_prefix_points'];assert 3<=prefix<len(path.poses)
    assert all(p.pose.position.x==0. and p.pose.position.y==0. for p in path.poses[:prefix])
    assert review['stages'][1]['requires_measured_heading']
    assert path.poses[-1].pose.position.x<-.9


def test_stale_scene_clears_maneuver_instead_of_replaying_old_permission():
    n=fixture((-1.,0.));n.obstacles_rx-=2.
    assert m.publish_guide_maneuver(n)
    n.path_pub.publish.assert_not_called();n._clear_candidate_visualization.assert_called_once()
    assert not json.loads(n.diag_pub.publish.call_args.args[0].data)['candidate_review_ready']
