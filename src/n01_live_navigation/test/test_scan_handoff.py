"""Native SCAN admission: no second planner and no actuator output."""
import json
import math
import time
from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time
from scan_planner_msgs.msg import Bspline
from n01_scan_dynacbf_bridge.curve import BSplineCurve
from n01_live_navigation.trajectory_monitor import Hive2TrajectoryMonitor as Monitor
from n01_live_navigation.scan_handoff import SplineEvents, sample_native_prefix


def spline(id=1, sec=10):
    return Bspline(order=3, traj_id=id, goal_id=9*10**9, start_time=Time(sec=sec),
        knots=[0.,0.,0.,0.,20.,20.,20.,20.],
        pos_pts=[Point(x=x,y=y,z=.775) for x,y in [(0.,0.),(.4,.1),(.8,.1),(1.2,0.)]])


def monitor():
    from n01_live_navigation.hive2_kinematics import Hive2Geometry
    now=time.monotonic()
    stamp=Time(sec=10)
    n=S(footprint_client=S(get=lambda:np.array([[-.5,-.2],[.5,-.2],[.5,.2],[-.5,.2]]),description=lambda:{}),
        native_model_sync=S(geometry=lambda:Hive2Geometry(),limits=lambda:{"chassis.steer.max_speed":1.57,"chassis.drive.max_speed":20.}),max_speed=.03,max_accel=.04,
        wheel_base_x=.56,track_y=.27,wheel_radius=.095,delta_y=.085,steering_limit=math.pi,
        execution_preview=1.,spline_events=SplineEvents(),accepted_goal_ns=9*10**9,accepted_goal_xy=(1.2,0.),accepted_goal_yaw=0.,map_id="synthetic",trajectory_pub=Mock(),rotation_raw=[],
        last_bspline=None,last_curve=None,last_bspline_rx=None,last_trajectory_window=None,
        joint_positions={f'{leg}_ankle_yaw':0. for leg in ('fl','fr','rl','rr')},
        joint_rx=now,joint_stamp_ns=10**10,
        obstacles=np.empty((0,3)),obstacles_rx=now,obstacles_stamp_ns=10**10,
        ground_plane=[0,0,0],plane_rx=now,map_window=[[-4,-4],[4,4]],
        frame='map',body_pose_rx=now,body_pose_stamp_ns=10**10,
        body_pose=S(header=S(frame_id='map'),pose=S(pose=S(position=S(x=0.,y=0.,z=.775),
        orientation=S(x=0.,y=0.,z=0.,w=1.)))),
        get_clock=lambda:S(now=lambda:S(nanoseconds=10**10,to_msg=lambda:stamp)),
        path_pub=Mock(),diag_pub=Mock(),publish_trajectory=Mock(),_publish_wheel_markers=Mock(),
        _yaw_from_quaternion=Monitor._yaw_from_quaternion,
        _clear_candidate_visualization=Mock(),_pitch_reference_match=lambda:True,get_logger=lambda:Mock())
    n.scene_inputs_ready=lambda:Monitor.scene_inputs_ready(n)
    n.check_native_scene=lambda states:Monitor.check_native_scene(n,states)
    n.on_bspline=lambda msg,replay=False:Monitor.on_bspline(n,msg,replay)
    return n


def review(n):
    return json.loads(n.diag_pub.publish.call_args.args[0].data)


def test_native_curve_is_not_suppressed_by_global_reference_or_searched_again():
    n=monitor();p=PoseStamped();p.pose.position.x=1.2
    n.global_reference=S(poses=[p])  # Previously caused unconditional early return.
    m=spline();n.on_bspline(m)
    d=review(n)
    assert d['candidate_review_ready'],d
    assert d['path_model']=='scan_bspline' and d['geometry_preserved']
    assert d['goal_stamp_ns']==9*10**9 and d['goal_xy']==[1.2,0.]
    curve=BSplineCurve(3,m.knots,[(p.x,p.y,p.z) for p in m.pos_pts],table_samples=500)
    expected=sample_native_prefix(curve,0.,0.,1.)
    actual=n.path_pub.publish.call_args.args[0].poses
    assert np.allclose([(p.pose.position.x,p.pose.position.y,p.pose.position.z) for p in actual],
                       [p[:3] for p in expected])
    assert d['trajectory_start_time']=={'sec':10,'nanosec':0}
    assert not hasattr(Monitor,'heading_candidate')


def test_scene_review_rejects_collision_and_stale_perception():
    for field in ('obstacles_rx','plane_rx'):
        n=monitor();setattr(n,field,0.);n.on_bspline(spline())
        assert not review(n)['candidate_review_ready']
        assert 'fresh' in review(n)['reason']
    n=monitor();n.obstacles=np.array([[.8,0.,.5]]);n.on_bspline(spline())
    assert 'collision' in review(n)['reason']
    assert not review(n)['candidate_review_ready']


def test_duplicate_delivery_preserves_source_event_and_receive_clock():
    n=monitor();m=spline();n.on_bspline(m)
    rx=n.last_bspline_rx;curve=n.last_curve
    n.on_bspline(spline())
    assert n.last_curve is curve and n.last_bspline_rx==rx
    assert n.last_trajectory_window==(10**10,20.)
    assert review(n)['candidate_review_ready']


def test_reordered_conflicting_and_malformed_events_cannot_replace_current():
    n=monitor();n.on_bspline(spline(2));n.diag_pub.publish.reset_mock()
    for m in (spline(1),spline(2,11),spline(3,8)):
        n.on_bspline(m)
        assert n.last_bspline.traj_id==2
    n.diag_pub.publish.assert_not_called()
    broken=spline(3);broken.knots=[0.];n.on_bspline(broken)
    assert n.last_bspline.traj_id==2
    n._clear_candidate_visualization.assert_not_called()


def test_replay_uses_execution_clock_without_rewriting_original_window():
    n=monitor();n.on_bspline(spline());n.get_clock=lambda:S(now=lambda:S(nanoseconds=31*10**9))
    n.on_bspline(n.last_bspline,replay=True)
    assert n.last_trajectory_window==(10**10,20.)
    assert 'expired' not in review(n).get('reason','')  # Fresh state still required independently.


def test_new_accepted_goal_invalidates_event_epoch_and_duplicate_goal_is_inert():
    n=monitor();n.on_bspline(spline());m=PoseStamped();m.header.frame_id='map';m.header.stamp.sec=11
    Monitor.on_goal(n,m)
    assert n.last_bspline is None and n.last_curve is None and n.spline_events.latest is None
    assert n.last_trajectory_window is None
    n._clear_candidate_visualization.reset_mock();Monitor.on_goal(n,m)
    n._clear_candidate_visualization.assert_not_called()
    n.on_bspline(spline(4,10));assert n.last_bspline is None
    n.get_clock=lambda:S(now=lambda:S(nanoseconds=11*10**9,to_msg=lambda:Time(sec=11)))
    n.obstacles_stamp_ns=11*10**9
    m=spline(1,11);m.goal_id=11*10**9;n.on_bspline(m)  # Explicit identity binds a restarted counter.
    assert n.last_bspline.traj_id==1


def test_no_goal_cannot_admit_unbound_event():
    gate=SplineEvents()
    with pytest.raises(ValueError,match='missing_accepted_goal'):gate.classify(spline(),0)

def test_one_shot_source_waits_for_feedback_without_losing_the_event():
    n=monitor();pose=n.body_pose;n.body_pose=None
    n.on_bspline(spline())
    assert not review(n)['candidate_review_ready']
    assert n.last_bspline is not None
    n.body_pose=pose;n.on_bspline(n.last_bspline,replay=True)
    assert review(n)['candidate_review_ready']


def test_typed_scene_is_packed_without_per_point_message_objects():
    n=monitor();n.obstacles=np.array([[1.,.2,.4],[1.,.2,.8]])
    Monitor.publish_trajectory(n,spline(),None,[(0.,0.,.775,0.)],{'recommended_speed_mps':.02},True)
    packet=n.trajectory_pub.publish.call_args.args[0]
    assert packet.scene_valid and packet.goal_id==9*10**9 and packet.map_id=='synthetic' and packet.spline.goal_id==packet.goal_id
    assert packet.max_speed==.02 and packet.max_wheel_rate==20.
    assert packet.obstacles.header.frame_id=='map' and packet.obstacles.point_step==12
    assert packet.obstacles.width==1 # identical XY collision columns deduplicate exactly
    xyz=np.frombuffer(packet.obstacles.data,dtype='<f4').reshape(-1,3)
    assert np.allclose(xyz[0,:2],[1.,.2])
