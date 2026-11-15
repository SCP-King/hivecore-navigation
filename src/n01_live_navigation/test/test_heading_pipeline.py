"""Offline interface checks: sensor freshness, native IK and output binding."""
import json
import math
import time
from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
import pytest
from n01_live_navigation.trajectory_monitor import Hive2TrajectoryMonitor
from n01_live_navigation.heading_search import FootprintCollision, search
from n01_live_navigation.core import four_wheel_steer_screen
from n01_live_navigation.navigation_path_node import NavigationPathNode


def test_search_detour_passes_native_four_wheel_screen():
    c=FootprintCollision([[1.5,0,.5]],[0,0,0],.775,[[-4,-4],[4,4]])
    r=search((0,0,0,0),(3,0),c,budget_s=2.)
    assert r.states, r.reason
    assert c.free(r.states)
    assert max(abs(s[1]) for s in r.states)>.3
    points=[(s[0],s[1],.775) for s in r.states]
    f=four_wheel_steer_screen(points,max_speed=.05,max_accel=.04,
                              initial_steering=[0]*4,allow_retime=True)
    assert f['necessary_conditions_pass'], f['kinematic_failures']


def monitor():
    now=time.monotonic()
    n=S(obstacles=np.empty((0,3)),obstacles_rx=now,obstacles_stamp_ns=10**10,
        ground_plane=[0,0,0],plane_rx=now,map_window=[[-4,-4],[4,4]],
        measured_twist=(0.,0.),odom_rx=now,frame='map',body_pose_rx=now,body_pose_stamp_ns=10**10,
        body_pose=S(header=S(frame_id='map'),pose=S(pose=S(position=S(x=0.,y=0.,z=.775),
        orientation=S(x=0.,y=0.,z=0.,w=1.)))),
        get_clock=lambda:S(now=lambda:S(nanoseconds=10**10)),
        _yaw_from_quaternion=Hive2TrajectoryMonitor._yaw_from_quaternion)
    return n


def test_pipeline_uses_measured_heading_and_rejects_stale_perception():
    guide=[(x,0,.775) for x in np.linspace(0,1,100)]
    n=monitor();points,r=Hive2TrajectoryMonitor.heading_candidate(n,guide)
    assert len(points)>3 and r.states[0][2]==0
    for field in ('obstacles_rx','plane_rx','odom_rx','body_pose_rx'):
        n=monitor();setattr(n,field,0.)
        with pytest.raises(ValueError,match='fresh'):
            Hive2TrajectoryMonitor.heading_candidate(n,guide)
    n=monitor();n.measured_twist=(0.,.08)
    with pytest.raises(ValueError,match='rotation_to_stop'):
        Hive2TrajectoryMonitor.heading_candidate(n,guide)


def test_mismatched_candidate_and_review_never_publish_valid_unified_path():
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    p=Path();p.header.frame_id='map';p.header.stamp.sec=10
    p.poses=[PoseStamped(),PoseStamped()]
    n=S(reference=p,local=p,local_rx=time.monotonic(),feas_rx=time.monotonic(),
        feasibility={'candidate_review_ready':True,'candidate_stamp_ns':9*10**9},
        status_pub=Mock(),path_pub=Mock(),last_error=None,
        get_clock=lambda:S(now=lambda:S(to_msg=lambda:p.header.stamp)))
    NavigationPathNode.tick(n)
    status=json.loads(n.status_pub.publish.call_args.args[0].data)
    assert not status['valid'] and 'matching_candidate' in status['reason']


def test_cached_candidate_rechecks_scene_and_expires_without_renewing(monkeypatch):
    import n01_live_navigation.trajectory_monitor as monitor_module
    from n01_live_navigation.heading_search import SearchResult
    guide=[(x,0,.775) for x in np.linspace(0,1,100)]
    n=monitor()
    _,first=Hive2TrajectoryMonitor.heading_candidate(n,guide)
    created=n.heading_cache['created']
    failed=Mock(return_value=SearchResult([], 'injected_search_timeout', 0, 120.))
    monkeypatch.setattr(monitor_module,'search',failed)
    _,reused=Hive2TrajectoryMonitor.heading_candidate(n,guide)
    assert reused.reason=='revalidated_previous_heading_path'
    assert n.heading_cache['created']==created
    failed.assert_not_called()
    n.obstacles=np.array([[.8,0,.5]])
    with pytest.raises(ValueError,match='injected_search_timeout'):
        Hive2TrajectoryMonitor.heading_candidate(n,guide)
    assert n.heading_cache is None


def test_cache_does_not_authorize_changed_goal_or_expired_lifetime(monkeypatch):
    import n01_live_navigation.trajectory_monitor as module
    from n01_live_navigation.heading_search import SearchResult
    guide=[(x,0,.775) for x in np.linspace(0,1,100)]
    n=monitor();Hive2TrajectoryMonitor.heading_candidate(n,guide)
    cache=n.heading_cache.copy()
    monkeypatch.setattr(module,'search',lambda *a,**kw:SearchResult([], 'search_failed', 0, 1.))
    changed=[(x,0,.775) for x in np.linspace(0,1.3,100)]
    with pytest.raises(ValueError):Hive2TrajectoryMonitor.heading_candidate(n,changed)
    n.heading_cache=cache;n.heading_cache['created']-=3
    with pytest.raises(ValueError):Hive2TrajectoryMonitor.heading_candidate(n,guide)


def test_new_accepted_goal_invalidates_old_spline_and_cached_search():
    from geometry_msgs.msg import PoseStamped
    n=S(frame='map',accepted_goal_ns=1,last_bspline='old',last_bspline_rx=10.,
        last_trajectory_window=(1,60),heading_cache={'states':[]},heading_search_failure={'reason':'old'},
        _clear_candidate_visualization=Mock())
    m=PoseStamped();m.header.frame_id='map';m.header.stamp.sec=10
    Hive2TrajectoryMonitor.on_goal(n,m)
    assert n.accepted_goal_ns==10**10 and n.last_bspline is None and n.heading_cache is None
    assert n.last_trajectory_window is None and n.heading_search_failure is None
    n._clear_candidate_visualization.assert_called_once()
