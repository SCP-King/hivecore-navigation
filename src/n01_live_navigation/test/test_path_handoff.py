import json,time
from types import SimpleNamespace as S
from unittest.mock import Mock
import pytest
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from n01_live_navigation.path_pairing import PathReviewPairs
from n01_live_navigation.navigation_path import splice_path
from n01_live_navigation.navigation_path_node import NavigationPathNode


def path(points,ns):
    p=Path();p.header.frame_id='map';p.header.stamp.sec=10;p.header.stamp.nanosec=ns
    for x,y in points:
        q=PoseStamped();q.pose.position.x=float(x);q.pose.position.y=float(y);p.poses.append(q)
    return p


def review(ns):
    return {'candidate_stamp_ns':10**10+ns,'candidate_review_ready':True,'trajectory_active':True,
            'trajectory_start_time':{'sec':10,'nanosec':0},'trajectory_duration_s':60.,
            'validated_prefix_points':3,'trajectory_id':7,'recommended_speed_mps':.02}


def node():
    return S(reference=path([(0,0),(1,0),(1,.1),(0,.1),(2,.1)],0),local=None,
      feasibility=None,local_rx=0.,feas_rx=0.,pairs=PathReviewPairs(),reference_goal_stamp_ns=0,
      path_pub=Mock(),status_pub=Mock(),last_error=None,
      get_clock=lambda:S(now=lambda:S(nanoseconds=10**10,to_msg=lambda:path([],0).header.stamp)))


def test_reference_suffix_corner_does_not_invalidate_checked_local_prefix():
    ref=[(0,0),(1,0),(1,.1),(0,.1),(2,.1)];local=[(0,0),(.5,0),(1,0)]
    with pytest.raises(ValueError,match='heading'):splice_path(ref,local)
    merged,prefix=splice_path(ref,local,reference_tail_only=True)
    assert prefix==3 and merged[:3]==local and merged[-1]==ref[-1]


@pytest.mark.parametrize('first',['path','review'])
def test_arrival_order_keeps_previous_matched_version_without_using_new_unreviewed_path(first):
    n=node();p1=path([(0,0),(.5,0),(1,0)],0);p2=path([(0,0),(.4,.01),(1,.01)],100)
    NavigationPathNode.on_local(n,p1)
    NavigationPathNode.on_feasibility(n,String(data=json.dumps(review(0))))
    NavigationPathNode.tick(n)
    assert json.loads(n.status_pub.publish.call_args.args[0].data)['valid']
    if first=='path':NavigationPathNode.on_local(n,p2)
    else:NavigationPathNode.on_feasibility(n,String(data=json.dumps(review(100))))
    NavigationPathNode.tick(n)
    status=json.loads(n.status_pub.publish.call_args.args[0].data)
    assert status['valid'] and status['candidate_stamp_ns']==10**10
    assert status['execution_review']['candidate_stamp_ns']==10**10
    if first=='path':NavigationPathNode.on_feasibility(n,String(data=json.dumps(review(100))))
    else:NavigationPathNode.on_local(n,p2)
    NavigationPathNode.tick(n)
    status=json.loads(n.status_pub.publish.call_args.args[0].data)
    assert status['valid'] and status['candidate_stamp_ns']==10**10+100
    assert status['validated_prefix_points']==3 and status['suffix_requires_local_replanning']


def test_stale_pairs_and_explicit_failed_review_are_not_reused():
    pairs=PathReviewPairs();pairs.put_path(10**10,'path',1.);pairs.put_review(review(0),1.)
    assert pairs.newest(1.5)
    assert pairs.newest(1.7) is None
    n=node();NavigationPathNode.on_local(n,path([(0,0),(.5,0),(1,0)],0))
    NavigationPathNode.on_feasibility(n,String(data=json.dumps(review(0))))
    NavigationPathNode.on_feasibility(n,String(data=json.dumps({'candidate_review_ready':False})))
    NavigationPathNode.tick(n)
    assert not json.loads(n.status_pub.publish.call_args.args[0].data)['valid']
