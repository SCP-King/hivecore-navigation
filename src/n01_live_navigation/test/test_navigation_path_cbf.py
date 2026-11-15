import math
import unittest
from types import SimpleNamespace
from n01_live_navigation.navigation_path import splice_path, execution_prefix
from n01_live_navigation.cbf_velocity import project_speed


class UnifiedPathTests(unittest.TestCase):
    def test_near_goal_sampling_noise_does_not_create_heading_rejection(self):
        reference=[(0.,0.),(.99,0.),(1.,0.)]
        local=[(0.,0.),(.95,.01),(.999,.0006)]
        merged,prefix=splice_path(reference,local)
        self.assertEqual(merged[-1],reference[-1])
        self.assertEqual(prefix,len(local))

    def test_unscreened_local_suffix_is_retained_but_not_marked_executable(self):
        import json,time
        from unittest.mock import Mock
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        from n01_live_navigation.navigation_path_node import NavigationPathNode
        def path(count):
            msg=Path();msg.header.frame_id='map'
            for i in range(count):
                p=PoseStamped();p.pose.position.x=float(i);msg.poses.append(p)
            return msg
        fake=SimpleNamespace(reference=path(6),local=path(4),local_rx=time.monotonic(),
            feas_rx=time.monotonic(),reference_goal_stamp_ns=0,
            feasibility={'candidate_review_ready':True,'candidate_stamp_ns':0,'trajectory_start_time':{'sec':10,'nanosec':0},
                         'trajectory_duration_s':60.,'trajectory_id':1,'validated_prefix_points':3},
            path_pub=Mock(),status_pub=Mock(),last_error=None,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000,
                                                      to_msg=lambda:path(1).header.stamp)))
        NavigationPathNode.tick(fake)
        status=json.loads(fake.status_pub.publish.call_args.args[0].data)
        self.assertTrue(status['valid'])
        self.assertEqual(status['validated_prefix_points'],3)
        self.assertEqual(status['points'],6)
        self.assertEqual(len(fake.path_pub.publish.call_args.args[0].poses),6)

    def test_preview_moves_with_robot_and_reference_goal_is_preserved(self):
        reference = [(i*.01,0.) for i in range(401)]
        local = execution_prefix(reference, (1.,0.), 1.)
        self.assertEqual(local[0], (1.,0.))
        self.assertLessEqual(local[-1][0],2.)
        merged, prefix = splice_path(reference,local)
        self.assertEqual(merged[-1], (4.,0.))
        self.assertEqual(prefix,len(local))

    def test_preview_rejects_disconnected_candidate(self):
        with self.assertRaisesRegex(ValueError,'too_far'):
            execution_prefix([(0.,0.),(.1,0.),(.2,0.)], (2.,0.))

    def test_reference_refresh_does_not_invalidate_same_goal_epoch(self):
        from n01_live_navigation.navigation_path_node import NavigationPathNode
        def path(stamp,x):
            return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp,nanosec=0)),
                poses=[SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x,y=0.,z=0.)))])
        fake=SimpleNamespace(reference=None,reference_goal_stamp_ns=0)
        NavigationPathNode.on_reference(fake,path(10,3.))
        self.assertEqual(fake.reference_goal_stamp_ns,10_000_000_000)
        NavigationPathNode.on_reference(fake,path(11,3.))
        self.assertEqual(fake.reference_goal_stamp_ns,10_000_000_000)
        NavigationPathNode.on_reference(fake,path(12,4.))
        self.assertEqual(fake.reference_goal_stamp_ns,12_000_000_000)

    def test_local_replaces_prefix_and_preserves_final_goal(self):
        reference = [(0.,0.),(1.,0.),(2.,0.),(3.,0.)]
        local = [(0.,0.),(.5,.1),(1.,0.)]
        merged, prefix = splice_path(reference,local)
        self.assertEqual(merged,local+reference[2:])
        self.assertEqual(prefix,3)
        self.assertEqual(merged[-1],reference[-1])

    def test_gap_and_reverse_join_rejected(self):
        with self.assertRaises(ValueError):
            splice_path([(0.,0.),(1.,0.)],[(0.,0.),(3.,0.)])
        with self.assertRaises(ValueError):
            splice_path([(0.,0.),(1.,0.),(2.,0.)],[(0.,0.),(1.1,0.),(1.,0.)])

    def test_no_global_tail_beyond_goal(self):
        path, count = splice_path([(0.,0.),(1.,0.)],[(.2,0.),(1.,0.)])
        self.assertEqual(len(path),count)


class CbfTests(unittest.TestCase):
    def test_clear_and_receding(self):
        self.assertEqual(project_speed((0,0),0,.05,[])[0],.05)
        self.assertEqual(project_speed((0,0),0,.05,[(1,0,.5,0,0)])[0],.05)

    def test_overlap_incoming_and_bad_data_stop(self):
        for obs in [[(.4,0,0,0,0)],[(1,0,-.5,0,0)],[(0,1,0,-.5,0)],[(math.nan,0,0,0,0)]]:
            self.assertEqual(project_speed((0,0),0,.05,obs)[0],0)

    def test_qp_obeys_each_barrier(self):
        obstacles=[(.6,0,0,0,0),(1,.2,-.1,0,0)]
        v,_ = project_speed((0,0),0,.1,obstacles)
        self.assertGreaterEqual(v,0)
        self.assertLessEqual(v,.1)
        for ox,oy,vx,vy,radius in obstacles:
            h=ox*ox+oy*oy-(.55+radius)**2
            self.assertGreaterEqual(2*((-ox)*(v-vx)+(-oy)*(-vy))+h,-1e-9)


if __name__=='__main__':
    unittest.main()
