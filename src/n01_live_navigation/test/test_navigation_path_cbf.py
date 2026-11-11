import math
import unittest
from n01_live_navigation.navigation_path import splice_path
from n01_live_navigation.cbf_velocity import project_speed


class UnifiedPathTests(unittest.TestCase):
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
