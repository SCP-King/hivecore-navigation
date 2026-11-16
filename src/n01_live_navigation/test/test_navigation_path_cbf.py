import math
import unittest
from types import SimpleNamespace
from n01_live_navigation.cbf_velocity import project_speed


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
