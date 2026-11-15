import unittest
import numpy as np
from n01_live_navigation.cbf_velocity import point_obstacles,project_speed


class PointSupportTests(unittest.TestCase):
    def test_unknown_velocity_does_not_invent_solid_disk(self):
        points=np.array([[1.,0.,.5],[1.,.1,.5]])
        obs=point_obstacles(points,[{'centroid_map':[1.,0.,.5],'velocity_valid':False}])
        self.assertEqual(len(obs),2)
        self.assertTrue(all(o[4]==0. for o in obs))
        self.assertGreater(project_speed((0.,0.),0.,.03,obs)[0],0.)

    def test_valid_velocity_assigned_only_to_measured_support(self):
        points=np.array([[1.,0.,.5],[2.,0.,.5]])
        track={'velocity_valid':True,'minimum_map':[.9,-.1,.4],
               'maximum_map':[1.1,.1,.6],'velocity_map_mps':[-1.,0.,0.]}
        obs=point_obstacles(points,[track])
        self.assertEqual(obs[0][2],-1.)
        self.assertEqual(obs[1][2],0.)
        self.assertEqual(project_speed((0.,0.),0.,.03,obs)[0],0.)

    def test_missing_moving_support_fails_closed(self):
        with self.assertRaises(ValueError):
            point_obstacles(np.array([[1.,0.,.5]]),[{'velocity_valid':True}])

    def test_real_overlap_remains_stop(self):
        obs=point_obstacles(np.array([[.3,0.,.5]]),[])
        self.assertEqual(project_speed((0.,0.),0.,.03,obs)[0],0.)
