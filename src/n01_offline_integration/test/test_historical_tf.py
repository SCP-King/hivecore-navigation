import math
import pathlib
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from n01_offline_integration.historical_tf import (
    HistoricalMapTf, interpolate, combine, mapped_mounts, rotate,
)


class HistoricalBagTf(unittest.TestCase):
    def test_interpolation_and_stale_rejection(self):
        tf=HistoricalMapTf(max_age_s=.8,max_interval_s=2)
        self.assertTrue(tf.add(1_000_000_000,(0.,0.,0.),(0.,0.,0.,1.)))
        self.assertTrue(tf.add(2_000_000_000,(2.,0.,0.),(0.,0.,0.,1.)))
        tf.finish()
        result=tf.lookup(1_500_000_000)
        self.assertAlmostEqual(result[0][0],1.)
        self.assertEqual(result[2],0)
        self.assertIsNone(tf.lookup(10_000_000_000))

    def test_selected_replay_geometry(self):
        body,lidar=mapped_mounts(((3.,4.,0.),(0.,0.,0.,1.)))
        self.assertEqual(body[0],(3.,4.,.9))
        self.assertAlmostEqual(lidar[0][0],3.295)
        self.assertAlmostEqual(lidar[0][2],.9)

    def test_rotation_composition(self):
        q=(0,0,math.sqrt(.5),math.sqrt(.5))
        result=combine((1.,2.,0.),q,(.295,0.,0.))
        self.assertAlmostEqual(result[0][0],1.,places=5)
        self.assertAlmostEqual(result[0][1],2.295,places=5)

    def test_do_not_infer_missing_map_tf(self):
        tf=HistoricalMapTf()
        tf.finish()
        self.assertIsNone(tf.lookup(12345678))

    def test_reject_invalid_quaternion(self):
        tf=HistoricalMapTf()
        self.assertFalse(tf.add(100,(0,0,0),(0,0,0,0)))
        self.assertFalse(tf.add(0,(0,0,0),(0,0,0,1)))
        tf.finish()
        self.assertIsNone(tf.lookup(100))

    def test_timestamp_query_outside_range(self):
        tf=HistoricalMapTf(max_age_s=.5)
        tf.add(10_000_000_000,(2.,2.,0.),(0.,0.,0.,1.))
        tf.finish()
        self.assertIsNone(tf.lookup(8_000_000_000))
        self.assertEqual(tf.lookup(9_800_000_000)[0],(2.,2.,0.))


if __name__=="__main__":
    unittest.main()
