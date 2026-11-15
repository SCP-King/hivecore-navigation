import unittest
from n01_live_navigation.feedback_continuity import SteeringRateGuard, bounded_pose_hold


class ContinuityTests(unittest.TestCase):
    def test_short_localization_gap_uses_only_bounded_live_odometry(self):
        def check(rx=1.12,stamp=.99,status=True,odom=True):
            return bounded_pose_hold(rx,stamp,.7,1.5,1.25,status,odom)
        self.assertTrue(check())
        self.assertFalse(check(odom=False))
        self.assertFalse(check(status=False))
        self.assertFalse(check(rx=1.26))
        self.assertFalse(check(stamp=1.26))
        self.assertFalse(check(stamp=-.3))

    def test_spike_does_not_reset_motion_but_sustained_overspeed_stops(self):
        g=SteeringRateGuard()
        self.assertTrue(g.update([1.6]*4,0.))
        self.assertTrue(g.update([1.2]*4,.1))
        self.assertTrue(g.update([1.6]*4,.2))
        self.assertFalse(g.update([1.6]*4,.4))
        self.assertFalse(g.update([1.55]*4,.5))
        self.assertFalse(g.update([1.2]*4,.6))
        self.assertTrue(g.update([1.2]*4,.81))

    def test_hard_overspeed_is_immediate(self):
        self.assertFalse(SteeringRateGuard().update([0.,0.,3.15,0.],0.))


if __name__=='__main__':unittest.main()
