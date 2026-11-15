import unittest
import numpy as np
from n01_live_navigation.reduced_tracking import reduced_yaw_rate, translation_velocity, prefix_requires_extension
from n01_live_navigation.core import four_wheel_steer_screen
from n01_live_navigation.hive2_kinematics import inverse


class ReducedTrackingTests(unittest.TestCase):
    def test_reverse_translation_uses_native_reverse_drive(self):
        import math
        vx,vy=translation_velocity(.03,math.pi)
        targets,drive=inverse((vx,vy,0.),[0.]*4)
        np.testing.assert_allclose(targets,0.,atol=1e-10)
        self.assertTrue((drive<0.).all())
    def test_goal_inside_validated_prefix_does_not_stop_twenty_cm_early(self):
        self.assertFalse(prefix_requires_extension((.8,0.),(.99,0.),(1.,0.),True))
        self.assertTrue(prefix_requires_extension((.8,0.),(.99,0.),(2.,0.),True))
        self.assertFalse(prefix_requires_extension((.1,0.),(.99,0.),(2.,0.),True))
    def test_translation_body_frame_and_native_wheel_direction(self):
        for angle in np.linspace(-1.4,1.4,25):
            vx,vy=translation_velocity(.03,float(angle))
            self.assertAlmostEqual(np.hypot(vx,vy),.03)
            target,_=inverse((vx,vy,0.),[0.]*4)
            np.testing.assert_allclose(target,[angle]*4,atol=1e-10)

    def test_curved_path_fixed_heading_has_zero_body_yaw_rate(self):
        x=np.linspace(0.,3.,501);p=np.column_stack((x,.3*np.sin(np.pi*x/3.),np.zeros_like(x)))
        r=four_wheel_steer_screen(p,body_yaw=.2)
        self.assertTrue(r['necessary_conditions_pass'])
        self.assertEqual(r['max_abs_yaw_rate_radps'],0.)
        self.assertEqual(r['tracking_model'],'fixed_heading_translation')

    def test_startup_never_requests_spin(self):
        for v in (0.,.002,.004,.008):
            self.assertEqual(reduced_yaw_rate(v,.8,.55,.04),0.)

    def test_curvature_bounded_through_ramp(self):
        for v in np.linspace(.01,.03,30):
            w=reduced_yaw_rate(float(v),.8,.55,.04,max_curvature=1.)
            self.assertLessEqual(abs(w),v+1e-12)
            target,drive=inverse((v,0.,w),[0.]*4)
            self.assertLess(np.max(np.abs(target)),np.pi/2)
            self.assertLess(np.max(np.abs(drive)),20.)

    def test_sign_and_cbf_reduction(self):
        self.assertLess(reduced_yaw_rate(.03,-.1,.55,.04),0.)
        self.assertEqual(reduced_yaw_rate(.003,.1,.55,.04),0.)

    def test_invalid_input(self):
        with self.assertRaises(ValueError):reduced_yaw_rate(float('nan'),0.,1.,.04)

if __name__=='__main__':unittest.main()


def test_default_tracking_does_not_impose_one_metre_radius():
    w=reduced_yaw_rate(.03,.4,.55,.08)
    assert w/.03>1. and abs(w)<=.08


def test_heading_slowdown_has_no_dead_zone_before_alignment():
    from n01_live_navigation.reduced_tracking import forward_speed_factor
    assert forward_speed_factor(0.)==1.
    for error in (.79,.8,.9,1.):
        assert 0.<forward_speed_factor(error)<=1.
        assert forward_speed_factor(error)==forward_speed_factor(-error)
    assert forward_speed_factor(np.pi)==0.
