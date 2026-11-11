"""Offline tests against the installed chassis_controller geometry library."""
import math
import unittest
from unittest.mock import patch
import numpy as np
from n01_live_navigation.hive2_kinematics import (
    Hive2Geometry, contact_points, forward, inverse, _native)
from n01_live_navigation.core import four_wheel_steer_screen


class Hive2KinematicsTests(unittest.TestCase):
    def test_zero_steer_contact_is_controller_not_urdf_proxy(self):
        np.testing.assert_allclose(contact_points([0.]*4),
            [[.28,.220],[.28,-.220],[-.28,.220],[-.28,-.220]])

    def test_straight_and_crab(self):
        for twist, angle in [((.1,0.,0.),0.),((0.,.1,0.),math.pi/2)]:
            target,drive=inverse(twist,[angle]*4)
            np.testing.assert_allclose(target,[angle]*4,atol=1e-12)
            np.testing.assert_allclose(drive,[.1/.095]*4,atol=1e-12)
            actual,residual=forward(target,[0.]*4,drive)
            np.testing.assert_allclose(actual,twist,atol=1e-12)
            self.assertLess(residual,1e-12)

    def test_reverse_has_signed_drive(self):
        target,drive=inverse((-.1,0.,0.),[0.]*4)
        np.testing.assert_allclose(target,[0.]*4,atol=1e-12)
        self.assertTrue((drive<0).all())

    def test_turning_unaligned_is_not_falsely_exact_fk(self):
        _,drive=inverse((0.,0.,1.),[0.]*4)
        np.testing.assert_allclose(drive,[-.220/.095,.220/.095,-.220/.095,.220/.095])
        actual,residual=forward([0.]*4,[0.]*4,drive)
        self.assertGreater(residual,.1)
        self.assertLess(actual[2],1.)  # lateral no-slip constraints are incompatible

    def test_steering_compensation(self):
        _,drive=inverse((0.,0.,0.),[0.]*4,[1.]*4)
        np.testing.assert_allclose(drive,np.array([-1.,1.,-1.,1.])*.085/.095)
        actual,residual=forward([0.]*4,[1.]*4,drive)
        np.testing.assert_allclose(actual,0.,atol=1e-12)
        self.assertLess(residual,1e-12)

    def test_random_native_projection_and_eight_row_fk(self):
        rng=np.random.default_rng(20261111)
        for _ in range(100):
            q=rng.uniform(-1.5,1.5,4); qd=rng.uniform(-1.,1.,4)
            v=rng.uniform(-.3,.3,3); c=contact_points(q)
            t=np.column_stack((np.cos(q),np.sin(q)))
            side=np.array([1.,-1.,1.,-1.])
            b=-c[:,1]*t[:,0]+c[:,0]*t[:,1]
            rows=np.column_stack((t,b))
            expected=(rows@v-side*.085*qd)/.095
            _,drive=inverse(v,q,qd)
            np.testing.assert_allclose(drive,expected,atol=1e-12)
            lateral=np.column_stack((-np.sin(q),np.cos(q),
                c[:,1]*np.sin(q)+c[:,0]*np.cos(q)))
            A=np.vstack((rows,lateral)); rhs=np.r_[.095*drive+side*.085*qd,np.zeros(4)]
            reference=np.linalg.lstsq(A,rhs,rcond=None)[0]
            actual,_=forward(q,qd,drive)
            np.testing.assert_allclose(actual,reference,atol=1e-10)

    def test_native_branch_continuity(self):
        low,_=inverse((0.,0.,1.),[0.]*4)
        high,_=inverse((0.,0.,1.),[math.pi/2]*4)
        self.assertLess(abs(low[0]),math.pi/2)
        self.assertGreater(high[0],math.pi/2)  # native preserves actual branch, no forced folding

    def test_invalid_input_rejected(self):
        for twist in ((math.nan,0,0),(0,0),(math.inf,0,0)):
            with self.assertRaises(ValueError): inverse(twist,[0.]*4)
        with self.assertRaises(ValueError): Hive2Geometry(delta_y=-1.).array()

    def test_missing_adapter_fails_closed(self):
        _native.cache_clear()
        try:
            with patch('n01_live_navigation.hive2_kinematics.Path.glob',return_value=[]):
                with self.assertRaisesRegex(ValueError,'adapter_missing'):
                    inverse((.1,0,0),[0.]*4)
        finally: _native.cache_clear()

    def test_screen_exposes_limit_conflict_and_actual_contact(self):
        pts=np.column_stack((np.linspace(0,2,40),np.zeros((40,2))))
        result=four_wheel_steer_screen(pts)
        self.assertTrue(result['necessary_conditions_pass'])
        self.assertTrue(result['controller_model_verified'])
        self.assertTrue(result['steering_limit_conflict'])
        self.assertFalse(result['wheel_contact_geometry_verified'])
        self.assertFalse(result['execution_authorized'])
        self.assertAlmostEqual(result['wheel_positions_m']['fl'][1],.220)

    def test_conflicting_navigation_geometry_is_rejected(self):
        pts=np.column_stack((np.linspace(0,2,40),np.zeros((40,2))))
        with self.assertRaisesRegex(ValueError,'mismatches_installed'):
            four_wheel_steer_screen(pts,track_y=.47)


if __name__=='__main__': unittest.main()
