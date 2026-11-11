"""Non-actuating adapter to the SAME Hive2 kinematics library as chassis_controller.

This covers geometric IK/FK, not the controller's OTG, current unloading or safety FSM.
Missing SDK/adapter is an explicit failure; there is no proxy-model fallback.
"""
import ctypes
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import numpy as np
import yaml

WHEELS = ('fl', 'fr', 'rl', 'rr')
MODEL_ID = 'hive2-motion-1.0.0/ChassisKinematics'
SDK_ROOT = Path('/opt/hivecore/hive2-motion/1.0.0/install')


@dataclass(frozen=True)
class Hive2Geometry:
    wheel_radius: float = .095
    wheel_base_x: float = .56
    wheel_base_y: float = .27
    delta_y: float = .085
    max_linear_speed: float = 2.
    max_angular_speed: float = 2.5
    spin_threshold: float = .01
    controller_steering_limit: float = np.pi
    near_zero_bias: float = 1.

    def array(self):
        a = np.array(list(self.__dict__.values()), dtype=np.float64)
        if not np.isfinite(a).all() or np.any(a[:3] <= 0) or a[3] < 0 or np.any(a[4:8] <= 0) or a[8] < 0:
            raise ValueError('invalid_hive2_geometry')
        return a


@lru_cache(maxsize=1)
def controller_geometry():
    """Read installed controller settings; fail rather than inventing defaults."""
    try:
        data=yaml.safe_load((SDK_ROOT/'share/hive2_driver/config/robot_motion.yaml').read_text())
        p=data['controllers']['chassis_controller']
        if p['chassis.drive.axis'] != ['0 1 0']*4:
            raise ValueError('unsupported_hive2_drive_axis')
        if p['chassis.drive.steer_angle_source'] != 'feedback' or p['chassis.drive.steer_rate_comp_gain'] != 1.:
            raise ValueError('unsupported_hive2_drive_feedback_or_compensation')
        g=Hive2Geometry(
            wheel_radius=float(p['chassis.wheel_radius']),
            wheel_base_x=float(p['chassis.wheel_base_x']),
            wheel_base_y=float(p['chassis.wheel_base_y']),
            delta_y=float(p['chassis.delta_y']),
            max_linear_speed=float(p['chassis.max_linear_speed']),
            max_angular_speed=float(p.get('chassis.max_angular_speed',
                p['chassis.max_linear_speed']*p['chassis.omega_per_linear'])),
            spin_threshold=float(p['chassis.spin_v_threshold']),
            controller_steering_limit=float(p['chassis.steer.yaw_soft_limit_rad']),
            near_zero_bias=float(p['chassis.steer.near_zero_bias']))
        g.array()
        return g
    except (OSError,KeyError,TypeError,yaml.YAMLError) as exc:
        raise ValueError('hive2_controller_config_unavailable_or_invalid') from exc


def _array(values, size):
    a = np.ascontiguousarray(values, dtype=np.float64)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError('invalid_hive2_state')
    return a


@lru_cache(maxsize=1)
def _native():
    matches = list(Path(__file__).parent.glob('_hive2_native*.so'))
    if len(matches) != 1:
        raise ValueError('hive2_native_adapter_missing_or_ambiguous_rebuild_package')
    try:
        lib = ctypes.CDLL(str(matches[0]))
        ptr = ctypes.POINTER(ctypes.c_double)
        lib.hive2_inverse.argtypes = [ptr]*6
        lib.hive2_forward.argtypes = [ptr]*5
        lib.hive2_inverse.restype = lib.hive2_forward.restype = ctypes.c_int
        return lib
    except OSError as exc:
        raise ValueError('hive2_motion_sdk_unavailable') from exc


def _call(func, *arrays):
    if func(*(a.ctypes.data_as(ctypes.POINTER(ctypes.c_double)) for a in arrays)) != 0:
        raise ValueError('hive2_native_calculation_failed')


def inverse(twist, steering, steering_rates=(0.,)*4, geometry=None):
    geometry=controller_geometry() if geometry is None else geometry
    p,v,q,qd = geometry.array(),_array(twist,3),_array(steering,4),_array(steering_rates,4)
    target,drive = np.zeros(4),np.zeros(4)
    _call(_native().hive2_inverse,p,v,q,qd,target,drive)
    if not np.isfinite(target).all() or not np.isfinite(drive).all():
        raise ValueError('hive2_native_nonfinite')
    return target,drive


def forward(steering, steering_rates, drive_rates, geometry=None):
    geometry=controller_geometry() if geometry is None else geometry
    p,q,qd,d = geometry.array(),_array(steering,4),_array(steering_rates,4),_array(drive_rates,4)
    out = np.zeros(5)
    _call(_native().hive2_forward,p,q,qd,d,out)
    if out[4] != 1 or not np.isfinite(out).all():
        raise ValueError('hive2_forward_invalid')
    return out[:3],float(out[3])


def contact_points(steering, geometry=None):
    """Controller geometry, not an assertion that the URDF/hardware is calibrated."""
    geometry=controller_geometry() if geometry is None else geometry
    geometry.array()
    q=_array(steering,4); side=np.array((1.,-1.,1.,-1.))
    x=np.array((1.,1.,-1.,-1.))*geometry.wheel_base_x/2-side*geometry.delta_y*np.sin(q)
    y=side*geometry.wheel_base_y/2+side*geometry.delta_y*np.cos(q)
    return np.column_stack((x,y))
