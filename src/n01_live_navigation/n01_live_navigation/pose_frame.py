"""Rigid child-frame changes for localization pose, covariance and twist."""
import numpy as np
from .core import compose_transform, rotate_translate, invert_transform


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation(q):
    return np.column_stack([rotate_translate(v, (0, 0, 0), q) for v in np.eye(3)])


def change_pose_child(position, orientation, covariance, source_from_output):
    """map<-source -> map<-output; preserve covariance in map axes."""
    t, q = source_from_output
    out_t, out_q = compose_transform(position, orientation, t, q)
    lever_map = np.array(rotate_translate(t, (0, 0, 0), orientation))
    jac = np.eye(6)
    jac[:3, 3:] = -skew(lever_map)
    cov = jac@np.asarray(covariance).reshape(6, 6)@jac.T
    return out_t, out_q, cov.ravel().tolist()


def change_twist_child(linear, angular, covariance, source_from_output):
    """Shift the velocity evaluation point, then rotate into output axes."""
    t, q = source_from_output
    _, inverse_q = invert_transform(t, q)
    r = rotation(inverse_q)
    jac = np.zeros((6, 6))
    jac[:3, :3] = jac[3:, 3:] = r
    jac[:3, 3:] = -r@skew(t)
    value = jac@np.array([*linear, *angular])
    cov = jac@np.asarray(covariance).reshape(6, 6)@jac.T
    return value[:3].tolist(), value[3:].tolist(), cov.ravel().tolist()
