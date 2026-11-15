import numpy as np
from n01_live_navigation.core import compose_transform, invert_transform, transform_points
from n01_live_navigation.pose_frame import change_pose_child, change_twist_child


def test_live_scan_world_coordinates_survive_sensor_to_footprint_roundtrip():
    # Captured sensor pose; its former footprint label caused a second +0.9m.
    position=(-.822489,.832611,-.000973)
    orientation=(.000474,-.000325,-.327967,.944689)
    foot_from_lidar=((.295,0,.9),(0,0,0,1))
    lidar_from_foot=invert_transform(*foot_from_lidar)
    foot_t,foot_q,cov=change_pose_child(position,orientation,np.eye(6).ravel(),lidar_from_foot)
    body_t,_=compose_transform(foot_t,foot_q,(0,0,.9),(0,0,0,1))
    assert abs(body_t[2]-position[2])<.002
    rng=np.random.default_rng(3);scan=rng.normal(size=(100,3))
    foot_points=transform_points(scan,*foot_from_lidar)
    world=transform_points(foot_points,foot_t,foot_q)
    expected=transform_points(scan,position,orientation)
    np.testing.assert_allclose(world,expected,atol=3e-7)
    assert np.linalg.eigvalsh(np.reshape(cov,(6,6))).min()>0


def test_rotating_sensor_velocity_converts_to_stationary_footprint_origin():
    mount=invert_transform((.295,0,.9),(0,0,0,1))
    linear,angular,cov=change_twist_child((0,.295,0),(0,0,1),np.eye(6).ravel(),mount)
    np.testing.assert_allclose(linear,[0,0,0],atol=1e-10)
    np.testing.assert_allclose(angular,[0,0,1],atol=1e-10)
    assert np.linalg.eigvalsh(np.reshape(cov,(6,6))).min()>0


def test_initial_pose_conversion_is_inverse_of_output_conversion():
    mount=invert_transform((.295,0,.9),(0,0,0,1))
    p,q,cov=change_pose_child((1,2,-.9),(0,0,0,1),np.eye(6).ravel(),invert_transform(*mount))
    restored,orientation,_=change_pose_child(p,q,cov,mount)
    np.testing.assert_allclose(restored,[1,2,-.9])
    np.testing.assert_allclose(orientation,[0,0,0,1])
