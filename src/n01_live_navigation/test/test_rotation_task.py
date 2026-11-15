from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
from n01_live_navigation.rotation_guard import rotation_clear
from n01_live_navigation import rotation_task


def scene():
 x,y=np.meshgrid(np.linspace(-2,2,15),np.linspace(-2,2,15))
 return np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))


def node():
 return S(rotation_footprint=S(get=lambda:np.array([[-.4,-.15],[.4,-.15],[.4,.15],[-.4,.15]])),pose=(0.,0.,0.),pose_z=.775,target_yaw=1.,rotation_anchor=None,
  rotation_stop_since=9.,rotation_linear_speed=0.,rotation_yaw_speed=0.,
  rotation_began=None,rotation_odom=0.,rotation_odom_rx=10.,rotation_odom_anchor=None,
  rotation_plane=np.array([0.,0.,0.]),rotation_plane_rx=10.,rotation_static=scene(),
  rotation_static_rx=10.,rotation_map_ready=True,
  rotation_settled=None,max_yaw=.08,dry_run=False,finished=False,cmd_pub=Mock(),
  _zero=Mock(),_status=Mock(),_chassis_command_valid=Mock(return_value=(True,'ok')))


def test_complete_swept_circle_rejects_rear_obstacle():
 assert rotation_clear(scene(),(0,0,.775),(0,0,0))
 assert not rotation_clear(np.vstack((scene(),[[-.7,0,.4]])),(0,0,.775),(0,0,0))


def test_rotation_checks_static_and_live_obstacles(monkeypatch):
 n=node();monkeypatch.setattr(rotation_task,'live_points',lambda node:scene())
 rotation_task.control(n,10.)
 assert n.cmd_pub.publish.call_args.args[0].angular.z==.08
 n=node();n.rotation_static=np.vstack((scene(),[[.2,.25,.5]]))
 rotation_task.control(n,10.)
 n.cmd_pub.publish.assert_not_called();assert n._zero.call_args.args[0]=='ROTATION_BLOCKED'
 n=node();monkeypatch.setattr(rotation_task,'live_points',lambda node:np.vstack((scene(),[[0,-.20,.6]])))
 rotation_task.control(n,10.);n.cmd_pub.publish.assert_not_called()


def test_stale_inputs_and_missing_scan_time_tf_suppress_rotation(monkeypatch):
 n=node();n.rotation_plane_rx=0
 rotation_task.control(n,10.);n.cmd_pub.publish.assert_not_called()


def test_empty_effective_obstacle_map_is_valid_but_stale_or_unready_is_not(monkeypatch):
 n=node();n.rotation_static=np.empty((0,3))
 monkeypatch.setattr(rotation_task,'live_points',lambda node:scene())
 rotation_task.control(n,10.)
 n.cmd_pub.publish.assert_called_once()
 n=node();n.rotation_map_ready=False
 rotation_task.control(n,10.);n.cmd_pub.publish.assert_not_called()
 n=node();n.rotation_static_rx=8.
 rotation_task.control(n,10.);n.cmd_pub.publish.assert_not_called()
 n=node()
 def unavailable(node):raise ValueError('missing scan time TF')
 monkeypatch.setattr(rotation_task,'live_points',unavailable)
 rotation_task.control(n,10.);n.cmd_pub.publish.assert_not_called()


def test_yaw_completion_requires_settling_and_odom_agreement(monkeypatch):
 n=node();monkeypatch.setattr(rotation_task,'live_points',lambda node:scene())
 rotation_task.control(n,10.)
 n.pose=(0,0,1.);n.rotation_odom=1.;n.rotation_odom_rx=10.2
 rotation_task.control(n,10.2);assert not n.finished
 n.rotation_odom_rx=10.8;rotation_task.control(n,10.8)
 assert n.finished and n.completion_state=='ARRIVED'
 n=node();rotation_task.control(n,10.);n.pose=(0,0,1.);n.rotation_odom_rx=10.1
 rotation_task.control(n,10.1)
 assert n._zero.call_args.args[0]=='ROTATION_POSE_MISMATCH'


def test_raw_rotation_cloud_uses_exact_predicted_pose_and_mount_not_late_map_tf():
 from collections import deque
 import pytest
 from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
 from std_msgs.msg import Header
 from builtin_interfaces.msg import Time
 h=Header();h.frame_id='livox_frame';h.stamp=Time(sec=10)
 cloud=create_cloud_xyz32(h,[[1.,0.,.3]]*40)
 mount=S(transform=S(translation=S(x=.295,y=0.,z=0.),rotation=S(x=0.,y=0.,z=0.,w=1.)))
 n=S(rotation_raw=deque([cloud]),rotation_pose_cache=deque([(10*10**9,(2.,0.,0.),(0.,0.,0.,1.))]),
     readiness={'bridge':{'pose_prediction':{}}},
     get_clock=lambda:S(now=lambda:S(nanoseconds=int(10.1e9))),
     rotation_tf=S(lookup_transform=Mock(return_value=mount)))
 xyz=rotation_task.live_points(n)
 assert np.allclose(xyz[:,0],3.295)
 assert n.rotation_tf.lookup_transform.call_args.args[:2]==('base_link','livox_frame')
 n.rotation_pose_cache.clear()
 with pytest.raises(ValueError,match='scan-time TF'):rotation_task.live_points(n)
 assert n.rotation_tf.lookup_transform.call_count==1


def test_rotation_waits_for_measured_stop_before_checking_sweep(monkeypatch):
 n=node();n.rotation_linear_speed=.04
 monkeypatch.setattr(rotation_task,'live_points',lambda node:scene())
 rotation_task.control(n,10.)
 assert n.rotation_anchor is None
 n.cmd_pub.publish.assert_not_called()
 assert n._zero.call_args.args[0]=='STOPPING_FOR_TURN'
 n.rotation_linear_speed=0.;n.rotation_odom_rx=10.1
 rotation_task.control(n,10.1);n.cmd_pub.publish.assert_not_called()
 n.rotation_odom_rx=10.5
 rotation_task.control(n,10.5);n.cmd_pub.publish.assert_called_once()


def test_missing_urdf_geometry_blocks_rotation(monkeypatch):
 n=node()
 def missing():raise ValueError('urdf_footprint_missing_or_stale')
 n.rotation_footprint.get=missing
 monkeypatch.setattr(rotation_task,'live_points',lambda node:scene())
 rotation_task.control(n,10.)
 n.cmd_pub.publish.assert_not_called()
 assert n._zero.call_args.args[0]=='ROTATION_BLOCKED'
