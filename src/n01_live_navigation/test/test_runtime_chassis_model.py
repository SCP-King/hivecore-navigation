import math
import pytest
import numpy as np
from dataclasses import replace
from n01_live_navigation.hive2_kinematics import controller_geometry,inverse
from n01_live_navigation.runtime_chassis_model import FIELDS,decode_geometry,RuntimeChassisModel


def test_runtime_90_degree_limit_selects_reverse_drive_branch_for_heading_correction():
    base=controller_geometry()
    values={key:getattr(base,field)for key,field in FIELDS.items()}
    values['chassis.steer.yaw_soft_limit_rad']=math.pi/2
    actual=decode_geometry(values,base)
    twist=(.00001,.015,-.0004)
    unconstrained,_=inverse(twist,[math.pi/2]*4,geometry=base)
    constrained,_=inverse(twist,[math.pi/2]*4,geometry=actual)
    assert max(abs(unconstrained))>math.pi/2
    assert max(abs(constrained))<=math.pi/2+1e-10


def test_missing_nonfinite_or_stale_runtime_geometry_fails_closed():
    base=controller_geometry();values={key:getattr(base,field)for key,field in FIELDS.items()}
    with pytest.raises(ValueError):decode_geometry({},base)
    values['chassis.wheel_radius']=float('nan')
    with pytest.raises(ValueError):decode_geometry(values,base)
    fake=type('Model',(),{'current':base,'received':10.,'error':''})()
    with pytest.raises(ValueError):RuntimeChassisModel.geometry(fake,14.)
    assert RuntimeChassisModel.geometry(fake,12.) is base


def test_control_transport_reexecutes_before_ros_init_when_rmw_was_loaded_early(monkeypatch):
    import rclpy,os,sys
    import n01_live_navigation.runtime_chassis_model as module
    calls=[]
    monkeypatch.setattr(rclpy,'get_rmw_implementation_identifier',lambda:'rmw_cyclonedds_cpp')
    monkeypatch.setattr(rclpy,'ok',lambda:False)
    monkeypatch.delenv('N01_CONTROL_RMW_REEXEC',raising=False)
    monkeypatch.setenv('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
    monkeypatch.setattr(sys,'orig_argv',['python3','-m','n01_live_navigation.mapless_motion','forward','.1'])
    monkeypatch.setattr(os,'execv',lambda exe,args:calls.append((exe,args)))
    module.configure_chassis_transport()
    assert calls==[(sys.executable,[sys.executable,'-m','n01_live_navigation.mapless_motion','forward','.1'])]
    assert os.environ['RMW_IMPLEMENTATION']=='rmw_fastrtps_cpp'


def test_control_transport_cannot_change_after_ros_context_has_started(monkeypatch):
    import rclpy
    from n01_live_navigation.runtime_chassis_model import configure_chassis_transport
    monkeypatch.setattr(rclpy,'get_rmw_implementation_identifier',lambda:'rmw_cyclonedds_cpp')
    monkeypatch.setattr(rclpy,'ok',lambda:True)
    with pytest.raises(RuntimeError,match='before ROS initialization'):
        configure_chassis_transport()


def test_lost_parameter_rpc_retries_instead_of_waiting_forever(monkeypatch):
    from types import SimpleNamespace as S
    from unittest.mock import Mock
    import n01_live_navigation.runtime_chassis_model as module
    monkeypatch.setattr(module.time,'monotonic',lambda:10.)
    old=Mock();new=Mock()
    client=Mock();client.service_is_ready.return_value=True;client.call_async.return_value=new
    n=S(pending=old,pending_since=1.,client=client,error='',on_parameters=Mock())
    RuntimeChassisModel.poll(n)
    old.cancel.assert_called_once();client.remove_pending_request.assert_called_once_with(old)
    assert n.pending is new and n.pending_since==10.
    # A cancelled old reply cannot replace new geometry.
    n.current='retained'
    RuntimeChassisModel.on_parameters(n,old)
    assert n.current=='retained'
