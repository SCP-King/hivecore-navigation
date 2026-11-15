import math
import pytest
from types import SimpleNamespace
from unittest.mock import Mock
from n01_live_navigation.mapless_motion import command,relative_pose,MaplessMotion,YawAccumulator,lateral_feedback_command,wrap


def test_complete_turn_is_accumulated_across_wrap_and_is_not_zero_goal():
    accumulator=YawAccumulator()
    for i in range(401):accumulator.update(wrap(2.8+2*math.pi*i/400))
    assert accumulator.total==pytest.approx(2*math.pi)
    v,arrived=command('rotate',2*math.pi,(0,0,0))
    assert not arrived and v[2]==.08
    assert command('rotate',2*math.pi,(0,0,2*math.pi-.01))[1]
    with pytest.raises(ValueError):accumulator.update(wrap(accumulator.previous+1.))


def test_extended_lateral_corrects_heading_and_returns_to_same_anchor():
    v,arrived=lateral_feedback_command(1.,(.02,.2,.02))
    assert not arrived and v[0]<0 and v[2]<0
    assert math.hypot(*v[:2])<=.030000001
    v,arrived=lateral_feedback_command(0.,(0,1.,0))
    assert not arrived and v[1]<0
    assert lateral_feedback_command(0.,(.005,.005,.005))[1]
    with pytest.raises(ValueError):lateral_feedback_command(1.,(0,0,.2))


def test_complete_turn_closed_loop_does_not_finish_at_start_or_half_turn():
    yaw=0.;dt=.05;steps=0
    while steps<4000:
        v,done=command('rotate',2*math.pi,(0,0,yaw))
        if done:break
        yaw+=v[2]*dt;steps+=1
    assert 1500<steps<4000
    assert abs(yaw-2*math.pi)<math.radians(1)


def test_extended_out_and_back_with_small_heading_disturbance():
    x=y=yaw=0.;dt=.05;outbound=None
    for target in (1.,0.):
        for step in range(3000):
            (vx,vy,omega),done=lateral_feedback_command(target,(x,y,yaw))
            if done:break
            c,s=math.cos(yaw),math.sin(yaw)
            x+=(c*vx-s*vy+.0005)*dt
            y+=(s*vx+c*vy)*dt
            yaw+=(omega+.0005)*dt
        else:pytest.fail('closed-loop commissioning target did not converge')
        if target==1.:outbound=(x,y,yaw)
    assert abs(outbound[1]-1)<.008
    assert math.hypot(x,y)<.02 and abs(yaw)<math.radians(1)

def test_left_relative_to_initial_body_heading():
    x,y,yaw=relative_pose((2,3,math.pi/2),(1.9,3,math.pi/2))
    assert abs(x)<1e-9 and y==pytest.approx(.1) and yaw==0

def test_translation_and_rotation_commands_are_separate_and_bounded():
    v,done=command('rotate',math.radians(10),(0,0,0))
    assert v[:2]==(0,0) and 0<v[2]<=.08 and not done
    v,done=command('lateral',.1,(0,0,0))
    assert v[0]==v[2]==0 and 0<v[1]<=.03 and not done
    assert command('lateral',.1,(0,.095,0))[1]

def test_cross_axis_drift_rejects_motion():
    with pytest.raises(ValueError):command('rotate',.1,(.05,0,0))
    with pytest.raises(ValueError):command('lateral',.1,(0,0,.1))

def test_yaw_wrap_crossing_pi():
    assert relative_pose((0,0,math.pi-.05),(0,0,-math.pi+.05))[2]==pytest.approx(.1)

def test_lateral_does_not_enter_native_spin_deadband_near_goal():
    v,done=command('lateral',.1,(0,.088,0))
    assert not done and v[1]>.01
    v,done=command('lateral',.1,(0,.095,0))
    assert done and v==(0.,0.,0.)

def test_manual_owner_receives_no_commands_even_during_cleanup():
    fake=SimpleNamespace(cmd_pub=Mock(),manual_override=True)
    MaplessMotion.emit(fake,(0.,0.,0.))
    fake.cmd_pub.publish.assert_not_called()

def test_dry_run_emits_no_command_without_publisher():
    fake=SimpleNamespace(cmd_pub=None,manual_override=False)
    MaplessMotion.emit(fake,(0.,.03,0.))
    assert fake.cmd_linear_y==.03


def test_short_forward_primitive_is_bounded_and_rejects_cross_axis_drift():
    v,done=command('forward',.1,(0,0,0))
    assert v==(.03,0.,0.) and not done
    assert command('forward',.1,(.095,0,0))==((0.,0.,0.),True)
    with pytest.raises(ValueError):command('forward',.1,(0,.041,0))
    with pytest.raises(ValueError):command('forward',.1,(0,0,.06))
    with pytest.raises(ValueError):command('invalid',.1,(0,0,0))
