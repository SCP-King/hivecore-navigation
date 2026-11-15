import math
import pytest
from n01_live_navigation.delayed_pose import DelayedPoseFusion
from n01_live_navigation.arrival_confirmation import measurement_confirms

def pose(x=0.,y=0.,yaw=0.):return (x,y,0.),(0.,0.,math.sin(yaw/2),math.cos(yaw/2))
def stamp(s):return int(s*1e9)
def fusion():
 f=DelayedPoseFusion();f.accept(stamp(10),pose(),pose(100),stamp(10),pose(100));return f

def test_predicts_15_second_delayed_pose_with_measured_odometry():
 f=fusion();p=f.pose(stamp(11.5),pose(100.045))
 assert p[0][0]==pytest.approx(.045)
 assert f.last_source==stamp(10)
 with pytest.raises(ValueError,match='horizon'):f.pose(stamp(12.01),pose(100.06))

def test_distance_turn_and_correction_jump_bounds_are_independent_of_time():
 f=fusion()
 with pytest.raises(ValueError,match='distance'):f.pose(stamp(10.1),pose(100.13))
 with pytest.raises(ValueError,match='turn'):f.pose(stamp(10.1),pose(100,yaw=.21))
 with pytest.raises(ValueError,match='jump'):f.accept(stamp(10.1),pose(1),pose(100),stamp(10.1),pose(100))
 with pytest.raises(ValueError,match='reset'):f.pose(stamp(10.2),pose(100))
 f.reset();assert not f.fault

def test_delayed_real_correction_is_continuous_bounded_and_converges():
 f=fusion();before=f.pose(stamp(11.5),pose(100.045))
 f.accept(stamp(10.5),pose(.005),pose(100.015),stamp(11.5),pose(100.045))
 assert f.pose(stamp(11.5),pose(100.045))==before
 assert f.pose(stamp(11.6),pose(100.045))[0][0]==pytest.approx(.04375)
 # Standstill odometry and a newer real fix permit convergence without aging out.
 f.accept(stamp(11.6),pose(.035),pose(100.045),stamp(11.6),pose(100.045))
 assert f.pose(stamp(12.5),pose(100.045))[0][0]==pytest.approx(.035)
 # Evaluating old scans after a new correction preserves their historical pose.
 assert f.pose(stamp(11.5),pose(100.045))==before

def test_duplicates_and_reordered_observations_never_refresh_anchor():
 f=fusion()
 assert not f.accept(stamp(10),pose(5),pose(100),stamp(11),pose(100))
 assert not f.accept(stamp(9),pose(5),pose(100),stamp(11),pose(100))
 assert f.last_source==stamp(10)

def test_rotation_correction_does_not_amplify_distant_odom_origin():
 f=DelayedPoseFusion();f.accept(stamp(10),pose(),pose(1e5,1e5),stamp(10),pose(1e5,1e5))
 f.accept(stamp(10.1),pose(yaw=.04),pose(1e5,1e5),stamp(10.1),pose(1e5,1e5))
 p=f.pose(stamp(10.5),pose(1e5,1e5))
 assert math.hypot(*p[0][:2])<1e-8

def test_prediction_cannot_confirm_arrival_before_real_post_stop_measurement():
 h={'bridge':{'pose_prediction':{'source_stamp_ns':10,'source_pose':{'x':1.,'y':0.,'quaternion':[0,0,0,1]},'fault':''}}}
 assert not measurement_confirms(h,11,xy=(1,0))
 h['bridge']['pose_prediction']['source_stamp_ns']=12
 assert measurement_confirms(h,11,xy=(1,0))
 assert not measurement_confirms(h,11,xy=(1.3,0))
 assert not measurement_confirms(h,11,yaw=.1)

def test_twist_integration_ignores_doubled_raw_position_and_handles_gap():
 from n01_live_navigation.twist_odometry import TwistOdometry
 o=TwistOdometry()
 for i in range(101):
  p,q=o.update(stamp(10+i*.02),(.06*i*.02,0,0),(0,0,0,1),(.03,0,0))
 assert p[0]==pytest.approx(.06)
 assert p[0]!=pytest.approx(.12)
 with pytest.raises(ValueError,match='advancing'):o.update(stamp(12),p,q,(.03,0,0))
 o.update(stamp(13),(3,0,0),q,(.03,0,0));assert o.segment_start==stamp(13)

def test_twist_integration_rotates_body_velocity_and_rejects_nonfinite():
 from n01_live_navigation.twist_odometry import TwistOdometry
 o=TwistOdometry();q=pose(yaw=math.pi/2)[1]
 o.update(stamp(10),(0,0,0),q,(1,0,0))
 p,_=o.update(stamp(10.05),(0,0,0),q,(1,0,0))
 assert p[1]==pytest.approx(.05)
 with pytest.raises(ValueError,match='invalid'):o.update(stamp(10.06),p,q,(float('nan'),0,0))


def test_held_board_copies_cannot_refresh_anchor_while_odometry_moves():
 from n01_live_navigation.held_pose import HeldPoseGuard
 g=HeldPoseGuard();p,q=pose()
 assert g.accept(stamp(10),p,q,pose())
 assert g.accept(stamp(10.1),p,q,pose(.003))
 assert not g.accept(stamp(10.2),p,q,pose(.006))
 assert not g.accept(stamp(10.3),p,q,pose(.01))
 assert not g.accept(stamp(11),p,q,None)
 p,q=pose(.01)
 assert g.accept(stamp(11.1),p,q,pose(.01))


def test_five_second_low_speed_profile_still_enforces_motion_and_time_budgets():
 f=DelayedPoseFusion(horizon=5.)
 f.accept(stamp(10),pose(),pose(),stamp(10),pose())
 assert f.pose(stamp(13.5),pose(.1))[0][0]==pytest.approx(.1)
 with pytest.raises(ValueError,match='distance'):f.pose(stamp(13.6),pose(.121))
 with pytest.raises(ValueError,match='turn'):f.pose(stamp(13.6),pose(yaw=.201))
 with pytest.raises(ValueError,match='horizon'):f.pose(stamp(15.01),pose(.1))
 # A delayed true fix starts a smooth correction rather than resetting position.
 before=f.pose(stamp(14),pose(.11))
 f.accept(stamp(12),pose(.052),pose(.06),stamp(14),pose(.11))
 assert f.pose(stamp(14),pose(.11))==before


def test_measured_yaw_rate_ignores_raw_odom_heading_jumps_without_restarting():
 from n01_live_navigation.twist_odometry import TwistOdometry
 o=TwistOdometry()
 o.update(stamp(10),pose()[0],pose()[1],(.1,0,0),(0,0,.1))
 # Raw yaw jumps 30 degrees; actual measured rate is only .1 rad/s.
 p,q=o.update(stamp(10.05),pose()[0],pose(yaw=.52)[1],(.1,0,0),(0,0,.1))
 assert 2*math.atan2(q[2],q[3])==pytest.approx(.005)
 assert o.segment_start==stamp(10)
 assert p[0]==pytest.approx(.005,abs=1e-6)
 assert p[1]==pytest.approx(.0000125,abs=1e-7)
 # Timestamp gaps still break prediction continuity and require a new SLAM anchor.
 _,q=o.update(stamp(11),pose()[0],pose(yaw=.6)[1],(.1,0,0),(0,0,.1))
 assert o.segment_start==stamp(11)
 assert 2*math.atan2(q[2],q[3])==pytest.approx(.6)


def test_heading_integration_uses_trapezoid_and_rejects_invalid_or_missing_twist():
 from n01_live_navigation.twist_odometry import TwistOdometry
 o=TwistOdometry()
 o.update(stamp(10),pose()[0],pose(yaw=3.14)[1],(0,0,0),(0,0,.1))
 _,q=o.update(stamp(10.04),pose()[0],pose(yaw=-3.13)[1],(0,0,0),(0,0,.3))
 assert abs(math.atan2(math.sin(2*math.atan2(q[2],q[3])-3.148),
                       math.cos(2*math.atan2(q[2],q[3])-3.148)))<1e-8
 with pytest.raises(ValueError,match='angular_twist_invalid'):
  o.update(stamp(10.06),pose()[0],pose()[1],(0,0,0),(0,0,float('nan')))
 with pytest.raises(ValueError,match='angular_twist_missing'):
  o.update(stamp(10.06),pose()[0],pose()[1],(0,0,0))
