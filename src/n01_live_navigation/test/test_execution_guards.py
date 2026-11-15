import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from n01_live_navigation.trajectory_monitor import Hive2TrajectoryMonitor
from n01_live_navigation.slow_motion_executor import SlowMotionExecutor


class ExecutionGuardTests(unittest.TestCase):
    def test_wheel_only_joint_update_preserves_leg_pose_without_refresh(self):
        positions={'fl_leg_pitch1':.78}
        fake=SimpleNamespace(joint_positions=positions,joint_rx=10.,joint_stamp_ns=123)
        Hive2TrajectoryMonitor.on_joint_states(fake,SimpleNamespace(name=['wheel_fl']))
        self.assertIs(fake.joint_positions,positions)
        self.assertEqual((fake.joint_rx,fake.joint_stamp_ns),(10.,123))

    def test_wheel_only_joint_update_preserves_but_does_not_refresh_steering(self):
        feedback=([.1]*4,[0.]*4)
        fake=SimpleNamespace(steering_feedback=feedback,steering_feedback_rx=10.)
        SlowMotionExecutor._on_steering(fake,SimpleNamespace(name=['wheel_fl','wheel_fr','wheel_rl','wheel_rr']))
        self.assertIs(fake.steering_feedback,feedback)
        self.assertEqual(fake.steering_feedback_rx,10.)

    def test_disabling_after_manual_takeover_does_not_publish_zero(self):
        fake=SimpleNamespace(finished=False,enabled=True,manual_override=True,_zero=Mock())
        SlowMotionExecutor._on_enable(fake,SimpleNamespace(data=False))
        self.assertFalse(fake.enabled)
        fake._zero.assert_not_called()

    def test_new_navigation_goal_cancels_rotation(self):
        fake=SimpleNamespace(motion_mode='rotate',finished=False)
        SlowMotionExecutor._on_accepted_goal(fake,SimpleNamespace())
        self.assertTrue(fake.finished)
        self.assertEqual(fake.completion_state,'GOAL_CHANGED')

    def test_arrival_verification_tolerates_localization_jitter_without_restarting(self):
        from builtin_interfaces.msg import Time
        fake=SimpleNamespace(final_goal=SimpleNamespace(x=1.,y=0.,z=0.),pose=(.921,0.,0.),
            arrival_check_started=None,finished=False,_zero=Mock(),goal_reached_pub=Mock(),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=10))))
        self.assertTrue(SlowMotionExecutor._handle_arrival(fake,10.))
        self.assertEqual(fake._zero.call_args.args[0],'VERIFYING_ARRIVAL')
        fake.pose=(.919,0.,0.)  # 1mm outside entry radius, inside exit hysteresis
        self.assertTrue(SlowMotionExecutor._handle_arrival(fake,10.6))
        self.assertTrue(fake.finished)
        self.assertEqual(fake._zero.call_args.args[0],'ARRIVED')
        event=fake.goal_reached_pub.publish.call_args.args[0]
        self.assertEqual(event.header.frame_id,'map')
        self.assertEqual(event.point.x,1.)

    def test_arrival_rejects_motion_outside_exit_band(self):
        fake=SimpleNamespace(final_goal=SimpleNamespace(x=1.,y=0.),pose=(.87,0.,0.),
            arrival_check_started=10.,finished=False,_zero=Mock())
        self.assertFalse(SlowMotionExecutor._handle_arrival(fake,11.))
        self.assertIsNone(fake.arrival_check_started)
        self.assertFalse(fake.finished)

    def test_native_command_requires_fresh_steering_feedback(self):
        fake=SimpleNamespace(steering_feedback=None,steering_feedback_rx=0.)
        self.assertFalse(SlowMotionExecutor._chassis_command_valid(fake,.03,0.,1.)[0])
        fake.steering_feedback=([0.]*4,[0.]*4)
        self.assertFalse(SlowMotionExecutor._chassis_command_valid(fake,.03,0.,1.)[0])
        self.assertTrue(SlowMotionExecutor._chassis_command_valid(fake,.03,0.,.1)[0])

    def test_native_command_preserves_physical_steering_guard(self):
        fake=SimpleNamespace(steering_feedback=([3.2]*4,[0.]*4),steering_feedback_rx=1.)
        valid,reason=SlowMotionExecutor._chassis_command_valid(fake,.03,0.,1.)
        self.assertFalse(valid)
        self.assertIn('steering range',reason)

    def test_steering_callback_does_not_fabricate_missing_velocity(self):
        fake=SimpleNamespace(steering_feedback=([0.]*4,[0.]*4),steering_feedback_rx=0.,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000_000)))
        msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0)),
            name=[f'{leg}_ankle_yaw' for leg in ('fl','fr','rl','rr')],position=[0.]*4,velocity=[])
        SlowMotionExecutor._on_steering(fake,msg)
        self.assertIsNone(fake.steering_feedback)

    def test_completed_session_cannot_be_reenabled(self):
        fake = SimpleNamespace(finished=True, enabled=False)
        SlowMotionExecutor._on_enable(fake, SimpleNamespace(data=True))
        self.assertFalse(fake.enabled)

    def test_stale_odometry_cannot_refresh_travel_guard(self):
        fake = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=20_000_000_000)),
            odom_rx=0.0, odom_xy=None, travelled_m=0.0)
        msg = SimpleNamespace(header=SimpleNamespace(frame_id='odom',stamp=SimpleNamespace(sec=1,nanosec=0)),
                              pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.,y=1.))))
        SlowMotionExecutor._on_odom(fake, msg)
        self.assertEqual(fake.odom_rx, 0.0)

    def test_valid_window_rechecks_feedback_without_extending_receive_time(self):
        fake = SimpleNamespace(
            last_path_active=True, last_bspline=object(),
            last_trajectory_window=(10_000_000_000, 8.0),
            last_bspline_rx=time.monotonic()-6,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=16_000_000_000)),
            on_bspline=Mock())
        original_rx = fake.last_bspline_rx
        Hive2TrajectoryMonitor.publish_missing_or_stale_status(fake)
        fake.on_bspline.assert_called_once_with(fake.last_bspline, replay=True)
        self.assertEqual(original_rx, fake.last_bspline_rx)

    def test_travel_limit_latches_stop(self):
        now = time.monotonic()
        fake = SimpleNamespace(manual_override=False, observer_ready=True, _poll_command_observer=Mock(),
                               speed=.03, commanded_m=.18, last_control=now,
                               finished=False, enabled=True, session_start=now, session_duration=15,
                               distance_limit=.18, travelled_m=.19, _zero=Mock())
        SlowMotionExecutor._control(fake)
        self.assertTrue(fake.finished)
        self.assertEqual(fake._zero.call_args.args[0], 'SESSION_COMPLETE')
        fake._zero.reset_mock()
        fake.travelled_m = fake.commanded_m = 0
        SlowMotionExecutor._control(fake)
        self.assertEqual(fake._zero.call_args.args[0], 'SESSION_COMPLETE')

    def test_expired_window_does_not_replay(self):
        fake = SimpleNamespace(
            last_path_active=True, last_bspline=object(),
            last_trajectory_window=(10_000_000_000, 2.0),
            last_bspline_rx=time.monotonic()-8,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=20_000_000_000)),
            on_bspline=Mock(), _clear_candidate_visualization=Mock(),
            last_watchdog_status='STALE_BSPLINE', max_bspline_receive_age=1,
            diag_pub=SimpleNamespace(publish=Mock()))
        Hive2TrajectoryMonitor.publish_missing_or_stale_status(fake)
        fake.on_bspline.assert_not_called()
        fake._clear_candidate_visualization.assert_called_once()


if __name__ == '__main__':
    unittest.main()


def test_confirmed_180_degree_range_accepts_feedback_above_90_degrees():
    from types import SimpleNamespace
    from n01_live_navigation.slow_motion_executor import SlowMotionExecutor
    fake=SimpleNamespace(steering_feedback=([2.]*4,[0.]*4),steering_feedback_rx=1.)
    assert SlowMotionExecutor._chassis_command_valid(fake,.03,0.,1.)[0]
