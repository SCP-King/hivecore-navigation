import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from n01_live_navigation.trajectory_monitor import Hive2TrajectoryMonitor
from n01_live_navigation.slow_motion_executor import SlowMotionExecutor


class ExecutionGuardTests(unittest.TestCase):
    def test_native_command_requires_fresh_steering_feedback(self):
        fake=SimpleNamespace(steering_feedback=None,steering_feedback_rx=0.)
        self.assertFalse(SlowMotionExecutor._chassis_command_valid(fake,.03,0.,1.)[0])
        fake.steering_feedback=([0.]*4,[0.]*4)
        self.assertFalse(SlowMotionExecutor._chassis_command_valid(fake,.03,0.,1.)[0])
        self.assertTrue(SlowMotionExecutor._chassis_command_valid(fake,.03,0.,.1)[0])

    def test_native_command_preserves_physical_steering_guard(self):
        fake=SimpleNamespace(steering_feedback=([2.]*4,[0.]*4),steering_feedback_rx=1.)
        valid,reason=SlowMotionExecutor._chassis_command_valid(fake,.03,0.,1.)
        self.assertFalse(valid)
        self.assertIn('90deg',reason)

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
        fake = SimpleNamespace(speed=.03, commanded_m=.18, last_control=now,
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
