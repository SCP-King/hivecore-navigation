"""Behavior checks for feedback throttling and execution outcome semantics."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motion_feedback import MotionFeedback, command_conflicts


class FeedbackTests(unittest.TestCase):
    def test_progress_is_periodic_and_arrival_reported_once(self):
        feedback = MotionFeedback()
        self.assertEqual(len(feedback.feed({'state': 'MOVING'}, 0)), 1)
        self.assertEqual(feedback.feed({'state': 'MOVING'}, 1), [])
        self.assertEqual(len(feedback.feed({'state': 'MOVING'}, 5)), 1)
        result = feedback.feed({'state': 'ARRIVED', 'goal_distance_m': .04}, 6)[0]
        self.assertTrue(result['terminal'])
        self.assertEqual(result['level'], 'ok')
        self.assertEqual(feedback.feed({'state': 'ARRIVED'}, 7), [])
        self.assertEqual(feedback.watchdog(20), [])

    def test_conflict_is_explained_and_recovery_remains_possible(self):
        feedback = MotionFeedback()
        result = feedback.feed({'state': 'COMMAND_SOURCE_CONFLICT', 'reason': 'person_follow_controller_node'}, 0)[0]
        self.assertIn('person_follow_controller_node', result['text'])
        self.assertFalse(result['terminal'])
        self.assertFalse(result['notify'])
        self.assertEqual(feedback.feed({'state': 'MOVING'}, 1)[0]['level'], 'info')

    def test_session_limit_is_not_arrival(self):
        result = MotionFeedback().feed({'state': 'SESSION_COMPLETE'}, 1)[0]
        self.assertNotEqual(result['level'], 'ok')
        self.assertIn('未确认到达', result['text'])

    def test_intermediate_changes_only_log_and_arrival_notifies_once(self):
        feedback = MotionFeedback()
        for i,state in enumerate(['MOVING','NOT_READY','MOVING','NO_PATH','MANUAL_OVERRIDE','ESTOP']):
            result=feedback.feed({'state':state},i)[0]
            self.assertFalse(result['notify'])
        self.assertFalse(feedback.watchdog(20)[0]['notify'])
        self.assertTrue(feedback.feed({'state':'ARRIVED'},21)[0]['notify'])
        self.assertEqual(feedback.feed({'state':'ARRIVED'},22),[])

    def test_lost_feedback_once_and_recovery(self):
        feedback = MotionFeedback()
        feedback.feed({'state': 'MOVING'}, 0)
        self.assertEqual(len(feedback.watchdog(4)), 1)
        self.assertEqual(feedback.watchdog(5), [])
        feedback.feed({'state': 'MOVING'}, 6)
        self.assertEqual(len(feedback.watchdog(10)), 1)

    def test_external_command_publishers_are_preserved_as_conflicts(self):
        self.assertEqual(command_conflicts(['/n01_slow_motion_executor', '/person_follow_controller_node']),
                         ['/person_follow_controller_node'])

    def test_startup_wait_is_distinct_from_lost_running_feedback(self):
        feedback = MotionFeedback()
        feedback.last_rx = 0
        self.assertEqual(feedback.watchdog(4), [])
        result = feedback.watchdog(11)[0]
        self.assertIn('启动超过 10 秒', result['text'])


if __name__ == '__main__':
    unittest.main()


def test_rotation_terminal_failures_report_result_once():
    for state in ('ROTATION_TIMEOUT','ROTATION_DRIFT'):
        feedback=MotionFeedback()
        result=feedback.feed({'state':state},1)[0]
        assert result['terminal'] and result['notify'] and result['level']=='err'
        assert feedback.feed({'state':state},2)==[]
