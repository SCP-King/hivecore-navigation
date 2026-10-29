import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from n01_scan_dynacbf_bridge.curve import BSplineCurve, CurveError
from n01_scan_dynacbf_bridge.guard_logic import checked_command


def straight():
    return BSplineCurve(3, [0, 0, 0, 0, 1, 1, 1, 1],
                        [(0, 0, .3), (1, 0, .3), (2, 0, .3), (3, 0, .3)])


class TestCurve(unittest.TestCase):
    def test_spline_and_arc(self):
        c = straight()
        self.assertAlmostEqual(c.evaluate(.5)[0], 1.5)
        self.assertAlmostEqual(c.evaluate(c.end)[0], 3)
        self.assertAlmostEqual(c.length, 3)
        self.assertAlmostEqual(c.parameter_at_arc_length(1.5), .5, places=4)

    def test_path_window(self):
        s, points = straight().sample_window(1., 0., 30, .1)
        self.assertAlmostEqual(s, 1., places=2)
        self.assertAlmostEqual(points[0][0], 1., places=2)
        self.assertAlmostEqual(points[-1][0], 3., places=2)

    def test_reject_bad_knots(self):
        with self.assertRaises(CurveError):
            BSplineCurve(3, [0, 0, 0, 0, 1, 1, 0, 1], [(i, 0, 0) for i in range(4)])
        with self.assertRaises(CurveError):
            BSplineCurve(3, [0, 0, 0, 0, 1, 1, 1], [(i, 0, 0) for i in range(4)])

    def test_reject_bad_points(self):
        with self.assertRaises(CurveError):
            BSplineCurve(3, [0, 0, 0, 0, 1, 1, 1, 1], [(math.nan, 0, 0)] * 4)
        with self.assertRaises(CurveError):
            BSplineCurve(3, [0, 0, 0, 0, 1, 1, 1, 1], [(0, 0, 0)] * 4)

    def test_guard(self):
        ok = dict(bridge_age=.1, move_age=.1, plan_age=.1)
        self.assertEqual(checked_command(False, True, True, [.2, .1], **ok)[2], 'DISARMED')
        self.assertEqual(checked_command(True, True, True, [.2, .1], **ok)[2], 'OK')
        self.assertEqual(checked_command(True, True, True, [.2, .1], 2, 0, 0)[2], 'STALE')
        self.assertEqual(checked_command(True, True, True, [math.nan, 0], **ok)[2], 'INVALID_PLAN')
        self.assertEqual(checked_command(True, True, True, [1.2, 0], **ok)[2], 'OUT_OF_BOUNDS')


if __name__ == '__main__':
    unittest.main()
