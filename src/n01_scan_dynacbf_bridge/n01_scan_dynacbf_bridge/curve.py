"""Pure Python evaluation of SCAN position B-splines (no commands)."""
import math
from bisect import bisect_left, bisect_right


class CurveError(ValueError):
    """Invalid or unsampleable B-spline."""


class BSplineCurve:
    def __init__(self, order, knots, control_points, table_samples=400):
        if isinstance(order, bool) or not isinstance(order, int) or not 1 <= order <= 6:
            raise CurveError('invalid B-spline degree')
        self.degree = order
        self.points = [tuple(float(v) for v in p) for p in control_points]
        self.knots = [float(v) for v in knots]
        self.count = len(self.points)
        if not order < self.count <= 4096:
            raise CurveError('invalid control point count')
        if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in self.points):
            raise CurveError('invalid xyz control points')
        if len(self.knots) != self.count + order + 1:
            raise CurveError('invalid knot vector size')
        if not all(math.isfinite(k) for k in self.knots) or any(a > b for a, b in zip(self.knots, self.knots[1:])):
            raise CurveError('invalid knot vector')
        self.start, self.end = self.knots[order], self.knots[self.count]
        if self.end - self.start <= 1e-9:
            raise CurveError('empty domain')
        if isinstance(table_samples, bool) or not isinstance(table_samples, int) or not 20 <= table_samples <= 20000:
            raise CurveError('invalid table sample count')
        self.table_u = [self.start + (self.end - self.start) * i / table_samples for i in range(table_samples + 1)]
        self.table_xyz = [self.evaluate(u) for u in self.table_u]
        self.table_s = [0.0]
        for a, b in zip(self.table_xyz, self.table_xyz[1:]):
            self.table_s.append(self.table_s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = self.table_s[-1]
        if self.length <= 1e-6:
            raise CurveError('zero planar arc length')

    def evaluate(self, u):
        if not math.isfinite(u):
            raise CurveError('non-finite parameter')
        u = min(max(float(u), self.start), self.end)
        p, n = self.degree, self.count
        k = n - 1 if u >= self.end else max(p, min(n - 1, bisect_right(self.knots, u) - 1))
        d = [list(self.points[k - p + j]) for j in range(p + 1)]
        for r in range(1, p + 1):
            for j in range(p, r - 1, -1):
                idx = k - p + j
                den = self.knots[idx + p - r + 1] - self.knots[idx]
                alpha = (u - self.knots[idx]) / den if den > 1e-12 else 0.0
                d[j] = [(1.0 - alpha) * a + alpha * b for a, b in zip(d[j - 1], d[j])]
        return tuple(d[p])

    def parameter_at_arc_length(self, s):
        if not math.isfinite(s):
            raise CurveError('non-finite arc')
        s = min(max(s, 0.0), self.length)
        i = min(max(1, bisect_left(self.table_s, s)), len(self.table_s) - 1)
        s0, s1 = self.table_s[i - 1], self.table_s[i]
        if s1 - s0 <= 1e-12:
            return self.table_u[i]
        f = (s - s0) / (s1 - s0)
        return self.table_u[i - 1] + f * (self.table_u[i] - self.table_u[i - 1])

    def nearest_arc_length(self, x, y):
        if not (math.isfinite(x) and math.isfinite(y)):
            raise CurveError('non-finite robot position')
        best, s = float('inf'), 0.0
        for i in range(1, len(self.table_xyz)):
            a, b = self.table_xyz[i - 1], self.table_xyz[i]
            dx, dy = b[0] - a[0], b[1] - a[1]
            den = dx * dx + dy * dy
            f = min(max(((x - a[0]) * dx + (y - a[1]) * dy) / den, 0.0), 1.0) if den > 1e-20 else 0.0
            qx, qy = a[0] + f * dx, a[1] + f * dy
            d2 = (qx - x) ** 2 + (qy - y) ** 2
            if d2 < best:
                best = d2
                s = self.table_s[i - 1] + f * (self.table_s[i] - self.table_s[i - 1])
        return s

    def sample_window(self, x, y, count=120, step=0.05):
        if isinstance(count, bool) or not isinstance(count, int) or not 2 <= count <= 3000:
            raise CurveError('invalid point count')
        if not math.isfinite(step) or not 0.001 <= step <= 5.0:
            raise CurveError('invalid arc step')
        anchor = self.nearest_arc_length(x, y)
        points = []
        for i in range(count):
            u = self.parameter_at_arc_length(min(self.length, anchor + i * step))
            px, py, pz = self.evaluate(u)
            eps = min((self.end - self.start) * 1e-4, 0.005)
            a = self.evaluate(max(self.start, u - eps))
            b = self.evaluate(min(self.end, u + eps))
            yaw = math.atan2(b[1] - a[1], b[0] - a[0]) if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-10 else (points[-1][3] if points else 0.0)
            points.append((px, py, pz, yaw))
        return anchor, points
