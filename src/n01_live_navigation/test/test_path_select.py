"""Unit tests for path_select.select_source (no ROS imports needed).

Covers the executor's path-source decision surface:
unified strict / unified_grace gap handling / global fallback / exhausted
prefix skipping at replan boundaries / frame and freshness gating.
"""
import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
MOD_PATH = ROOT / "src/n01_live_navigation/n01_live_navigation/path_select.py"


def load_module():
    """Load path_select from source without rclpy."""
    mod = types.ModuleType("path_select")
    exec(compile(MOD_PATH.read_text(), str(MOD_PATH), "exec"), mod.__dict__)
    sys.modules["path_select"] = mod
    return mod


select_source = load_module().select_source


def make_path(frame="map", n=10, stamp_ns=1000, dx=0.2):
    poses = []
    for i in range(n):
        poses.append(types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=float(i) * dx, y=0.0, z=0.0)),
            header=types.SimpleNamespace(frame_id=frame,
                                         stamp=types.SimpleNamespace(sec=0, nanosec=2))))
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id=frame,
                                     stamp=types.SimpleNamespace(sec=0, nanosec=stamp_ns)),
        poses=poses)


def base_kwargs(**over):
    kw = dict(
        path_source="auto",
        now=50.0,
        path=make_path(),
        path_rx=49.5,
        path_timeout=300.0,
        path_status=dict(valid=True, validated_prefix_points=5, path_stamp_ns=1000),
        path_status_rx=49.9,
        path_valid_at=49.5,
        grace_s=1.0,
        global_path=make_path(stamp_ns=5000),
        global_path_rx=49.0,
        global_timeout_s=30.0,
        pose=(0.4, 0.0, 0.0),
    )
    kw.update(over)
    return kw


class SelectSourceTests(unittest.TestCase):
    def test_unified_preferred_when_fresh_and_valid(self):
        r = select_source(**base_kwargs())
        self.assertIsNotNone(r)
        source, poses, prefix, stamp = r
        self.assertEqual(source, "unified")
        self.assertEqual(prefix, 5)
        self.assertEqual(stamp, 1000)
        self.assertEqual(len(poses), 10)

    def test_unified_strict_never_falls_back(self):
        self.assertIsNone(select_source(**base_kwargs(
            path_source="unified", path_status_rx=10.0)))
        # strict mode ignores grace data
        r = select_source(**base_kwargs(path_source="unified", path_valid_at=0.0))
        self.assertEqual(r[0], "unified")

    def test_invalid_current_status_does_not_stop_auto(self):
        # an invalid *current* status must route auto to the fresh global path
        kw = base_kwargs(path_status=dict(valid=False, reason="replanning"))
        r = select_source(**kw)
        self.assertEqual(r[0], "global")

    def test_grace_holds_last_validated_prefix_in_gap(self):
        # status freshness lapsed (replan in flight) but last valid pair is
        # recent: keep the validated prefix, do not zero /cmd_vel.
        r = select_source(**base_kwargs(path_status_rx=0.0, path_valid_at=49.6))
        self.assertIsNotNone(r)
        self.assertEqual((r[0], r[2]), ("unified_grace", 5))

    def test_grace_expires_then_global(self):
        r = select_source(**base_kwargs(path_status_rx=0.0, path_valid_at=20.0))
        self.assertIsNotNone(r)
        self.assertEqual(r[0], "global")
        self.assertEqual(r[2], len(r[1]))

    def test_exhausted_prefix_skips_to_global_at_replan_boundary(self):
        # pose sits on the validated prefix tip (idx 4 -> x=0.8), prefix<len:
        # auto must go global instead of holding (which would WAITING).
        r = select_source(**base_kwargs(pose=(0.95, 0.0, 0.0)))
        self.assertEqual(r[0], "global")

    def test_global_only_source(self):
        r = select_source(**base_kwargs(path_source="global"))
        self.assertEqual(r[0], "global")
        self.assertIsNone(select_source(**base_kwargs(
            path_source="global", global_path_rx=0.0)))

    def test_global_frame_and_length_gates(self):
        self.assertIsNone(select_source(**base_kwargs(
            path_source="global", global_path=make_path(frame="odom", stamp_ns=7000))))
        self.assertIsNone(select_source(**base_kwargs(
            path_source="global", global_path=make_path(n=1))))

    def test_path_absent_falls_back_to_fresh_global(self):
        r = select_source(**base_kwargs(path=None, path_status=None))
        self.assertEqual(r[0], "global")
        self.assertIsNone(select_source(**base_kwargs(path=None, path_status=None,
                                                      global_path=None)))

    def test_stamp_mismatch_rejects_unified(self):
        self.assertIsNone(select_source(**base_kwargs(
            path_status=dict(valid=True, validated_prefix_points=5,
                             path_stamp_ns=123),
            global_path=None)))

    def test_stale_unified_path_drops_to_global_then_none(self):
        # 50 s old with a 10 s unified timeout: unified+grace both dead.
        kw = dict(path_rx=0.0, path_timeout=10.0, path_valid_at=20.0)
        r = select_source(**base_kwargs(**kw))
        self.assertEqual(r[0], "global")
        self.assertIsNone(select_source(**base_kwargs(
            global_path=None, **kw)))


if __name__ == "__main__":
    unittest.main(verbosity=2)