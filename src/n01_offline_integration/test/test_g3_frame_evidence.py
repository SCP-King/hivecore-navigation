"""Pure-Python G3.1 audit tests: exact provenance, no synthetic safety claim."""
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from n01_offline_integration.g3_frame_evidence import (
    frame_row, append_frame, summarize)


def result(posthoc=False, stamp=123456789):
    data = dict(schema="n01.offline.g3_mpc_cbf.v1", solver_executed=True,
                recorded_scan_stamp_ns=stamp, path_reference_seq=1,
                source="REAL_HISTORICAL_BAG_DETECTED_TARGET_HYPOTHESES",
                post_hoc_reference_comparison=posthoc,
                safety_authorization="NEVER", actuator_output="DISABLED",
                nominal_xy_m=[[0, 0], [.1, 0]],
                candidate_xy_m=[[0, 0], [.1, 0]],
                status="FEASIBLE_NOMINAL_ONLY_NO_DYNAMIC_CONSTRAINTS",
                moving_constraint_tracks=0, cbf_constraints_checked=0,
                cbf_constraints_rejected=0, minimum_nominal_h_m2=None,
                minimum_candidate_h_m2=None)
    return data


class G3FrameEvidenceTests(unittest.TestCase):
    def test_new_attempt_has_strict_offline_provenance(self):
        row = frame_row(result(posthoc=True))
        self.assertTrue(row["solver_executed"])
        self.assertEqual(row["path_origin"], "POST_HOC_HISTORICAL_COMPARISON")
        self.assertEqual(row["safety_authorization"], "NEVER")
        self.assertFalse(row["collision_safety_verified"])
        self.assertIsNone(row["minimum_candidate_h_m2"])

    def test_heartbeats_unsolved_and_unsafe_input_not_recorded(self):
        data = result()
        data["solver_executed"] = False
        self.assertIsNone(frame_row(data))
        data = result()
        data["actuator_output"] = "ENABLED"
        with self.assertRaises(ValueError):
            frame_row(data)

    def test_nonfinite_and_no_candidate_fail_closed(self):
        data = result()
        data["nominal_xy_m"][0][0] = float("nan")
        with self.assertRaises(ValueError):
            frame_row(data)
        data = result()
        data["candidate_xy_m"] = []
        with self.assertRaises(ValueError):
            frame_row(data)

    def test_journal_summary_detects_duplicate_and_truncation(self):
        with tempfile.TemporaryDirectory() as d:
            path = pathlib.Path(d) / "g3_frame_evidence.jsonl"
            with path.open("w", encoding="utf-8") as f:
                append_frame(f, result())
                append_frame(f, result(posthoc=True, stamp=123456790))
            summary = summarize(path)
            self.assertEqual(summary["counts"]["attempts"], 2)
            self.assertEqual(summary["counts"]["post_hoc_attempts"], 1)
            with path.open("a", encoding="utf-8") as f:
                append_frame(f, result())
            with self.assertRaisesRegex(ValueError, "duplicate"):
                summarize(path)
            path.write_text('{"broken":', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "truncated"):
                summarize(path)

    def test_nonexecuted_solver_frame_does_not_write(self):
        with tempfile.TemporaryDirectory() as d:
            path = pathlib.Path(d) / "g3_frame_evidence.jsonl"
            with path.open("w", encoding="utf-8") as f:
                data = result()
                data["solver_executed"] = False
                self.assertIsNone(append_frame(f, data))
            self.assertEqual(path.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
