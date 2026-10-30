"""G3.5 regression: CBF hypothesis detection is not proof of intervention.

The captured 2026-10-30 Bag had moving-hypothesis frames with zero CBF
candidate rejections. Do not turn that into an avoidance-success claim.
"""
import pathlib
import sys
import tempfile
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"src/n01_offline_integration"))

from n01_offline_integration.g3_mpc_cbf_core import solve_discrete_mpc_cbf
from n01_offline_integration.g3_frame_evidence import append_frame,summarize


def moving(x):
    return {"id": 7,"classification":"MOVING_HYPOTHESIS_UNVERIFIED",
            "motion_consistency_passed":True,"hits":6,
            "position_map_m":[x,0.0,0.9],"velocity_xy_mps":[-.1,0.],
            "extent_xy_m":[.3,.4]}


def decorated(result,stamp):
    result.update({
        "recorded_scan_stamp_ns":stamp,"path_reference_seq":1,
        "source":"REAL_HISTORICAL_BAG_DETECTED_TARGET_HYPOTHESES",
        "post_hoc_reference_comparison":True,
        "safety_authorization":"NEVER",
        "actuator_output":"DISABLED",
    })
    return result


class CBFAttributionTests(unittest.TestCase):
    def test_far_moving_hypothesis_does_not_count_as_avoidance(self):
        r=solve_discrete_mpc_cbf(
            [(0.,0.),(1.,0.),(3.,0.)],(0.,0.),[moving(8.)],
            max_accel=.5,max_speed=.75,control_lattice="acceleration_increment")
        self.assertTrue(r["solver_executed"])
        self.assertEqual(r["moving_constraint_tracks"],1)
        self.assertGreater(r["cbf_constraints_checked"],0)
        self.assertEqual(r["cbf_constraints_rejected"],0)
        self.assertFalse(r["cbf_candidate_pruning_observed"])
        self.assertFalse(r["nominal_cbf_proxy_overlap_observed"])
        self.assertEqual(r["status"],"FEASIBLE_CBF_CONSTRAINED_CANDIDATE_UNVERIFIED")
        self.assertEqual(r["safety_authorization"],"NEVER")

    def test_near_obstacle_requires_real_cbf_candidate_rejection(self):
        r=solve_discrete_mpc_cbf(
            [(0.,0.),(1.,0.),(2.,0.),(3.,0.)],(0.,0.),[moving(1.)])
        self.assertTrue(r["solver_executed"])
        self.assertGreater(r["cbf_constraints_rejected"],0)
        self.assertTrue(r["cbf_candidate_pruning_observed"])
        self.assertLess(r["minimum_nominal_h_m2"],0)
        self.assertTrue(r["nominal_cbf_proxy_overlap_observed"])
        self.assertEqual(r["safety_authorization"],"NEVER")

    def test_frame_journal_counts_hypotheses_vs_search_pruning(self):
        good=decorated(solve_discrete_mpc_cbf(
            [(0.,0.),(1.,0.),(3.,0.)],(0.,0.),[moving(8.)],
            max_accel=.5,max_speed=.75,control_lattice="acceleration_increment"),101)
        prune=decorated(solve_discrete_mpc_cbf(
            [(0.,0.),(1.,0.),(2.,0.),(3.,0.)],(0.,0.),[moving(1.)]),102)
        with tempfile.TemporaryDirectory() as d:
            path=pathlib.Path(d)/"audit.jsonl"
            with path.open("w",encoding="utf-8") as f:
                a=append_frame(f,good)
                b=append_frame(f,prune)
            self.assertFalse(a["cbf_candidate_pruning_observed"])
            self.assertFalse(a["nominal_cbf_proxy_overlap_observed"])
            self.assertTrue(b["cbf_candidate_pruning_observed"])
            self.assertTrue(b["nominal_cbf_proxy_overlap_observed"])
            counts=summarize(path)["counts"]
            self.assertEqual(counts["cbf_hypothesis_present_attempts"],2)
            self.assertEqual(counts["cbf_candidate_pruning_observed_attempts"],1)
            self.assertEqual(counts["nominal_cbf_proxy_overlap_observed_attempts"],1)
            self.assertEqual(counts["attempts"],2)

if __name__=="__main__":
    unittest.main()
