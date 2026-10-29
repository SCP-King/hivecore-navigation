"""G2.3 pure-Python replay path matching and timing regression tests."""
import pathlib
import sys
import unittest

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from n01_offline_integration.replay_path_sync import (
    ReplayPathSynchronizer, project_path,
)


def candidate(stamp_ns, moving=True, x=1.):
    track={"id":17, "position_map_m":[x,.0,.9],
           "velocity_xy_mps":[-.3,0.], "extent_xy_m":[.3,.4],
           "hits":6,"motion_consistency_passed":True,
           "classification":"MOVING_HYPOTHESIS_UNVERIFIED"}
    return {"schema":"n01.offline.tracks.v1","frame_id":"map",
            "status":"PROCESSED", "stamp_ns":stamp_ns,
            "tracks":[track] if moving else []}


class RecordedTimeAlignmentTests(unittest.TestCase):
    def make_sync(self):
        s=ReplayPathSynchronizer()
        self.assertTrue(s.set_path([(0.,0.),(1.,0.),(2.,0.),(4.,0.)],
                                   header_stamp_ns=1761770000000000000))
        return s

    def test_exact_timestamp_no_latest_pose_substitution(self):
        s=self.make_sync()
        self.assertIsNone(s.put_body(1000000000,(.1,0.)))
        self.assertIsNone(s.put_tracks(candidate(1000000001)))
        self.assertEqual(s.frames_aligned,0)
        result=s.put_tracks(candidate(1000000000))
        self.assertEqual(result["recorded_scan_stamp_ns"],1000000000)
        self.assertEqual(result["aligned_replay_frames"] if "aligned_replay_frames" in result else s.frames_aligned,1)
        self.assertEqual(result["time_alignment"],
                         "EXACT_RECORDED_BODY_POSE_AND_TRACK_STAMP")
        self.assertFalse(result["path_header_is_historical_stamp"])

    def test_tracks_then_body_works(self):
        s=self.make_sync()
        self.assertIsNone(s.put_tracks(candidate(1000000000)))
        result=s.put_body(1000000000,(0.,0.))
        self.assertEqual(result["status"],"POTENTIAL_PROXIMITY_CONFLICT")

    def test_retains_single_path_across_many_bag_frames(self):
        s=self.make_sync()
        reports=[]
        for i in range(1,26):
            stamp=i*250000000
            s.put_body(stamp,(.1+i*.02,0.))
            reports.append(s.put_tracks(candidate(stamp,moving=True)))
        self.assertTrue(all(p is not None for p in reports))
        self.assertEqual(s.path_seq,1)
        self.assertEqual(s.frames_aligned,25)
        self.assertEqual(s.proximity_pairs,25)
        self.assertEqual(reports[-1]["reference_provenance"],
                         "RETAINED_SCAN_PATH_GEOMETRY_UNVERIFIED")
        self.assertTrue(reports[-1]["reference_reused_without_replanning"])
        self.assertNotEqual(reports[-1]["status"],"NO_FRESH_REFERENCE_PATH")

    def test_projects_robot_pose_to_path_at_each_recorded_frame(self):
        p=project_path([(0.,0.),(2.,0.),(4.,0.)],(1.25,.25))
        self.assertAlmostEqual(p[0],.25)
        self.assertAlmostEqual(p[1],1.25)
        self.assertAlmostEqual(p[2][0][0],1.25)
        s=self.make_sync()
        s.put_body(1000000000,(1.25,.25))
        result=s.put_tracks(candidate(1000000000,moving=False))
        self.assertAlmostEqual(result["projected_path_progress_m"],1.25)
        self.assertAlmostEqual(result["body_to_reference_distance_m"],.25)

    def test_path_outside_corridor_is_inconclusive(self):
        s=self.make_sync()
        s.put_body(1000000000,(5.,5.))
        r=s.put_tracks(candidate(1000000000))
        self.assertEqual(r["status"],"BODY_OUTSIDE_RETAINED_PATH_CORRIDOR")
        self.assertEqual(r["evaluated_pairs"],0)
        self.assertEqual(r["safety_authorization"],"NEVER")

    def test_reverse_time_invalidates_reference_and_requires_new_path(self):
        s=self.make_sync()
        s.put_body(2000000000,(.0,0.))
        s.put_tracks(candidate(2000000000))
        s.put_body(1000000000,(.0,0.))
        r=s.put_tracks(candidate(1000000000))
        self.assertEqual(r["status"],"REPLAY_TIMESTAMP_REVERSED")
        self.assertFalse(s.evidence()["reference_retained"])
        s.put_body(1250000000,(0.,0.))
        r=s.put_tracks(candidate(1250000000))
        self.assertEqual(r["status"],"NO_RETAINED_REFERENCE_GEOMETRY")

    def test_path_end_is_not_safety_pass(self):
        s=ReplayPathSynchronizer()
        s.set_path([(0.,0.),(1.,0.)],header_stamp_ns=1)
        s.put_body(1000000000,(1.,0.))
        r=s.put_tracks(candidate(1000000000))
        self.assertEqual(r["status"],"AT_REFERENCE_PATH_END")
        self.assertEqual(r["evaluated_pairs"],0)

    def test_no_moving_candidates_inconclusive_but_frame_aligned(self):
        s=self.make_sync()
        s.put_body(1000000000,(0.,0.))
        r=s.put_tracks(candidate(1000000000,moving=False))
        self.assertEqual(r["status"],"NO_CONFIRMED_MOVING_HYPOTHESES")
        self.assertEqual(s.frames_aligned,1)
        self.assertEqual(s.proximity_pairs,0)
        self.assertEqual(r["safety_authorization"],"NEVER")

    def test_invalid_and_unmatched_frames_bounded(self):
        s=ReplayPathSynchronizer(max_pairs=3)
        for i in range(1,7):
            self.assertIsNone(s.put_body(i,(.1,0.)))
        self.assertEqual(s.evidence()["unmatched_body_queue"],3)
        self.assertIsNone(s.put_tracks({"status":"PROCESSED"}))
        self.assertIsNone(s.put_tracks(candidate(1)))
        self.assertEqual(s.evidence()["unmatched_tracks_queue"],1)
        self.assertEqual(s.frames_aligned,0)

    def test_invalid_path_never_used(self):
        s=ReplayPathSynchronizer()
        self.assertFalse(s.set_path([(0.,0.),(float("nan"),0.)]))
        s.put_body(1000000000,(0.,0.))
        r=s.put_tracks(candidate(1000000000))
        self.assertEqual(r["status"],"NO_RETAINED_REFERENCE_GEOMETRY")

    def test_minimum_risk_has_recorded_time_and_both_xy_positions(self):
        s=self.make_sync()
        s.put_body(1000000000,(0.,0.))
        report=s.put_tracks(candidate(1000000000))
        self.assertEqual(report["recorded_scan_stamp_ns"],1000000000)
        self.assertGreater(report["evaluated_pairs"],0)
        self.assertIn("nearest_robot_path_xy_m",report)
        self.assertIn("nearest_predicted_obstacle_xy_m",report)
        self.assertEqual(len(report["nearest_robot_path_xy_m"]),2)
        self.assertEqual(len(report["nearest_predicted_obstacle_xy_m"]),2)
        self.assertGreaterEqual(report["at_time_s"],0.)
        self.assertLessEqual(report["at_time_s"],1.5)
        self.assertEqual(report["time_basis"],
                         "RELATIVE_TO_EXACT_MATCHED_RECORDED_SCAN")
        self.assertEqual(report["actuator_output"],"DISABLED")

    def test_reference_path_replacement_increments_version(self):
        s=self.make_sync()
        self.assertEqual(s.path_seq,1)
        self.assertTrue(s.set_path([(2.,0.),(3.,0.)],header_stamp_ns=999))
        self.assertEqual(s.path_seq,2)
        s.put_body(1000000000,(2.,0.))
        report=s.put_tracks(candidate(1000000000,moving=False))
        self.assertEqual(report["path_reference_seq"],2)
        self.assertEqual(report["path_header_stamp_ns"],999)
        self.assertFalse(report["path_header_is_historical_stamp"])

    def test_scan_gap_recorded_without_claiming_sync_success(self):
        s=self.make_sync()
        s.put_body(1000000000,(0.,0.))
        s.put_tracks(candidate(1000000000))
        s.put_body(5000000000,(0.,0.))
        s.put_tracks(candidate(5000000000))
        self.assertEqual(s.replay_gaps,1)


if __name__=="__main__":
    unittest.main()
