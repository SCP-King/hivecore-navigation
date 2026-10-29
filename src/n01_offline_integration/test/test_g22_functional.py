"""G2.2 deterministic tests: static geometry, jitter, and path risk proxy."""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))

from n01_offline_integration.dynamic_tracking_core import Tracker,clusters_from_points
from n01_offline_integration.dynamic_background import StaticBackgroundFilter
from n01_offline_integration.path_risk_core import evaluate_path_preview


def synthetic_cluster(x,y,z=.8):
    return [(x+dx,y+dy,z) for dx in (-.2,0,.2)
            for dy in (-.2,0,.2) for _ in range(3)]


def cluster_at(x,y):
    return clusters_from_points(synthetic_cluster(x,y),(0.,0.,.9),
                                cell_size=.25,min_cells=2,min_points=6)


def moving_track(x=1.0,y=0.0,vx=-.5,vy=0.):
    return {"id":7,"position_map_m":[x,y,.8],
            "velocity_xy_mps":[vx,vy],"extent_xy_m":[.3,.4],
            "hits":6,"motion_consistency_passed":True,
            "classification":"MOVING_HYPOTHESIS_UNVERIFIED"}


class BackgroundSuppressionTests(unittest.TestCase):
    def test_stationary_cluster_removed_after_background_warmup(self):
        filt=StaticBackgroundFilter(history_frames=8,min_observations=5)
        for i in range(5):
            accept,info=filt.update((i+1)*250_000_000,cluster_at(2,1))
            self.assertTrue(accept)
        accept,info=filt.update(1_500_000_000,cluster_at(2,1))
        self.assertEqual(len(accept),0)
        self.assertGreater(info["stable_background_clusters_rejected"],0)
        self.assertTrue(info["background_warmed"])

    def test_moving_target_not_static_filtered_by_default(self):
        filt=StaticBackgroundFilter()
        accepted=[]
        for i in range(6):
            selected,_=filt.update((i+1)*250_000_000,cluster_at(1.+i*.5,0))
            accepted.append(bool(selected))
        self.assertTrue(all(accepted))

    def test_oversized_structure_rejected(self):
        filt=StaticBackgroundFilter(max_extent_m=1.8)
        bad={"xyz":[4.,1.,.8],"points":50,"cells":20,
             "extent_xy_m":[5.94,.9],
             "voxel_cells":((12,1),(13,1))}
        result,info=filt.update(1,[bad])
        self.assertEqual(result,[])
        self.assertEqual(info["oversized_clusters_rejected"],1)

    def test_background_reset_on_reverse_time(self):
        filt=StaticBackgroundFilter(min_observations=3)
        for i in range(4):
            filt.update((i+1)*100,cluster_at(2,1))
        _,info=filt.update(100,cluster_at(2,1))
        self.assertEqual(info["background_clock_resets"],1)
        self.assertFalse(info["background_warmed"])

    def test_noise_direction_reversal_not_promoted(self):
        tracker=Tracker(min_hits=3,motion_threshold_mps=.18)
        for i in range(10):
            x=2.+(.10 if i%2 else -.10)
            out=tracker.update((i+1)*250_000_000,[{
                "xyz":[x,1.,.8],"cells":4,"extent_xy_m":[.3,.3]}])
        self.assertEqual(out["moving_hypotheses"],0)
        self.assertLess(out["tracks"][0]["motion_direction_consistency"],.8)

    def test_matched_history_persists_then_clears_after_miss(self):
        tracker=Tracker(min_hits=3)
        for i in range(5):
            state=tracker.update((i+1)*250_000_000,[{
                "xyz":[1.+i*.2,0.,.8],"cells":4,
                "extent_xy_m":[.3,.3]}])
        self.assertEqual(len(tracker.tracks),1)
        self.assertGreaterEqual(len(tracker.tracks[0].velocity_history),3)
        self.assertTrue(state["tracks"][0]["motion_consistency_passed"])
        tracker.update(1_500_000_000,[])
        self.assertEqual(tracker.tracks[0].velocity_history,())

    def test_consistent_target_remains_detectable(self):
        tracker=Tracker(min_hits=3)
        for i in range(7):
            out=tracker.update((i+1)*250_000_000,[{
                "xyz":[1.+i*.25,.2,.8],"cells":4,"extent_xy_m":[.3,.3]}])
        self.assertEqual(out["moving_hypotheses"],1)
        self.assertTrue(out["tracks"][0]["motion_consistency_passed"])


class PathPreviewTests(unittest.TestCase):
    def test_crossing_candidate_reports_proxy_conflict(self):
        path=[(0.,0.),(.5,0.),(1.,0.),(2.,0.)]
        report=evaluate_path_preview(path,[moving_track()])
        self.assertEqual(report["status"],"POTENTIAL_PROXIMITY_CONFLICT")
        self.assertLessEqual(report["minimum_barrier_proxy_m2"],0.)
        self.assertGreater(report["evaluated_pairs"],0)
        self.assertEqual(report["actuator_output"],"DISABLED")
        self.assertFalse(report["cbf_solver_implemented"])

    def test_far_object_reports_only_assumption_limited_clearance(self):
        path=[(0.,0.),(1.,0.),(2.,0.)]
        report=evaluate_path_preview(path,[moving_track(x=8.,y=5.,vx=.2)])
        self.assertEqual(report["status"],
                         "NO_PROXY_OVERLAP_UNDER_UNVERIFIED_ASSUMPTIONS")
        self.assertEqual(report["safety_authorization"],"NEVER")

    def test_absence_of_tracks_is_not_safe(self):
        report=evaluate_path_preview([(0,0),(1,0)],[])
        self.assertEqual(report["status"],"NO_CONFIRMED_MOVING_HYPOTHESES")
        self.assertIsNone(report["minimum_barrier_proxy_m2"])
        self.assertFalse(report["physical_collision_clearance_verified"])

    def test_reject_invalid_reference_and_model(self):
        self.assertEqual(evaluate_path_preview([],[])["status"],
                         "NO_USABLE_REFERENCE_PATH")
        self.assertEqual(evaluate_path_preview([(0,0),(float("nan"),0)],[])["status"],
                         "MALFORMED_PATH")
        with self.assertRaises(ValueError):
            evaluate_path_preview([(0,0),(1,0)],[],assumed_speed_mps=0)

    def test_untrusted_track_is_ignored(self):
        candidate=moving_track()
        candidate["motion_consistency_passed"]=False
        report=evaluate_path_preview([(0,0),(2,0)],[candidate])
        self.assertEqual(report["candidate_tracks"],0)


if __name__=="__main__":
    unittest.main()
