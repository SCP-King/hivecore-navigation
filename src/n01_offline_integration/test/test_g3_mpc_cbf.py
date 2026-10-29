"""G3 bounded real discrete CBF beam-search optimizer + full core data flow."""
import math
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))

from n01_offline_integration.g3_mpc_cbf_core import solve_discrete_mpc_cbf
from n01_offline_integration.g3_pipeline_core import G3HistoricalCoordinator


PATH=[(0.,0.),(1.,0.),(2.,0.),(3.,0.)]


def moving(x=1.,vx=-.1):
    return {"id":11,"classification":"MOVING_HYPOTHESIS_UNVERIFIED",
            "motion_consistency_passed":True,"hits":6,
            "position_map_m":[x,0.,.9],
            "velocity_xy_mps":[vx,0.],
            "extent_xy_m":[.3,.4]}


def report(stamp,tracks):
    return {"schema":"n01.offline.tracks.v1","status":"PROCESSED",
            "frame_id":"map","stamp_ns":stamp,"tracks":tracks}


class DiscreteCBFOptimizerTests(unittest.TestCase):
    def test_nominal_real_solver_no_unsafe_success_claim(self):
        out=solve_discrete_mpc_cbf(PATH,(0.,0.),[])
        self.assertTrue(out["solver_executed"])
        self.assertEqual(out["status"],
                         "FEASIBLE_NOMINAL_ONLY_NO_DYNAMIC_CONSTRAINTS")
        self.assertEqual(len(out["candidate_xy_m"]),7)
        self.assertEqual(out["cbf_constraints_checked"],0)
        self.assertEqual(out["safety_authorization"],"NEVER")
        self.assertFalse(out["collision_safety_verified"])
        self.assertFalse(out["trajectory_is_control_command"])
        self.assertTrue(out["search_truncated"])

    def test_dynamic_constraint_actually_checked_and_trajectory_changed(self):
        out=solve_discrete_mpc_cbf(PATH,(0.,0.),[moving()])
        self.assertTrue(out["solver_executed"])
        self.assertGreater(out["cbf_constraints_checked"],0)
        self.assertGreater(out["cbf_constraints_rejected"],0)
        self.assertLess(out["minimum_nominal_h_m2"],0)
        self.assertEqual(out["status"],
                         "FEASIBLE_CBF_CONSTRAINED_CANDIDATE_UNVERIFIED")
        self.assertGreaterEqual(out["minimum_candidate_h_m2"],-1e-6)
        self.assertNotEqual(out["candidate_xy_m"],out["nominal_xy_m"])
        self.assertEqual(len(out["candidate_xy_m"]),7)
        self.assertFalse(out["real_robot_ready"])

    def test_output_candidate_independently_satisfies_discrete_cbf_and_kinematics(self):
        out=solve_discrete_mpc_cbf(PATH,(0.,0.),[moving()])
        self.assertEqual(out["status"],
                         "FEASIBLE_CBF_CONSTRAINED_CANDIDATE_UNVERIFIED")
        pts=out["candidate_xy_m"]
        radius=.55+.5*math.hypot(.3,.4)
        dt=.25
        previous_v=(0.,0.)
        for k in range(6):
            a,b=pts[k:k+2]
            v=((b[0]-a[0])/dt,(b[1]-a[1])/dt)
            self.assertLessEqual(math.hypot(*v),.75+.002)
            self.assertLessEqual(math.hypot(v[0]-previous_v[0],
                                           v[1]-previous_v[1]),3.2*dt+.002)
            old_obstacle=(1.-.1*k*dt,0.)
            new_obstacle=(1.-.1*(k+1)*dt,0.)
            h=(a[0]-old_obstacle[0])**2+a[1]**2-radius**2
            hnext=(b[0]-new_obstacle[0])**2+b[1]**2-radius**2
            self.assertGreaterEqual(hnext+0.002,.65*h)
            self.assertGreaterEqual(hnext,-.002)
            previous_v=v

    def test_initial_overlap_never_relabeled_feasible(self):
        out=solve_discrete_mpc_cbf(PATH,(0.,0.),[moving(x=.3)])
        self.assertEqual(out["status"],"INFEASIBLE_INITIAL_CBF_PROXY_OVERLAP")
        self.assertEqual(out["candidate_xy_m"],[])
        self.assertTrue(out["solver_executed"])

    def test_invalid_or_far_map_path_refuses_optimization(self):
        bad=solve_discrete_mpc_cbf(PATH,(5.,5.),[])
        self.assertEqual(bad["status"],"BODY_OUTSIDE_REFERENCE_CORRIDOR")
        self.assertFalse(bad["solver_executed"])
        self.assertEqual(solve_discrete_mpc_cbf([(0,0),(float('nan'),0)],
                                                 (0,0),[])["status"],
                         "INVALID_INPUT")

    def test_unverified_or_stationary_clusters_not_used_as_cbf_truth(self):
        row=moving()
        row["motion_consistency_passed"]=False
        r=solve_discrete_mpc_cbf(PATH,(0,0),[row])
        self.assertEqual(r["moving_constraint_tracks"],0)
        self.assertEqual(r["status"],
                         "FEASIBLE_NOMINAL_ONLY_NO_DYNAMIC_CONSTRAINTS")

    def test_invalid_horizon_and_nonfinite_track_fail_closed(self):
        self.assertFalse(solve_discrete_mpc_cbf(PATH,(0,0),[],
                                                 dt=0)["solver_executed"])
        bad=moving()
        bad["velocity_xy_mps"]=[float("nan"),0]
        self.assertEqual(solve_discrete_mpc_cbf(PATH,(0,0),[bad])["status"],
                         "INVALID_TRACKS")


class G3RealBagPipelineTests(unittest.TestCase):
    def test_real_data_shape_exact_stamps_reaches_optimizer_after_late_path(self):
        p=G3HistoricalCoordinator()
        t=1_765_000_000_000_000_000
        for i in range(1,7):
            stamp=t+i*250_000_000
            self.assertEqual(p.put_pose(stamp,(.1*i,0.,.9)),[])
            out=p.put_tracks(report(stamp,[]))
            self.assertEqual(out[0]["status"],"WAITING_FOR_SCAN_PATH")
        self.assertEqual(p.input_pairs,6)
        solved=p.set_path(PATH,999)
        self.assertEqual(len(solved),6)
        self.assertEqual(p.evidence()["solver_executed_frames"],6)
        self.assertEqual(p.evidence()["posthoc_solver_frames"],6)
        self.assertTrue(all(x["solver_executed"] for x in solved))
        self.assertTrue(all(x["post_hoc_reference_comparison"] for x in solved))
        self.assertTrue(all(x["safety_authorization"]=="NEVER" for x in solved))
        self.assertEqual(p.evidence()["nominal_only_solutions"],6)

    def test_future_live_scan_reaches_dynamic_constrained_optimizer(self):
        p=G3HistoricalCoordinator()
        p.set_path(PATH,999)
        t=1_765_000_000_000_000_000
        self.assertEqual(p.put_tracks(report(t,[moving()])),[])
        out=p.put_pose(t,(0.,0.,.9))
        self.assertEqual(len(out),1)
        self.assertTrue(out[0]["solver_executed"])
        self.assertEqual(out[0]["moving_constraint_tracks"],1)
        self.assertFalse(out[0]["post_hoc_reference_comparison"])
        self.assertEqual(p.evidence()["dynamic_cbf_candidate_solutions"],1)
        self.assertEqual(out[0]["recorded_scan_stamp_ns"],t)

    def test_near_but_unmatched_stamps_cannot_use_latest_pose(self):
        p=G3HistoricalCoordinator()
        p.set_path(PATH)
        t=1_765_000_000_000_000_000
        p.put_pose(t,(0,0,.9))
        self.assertEqual(p.put_tracks(report(t+1,[moving()])),[])
        self.assertEqual(p.evidence()["solver_executed_frames"],0)

    def test_nonprojectable_real_pose_no_trajectory_is_fabricated(self):
        p=G3HistoricalCoordinator()
        p.set_path(PATH)
        stamp=1_765_000_000_000_000_000
        p.put_pose(stamp,(9.,9.,.9))
        result=p.put_tracks(report(stamp,[]))[0]
        self.assertEqual(result["status"],"BODY_OUTSIDE_REFERENCE_CORRIDOR")
        self.assertFalse(result["solver_executed"])
        self.assertEqual(result["candidate_xy_m"],[])

    def test_reverse_bag_stamp_invalidates_retained_path(self):
        p=G3HistoricalCoordinator()
        p.set_path(PATH)
        p.put_pose(2_000_000_000,(0,0,.9))
        p.put_tracks(report(2_000_000_000,[]))
        p.put_pose(1_000_000_000,(0,0,.9))
        result=p.put_tracks(report(1_000_000_000,[]))[0]
        self.assertEqual(result["status"],"REVERSED_RECORDED_SCAN_TIME")
        self.assertIsNone(p.path)
        self.assertEqual(p.clock_reversals,1)

    def test_posthoc_queue_bound(self):
        p=G3HistoricalCoordinator(max_backfill=8)
        for i in range(1,16):
            p.put_pose(i,(0,0,.9))
            p.put_tracks(report(i,[]))
        self.assertEqual(p.evidence()["waiting_pairs_for_path"],8)
        self.assertEqual(len(p.set_path(PATH)),8)


if __name__=="__main__":
    unittest.main()
