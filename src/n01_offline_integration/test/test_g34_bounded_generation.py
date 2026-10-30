"""G3.4 fixed-pitch Hive2 translationally bounded research solver contracts.

Legacy control lattice is preserved for historical comparison. Integrated G3
must generate, not merely post-hoc flag, SCAN acceleration-bounded candidates.
This is NOT a wheel steering, braking, or live motion test.
"""
import pathlib
import sys
import unittest
from unittest.mock import patch

ROOT=pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"src/n01_offline_integration"))
from n01_offline_integration.g3_mpc_cbf_core import solve_discrete_mpc_cbf
from n01_offline_integration.g3_pipeline_core import G3HistoricalCoordinator
from n01_offline_integration.g3_hive2_wheel_screen import (
    screen_g3_result,SCAN_MAX_ACCEL_MPS2,SCAN_MAX_SPEED_MPS)

PATH=[(0.,0.),(1.,0.),(2.,0.),(3.,0.)]


def report(stamp,tracks=()):
    return {"schema":"n01.offline.tracks.v1","status":"PROCESSED",
            "frame_id":"map","stamp_ns":stamp,"tracks":list(tracks)}


class G34BoundedGenerationTests(unittest.TestCase):
    def _paired(self, tracks=(), *, stamp=1_765_000_000_000_000_000):
        p=G3HistoricalCoordinator()
        p.set_path(PATH,path_header_ns=1234)
        self.assertEqual(p.put_pose(stamp,(0.,0.,.9)),[])
        result=p.put_tracks(report(stamp,tracks))[0]
        return p,result

    def test_g34_generates_positive_progress_under_scan_acceleration_cap(self):
        p,r=self._paired()
        self.assertTrue(r["solver_executed"])
        self.assertEqual(r["candidate_generation_profile"],
                         "HIVE2_SCAN_CONFIG_BOUNDED_XY_ONLY")
        self.assertEqual(r["control_lattice"],"acceleration_increment")
        self.assertEqual(r["status"],
                         "FEASIBLE_NOMINAL_ONLY_NO_DYNAMIC_CONSTRAINTS")
        self.assertEqual(len(r["candidate_xy_m"]),7)
        self.assertGreater(r["candidate_xy_m"][-1][0],0.02,
                           "must not pass by using a frozen zero-speed trajectory")
        screen=screen_g3_result(r)
        self.assertEqual(screen["status"],
                         "TRANSLATIONAL_BOUNDS_WITHIN_CONFIG_4WS_UNVERIFIED")
        self.assertLessEqual(screen["speed_max_mps"],SCAN_MAX_SPEED_MPS+.001)
        self.assertLessEqual(screen["accelerations_max_mps2"],
                             SCAN_MAX_ACCEL_MPS2+.005)
        self.assertEqual(p.evidence()["hive2_scan_limits_exceeded_frames"],0)
        self.assertEqual(p.evidence()["hive2_chassis_limits_exceeded_frames"],0)
        self.assertFalse(r["wheel_steering_feasibility_verified"])
        self.assertFalse(r["real_robot_ready"])
        self.assertEqual(r["safety_authorization"],"NEVER")
        self.assertEqual(r["actuator_output"],"DISABLED")
        self.assertEqual(r["initial_velocity_source"],
                         "ASSUMED_ZERO_NOT_MEASURED_BAG_TWIST")

    def test_legacy_model_is_not_silently_modified(self):
        legacy=solve_discrete_mpc_cbf(PATH,(0.,0.),[])
        self.assertEqual(legacy["control_lattice"],"legacy_fixed_speed")
        self.assertEqual(screen_g3_result(legacy)["status"],
                         "SCAN_TRANSLATIONAL_PLANNING_BOUNDS_EXCEEDED" if
                         not screen_g3_result(legacy)["chassis_config_limits_exceeded"]
                         else "CONFIGURED_CHASSIS_TRANSLATION_EXCEEDED")
        constrained=solve_discrete_mpc_cbf(
            PATH,(0.,0.),[],max_accel=SCAN_MAX_ACCEL_MPS2,
            max_speed=SCAN_MAX_SPEED_MPS,control_lattice="acceleration_increment")
        self.assertEqual(screen_g3_result(constrained)["status"],
                         "TRANSLATIONAL_BOUNDS_WITHIN_CONFIG_4WS_UNVERIFIED")

    def test_dynamic_obstacle_still_uses_cbf_not_false_clearance(self):
        tracked={
            "id": 11,"classification":"MOVING_HYPOTHESIS_UNVERIFIED",
            "motion_consistency_passed":True,"hits":6,
            "position_map_m":[1.0,0.0,.9],
            "velocity_xy_mps":[-.1,0.0],
            "extent_xy_m":[.3,.4]
        }
        p,r=self._paired([tracked])
        self.assertEqual(r["moving_constraint_tracks"],1)
        self.assertTrue(r["solver_executed"])
        self.assertGreater(r["cbf_constraints_checked"],0)
        self.assertEqual(r["trajectory_is_control_command"],False)
        if r["candidate_xy_m"]:
            self.assertEqual(r["status"],
                             "FEASIBLE_CBF_CONSTRAINED_CANDIDATE_UNVERIFIED")
            self.assertFalse(r["hive2_motion_limit_screen"]["scan_planning_limits_exceeded"])
        else:
            self.assertTrue(r["status"].startswith("INFEASIBLE"))
            self.assertEqual(p.evidence()["dynamic_cbf_candidate_solutions"],0)
        self.assertFalse(r["real_robot_ready"])

    def test_integrated_pipeline_rejects_overspeed_result_before_output(self):
        def inaccurate_solver(*args,**kwargs):
            # Synthetic malicious/upstream bad candidate, not a real planner result.
            return {
                "schema":"n01.offline.g3_mpc_cbf.v1",
                "status":"FEASIBLE_NOMINAL_ONLY_NO_DYNAMIC_CONSTRAINTS",
                "solver_executed":True,
                "moving_constraint_tracks":0,
                "cbf_constraints_checked":0,
                "cbf_constraints_rejected":0,
                "candidate_xy_m":[[0.,0.],[.1875,0.],[.375,0.]],
                "nominal_xy_m":[[0.,0.],[.1875,0.],[.375,0.]],
                "discretization_dt_s":.25,
            }
        with patch("n01_offline_integration.g3_pipeline_core.solve_discrete_mpc_cbf",
                   side_effect=inaccurate_solver):
            p,r=self._paired()
        self.assertEqual(r["status"],"INFEASIBLE_CONFIG_BOUNDS_SCREEN_REJECTED")
        self.assertEqual(r["candidate_xy_m"],[])
        self.assertTrue(r["candidate_rejected_after_screen"])
        self.assertTrue(r["hive2_motion_limit_screen"]["chassis_config_limits_exceeded"])
        self.assertEqual(p.evidence()["hive2_chassis_limits_exceeded_frames"],1)
        self.assertEqual(p.evidence()["infeasible_solver_frames"],1)
        self.assertEqual(p.evidence()["nominal_only_solutions"],0)

    def test_stamped_pairing_and_far_path_fail_closed(self):
        p=G3HistoricalCoordinator()
        p.set_path(PATH,path_header_ns=1234)
        self.assertEqual(p.put_pose(100,(0.,0.,.9)),[])
        self.assertEqual(p.put_tracks(report(101)),[])
        self.assertEqual(p.evidence()["solver_executed_frames"],0)
        result=p.put_tracks(report(100))[0]
        self.assertEqual(result["status"],
                         "FEASIBLE_NOMINAL_ONLY_NO_DYNAMIC_CONSTRAINTS")
        p2=G3HistoricalCoordinator()
        p2.set_path(PATH)
        p2.put_pose(100,(20.,20.,.9))
        bad=p2.put_tracks(report(100))[0]
        self.assertEqual(bad["candidate_xy_m"],[])
        self.assertFalse(bad["solver_executed"])
        self.assertFalse(bad["real_robot_ready"])


if __name__=="__main__":
    unittest.main()
