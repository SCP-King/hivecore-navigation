"""N01 RViz-only demo input geometry must be strict and temporally labeled."""
import pathlib
import sys
import unittest

PKG=pathlib.Path(__file__).resolve().parents[1]/"n01_offline_integration"
sys.path.insert(0,str(PKG.parent))
from n01_offline_integration.demo_geometry import (
    preview_tracks,predict_xy,reference_result,
)


def track(id=4):
    return {"id":id,"position_map_m":[1.,2.,.9],
            "velocity_xy_mps":[-.4,.2],
            "extent_xy_m":[.3,.6],"hits":5,
            "motion_consistency_passed":True,
            "classification":"MOVING_HYPOTHESIS_UNVERIFIED"}


class DemoGeometryTests(unittest.TestCase):
    def test_hypothesis_coordinates_and_velocity(self):
        got=preview_tracks({
            "schema":"n01.offline.tracks.v1","frame_id":"map",
            "status":"PROCESSED","stamp_ns":123,"tracks":[track()]})
        self.assertEqual(len(got),1)
        self.assertEqual(got[0]["xy"],(1.,2.))
        self.assertEqual(got[0]["velocity"],(-.4,.2))

    def test_unconfirmed_and_wrong_map_tracks_not_drawn(self):
        item=track()
        item["motion_consistency_passed"]=False
        self.assertEqual(preview_tracks({
            "schema":"n01.offline.tracks.v1","frame_id":"map",
            "status":"PROCESSED","stamp_ns":123,"tracks":[item]}),[])
        self.assertEqual(preview_tracks({
            "schema":"n01.offline.tracks.v1","frame_id":"lidar_link",
            "status":"PROCESSED","stamp_ns":123,"tracks":[track()]}),[])

    def test_prediction_is_constant_velocity_hypothesis_only(self):
        got=predict_xy((1.,2.),(-.4,.2),1.5,.25)
        self.assertEqual(len(got),7)
        self.assertAlmostEqual(got[-1][0],.4)
        self.assertAlmostEqual(got[-1][1],2.3)

    def test_posthoc_and_separate_nominal_candidate(self):
        msg={
            "schema":"n01.offline.g3_mpc_cbf.v1",
            "is_new_solver_input":True,
            "solver_executed":True,
            "safety_authorization":"NEVER",
            "actuator_output":"DISABLED",
            "recorded_scan_stamp_ns":123,
            "moving_constraint_tracks":1,
            "post_hoc_reference_comparison":True,
            "status":"FEASIBLE_CBF_CONSTRAINED_CANDIDATE_UNVERIFIED",
            "model_input_pose_map_xyz_m":[0,0,.9],
            "nominal_xy_m":[[0,0],[1,0]],
            "candidate_xy_m":[[0,0],[.5,.4]],
        }
        got=reference_result(msg)
        self.assertIsNotNone(got)
        self.assertTrue(got["posthoc"])
        self.assertEqual(got["stamp"],123)
        self.assertNotEqual(got["nominal"],got["candidate"])
        msg["is_new_solver_input"]=False
        self.assertIsNone(reference_result(msg))
        msg["is_new_solver_input"]=True
        msg["safety_authorization"]="YES"
        self.assertIsNone(reference_result(msg))

    def test_empty_candidate_invalid(self):
        msg={"schema":"n01.offline.g3_mpc_cbf.v1",
             "is_new_solver_input":True,"solver_executed":True,
             "safety_authorization":"NEVER",
             "actuator_output":"DISABLED",
             "recorded_scan_stamp_ns":333,
             "moving_constraint_tracks":0,
             "nominal_xy_m":[[0,0],[1,0]],
             "candidate_xy_m":[]}
        self.assertIsNone(reference_result(msg))


class DemoReadOnlySourceTests(unittest.TestCase):
    def test_only_marker_and_nominal_candidate_path_pubs(self):
        text=(PKG/"demo_markers_node.py").read_text(encoding="utf-8")
        self.assertIn('"/n01/offline/demo_markers"',text)
        self.assertIn('"/n01/offline/demo_nominal_path"',text)
        self.assertIn('"/n01/offline/demo_candidate_path"',text)
        self.assertNotIn("create_publisher(Twist",text)
        self.assertIn('"N01_HISTORICAL_BAG_ONLY"',text)
        self.assertIn("scene[\"stamp\"]",text)
        self.assertIn("POST-HOC HISTORICAL COMPARISON",text)

    def test_rviz_topics_explicit_and_not_control(self):
        cfg=(PKG.parent.parent.parent/"rviz"/"n01_full_pipeline.rviz")
        text=cfg.read_text(encoding="utf-8")
        for topic in ("/cloud","/grid_map/occupancy_inflate",
                      "/nav/test/global_path","/n01/offline/demo_markers",
                      "/n01/offline/demo_nominal_path",
                      "/n01/offline/demo_candidate_path"):
            self.assertIn(topic,text)
        self.assertNotIn("/cmd_vel",text)


if __name__=="__main__":
    unittest.main()
