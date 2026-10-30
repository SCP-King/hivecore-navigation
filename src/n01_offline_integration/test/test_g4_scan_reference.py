"""G4.4/G4.5/G4.6 offline-only research interface contract tests."""
import ast
import pathlib
import sys
import tempfile
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from n01_offline_integration.scan_reference_artifact import (
    prepare_reference,save_reference,load_and_prepare)
from n01_offline_integration.pcd_route_geometry_audit import audit_pcd_path

PROVENANCE="A_STAR_ON_RAY_VISIBILITY_WITH_PCD_CONFLICT_VETO_NOT_SCAN"


def fake_g43(path=None):
    if path is None:
        path=[[0.,0.],[.25,0.],[.5,0.],[.5,.25]]
    import math
    distance=sum(math.dist(a,b) for a,b in zip(path,path[1:]))
    return {
        "schema":"n01.offline.pcd_bag_evidence_audit.v1",
        "status":"CONFLICT_AWARE_RAY_VISIBILITY_ONLY_NOT_TRAVERSABILITY",
        "safety_authorization":"NEVER","actuator_output":"DISABLED",
        "global_reference_algorithm":PROVENANCE,
        "research_path_length_m":distance,
        "research_path_xy_m":path,
        "resolution_m":.25,
        "research_start_xy_m":path[0],
        "research_goal_xy_m":path[-1],
    }


class G4ResearchPipelineTests(unittest.TestCase):
    def test_prepare_reference_bounded_and_labelled(self):
        ref=prepare_reference(fake_g43(),ground_z=-1.)
        self.assertEqual(ref["schema"],"n01.offline.scan_reference.v1")
        self.assertEqual(ref["path_xyz_m"][0],[0.,0.,-1.])
        self.assertEqual(ref["path_xyz_m"][-1],[.5,.25,-1.])
        self.assertFalse(ref["robot_full_height_clearance_verified"])
        self.assertFalse(ref["static_full_pcd_integrated_into_scan_grid"])
        self.assertTrue(ref["physical_execution_forbidden"])
        with tempfile.TemporaryDirectory() as d:
            f=pathlib.Path(d)/"reference.json"
            save_reference(f,ref)
            self.assertTrue(f.exists())
            with self.assertRaises(FileExistsError):
                save_reference(f,ref)

    def test_reject_disconnected_reference_path(self):
        with self.assertRaisesRegex(ValueError,"nonadjacent"):
            prepare_reference(fake_g43([[0.,0.],[2.,2.]]),ground_z=0.)

    def test_fail_closed_when_not_a_genuine_research_path(self):
        r=fake_g43()
        r["global_reference_algorithm"]="NONE"
        with self.assertRaisesRegex(ValueError,"no completed A-star"):
            prepare_reference(r,ground_z=0.)
        r=fake_g43()
        r["actuator_output"]="ENABLED"
        with self.assertRaisesRegex(ValueError,"safety"):
            prepare_reference(r,ground_z=0.)

    def test_geometry_audit_proves_no_physical_safety(self):
        reference=prepare_reference(fake_g43(),ground_z=0.)
        surface=[(0.,0.,0.),(.25,0.,0.),(.5,0.,0.),(.5,.25,0.)]
        result=audit_pcd_path(surface,reference)
        self.assertEqual(result["pcd_surface_conflict_waypoints"],0)
        self.assertEqual(result["unknown_ground_support_waypoints"],0)
        self.assertEqual(result["candidate_supported_no_sampled_obstacle_waypoints"],4)
        self.assertFalse(result["full_robot_clearance_verified"])
        self.assertFalse(result["floor_physically_verified"])
        surface.append((.5,.25,.8))
        result=audit_pcd_path(surface,reference)
        self.assertGreater(result["pcd_surface_conflict_waypoints"],0)

    def test_unsupported_ground_remains_unknown(self):
        ref=prepare_reference(fake_g43(),ground_z=-1.)
        surface=[(0.,0.,3.)]
        result=audit_pcd_path(surface,ref)
        self.assertEqual(result["unknown_ground_support_waypoints"],4)

    def test_source_and_launch_syntax_and_isolation(self):
        base=ROOT/"n01_offline_integration"
        candidates=[
            base/"scan_reference_artifact.py",
            base/"pcd_route_geometry_audit.py",
            base/"g4_reference_node.py",
            ROOT/"launch"/"g4_offline_scan_reference.launch.py",
        ]
        for p in candidates:
            src=p.read_text(encoding="utf-8")
            ast.parse(src,filename=str(p))
            self.assertNotIn("create_publisher(Twist",src)
            self.assertNotIn('"/cmd_vel",10',src)
        publish=(base/"g4_reference_node.py").read_text()
        self.assertIn('N01_G4_RESEARCH_ENABLE',publish)
        self.assertIn('get_publishers_info_by_topic("/cmd_vel")',publish)
        self.assertIn('self.pub.get_subscription_count()',publish)
        self.assertIn('gap>self.limit',publish)
        self.assertIn('self.pub.publish(result)',publish)
        launch=(ROOT/"launch"/"g4_offline_scan_reference.launch.py").read_text()
        self.assertIn('"scan_navi_mode":"3"',launch)
        self.assertIn('"N01_HISTORICAL_BAG_ONLY":"1"',launch)


if __name__=="__main__":
    unittest.main()
