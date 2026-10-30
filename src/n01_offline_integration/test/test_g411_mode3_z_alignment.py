"""Source-checked mode3 reference height diagnostic, no ROS or floor assumption."""
import ast
import pathlib
import sys
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"src/n01_offline_integration"))
from n01_offline_integration.g411_mode3_z_alignment import check_mode3_z_alignment


def preflight(body_z=0.903491):
    return {
      "schema":"n01.offline.g48_bag_anchored_preflight.v1",
      "ground_verified":False,
      "first_accepted_bag_pose":{
          "offline_reconstructed_base_link_xyz_m":[0.,0.,body_z],
      }
    }


class Mode3HeightContractTest(unittest.TestCase):
    def test_z0_is_unverified_and_default_scan_mismatch_is_reported(self):
        r=check_mode3_z_alignment(preflight())
        self.assertEqual(r["scan_body_height_config_m"],.4)
        self.assertEqual(r["reference_z_hypothesis_m"],0.)
        self.assertAlmostEqual(r["reference_start_vertical_offset_m"],.503491,places=6)
        self.assertFalse(r["would_pass_reference_publisher_z_gate_on_these_recorded_values"])
        self.assertEqual(r["status"],"REFERENCE_HEIGHT_SEMANTICS_MISMATCH_FOR_HYPOTHESIS")
        self.assertFalse(r["real_map_floor_z_verified"])
        self.assertFalse(r["route_to_scan_executed"])
        self.assertEqual(r["safety_authorization"],"NEVER")

    def test_height_gate_agreement_is_not_traversability(self):
        report=check_mode3_z_alignment(preflight(.9),scan_body_height_m=.4)
        self.assertTrue(report["would_pass_reference_publisher_z_gate_on_these_recorded_values"])
        self.assertFalse(report["real_map_floor_z_verified"])
        self.assertFalse(report["hardware_control_authorized"])
        # A new height offset hypothesis doesn't change the actual floor evidence.
        report=check_mode3_z_alignment(preflight(.903491),scan_body_height_m=.9)
        self.assertTrue(report["would_pass_reference_publisher_z_gate_on_these_recorded_values"])
        self.assertFalse(report["real_map_floor_z_verified"])

    def test_invalid_source_and_misleading_verified_ground_are_refused(self):
        with self.assertRaises(ValueError):
            check_mode3_z_alignment({})
        data=preflight()
        data["ground_verified"]=True
        with self.assertRaises(ValueError):
            check_mode3_z_alignment(data)
        with self.assertRaises(ValueError):
            check_mode3_z_alignment(preflight(),assumed_reference_z_m=float("nan"))

    def test_real_cpp_and_g4_reference_gate_semantics_remain_auditable(self):
        fsm=(ROOT/"src/scan_planner/plan_manage/src/scan_replan_fsm.cpp").read_text()
        self.assertIn("pose_stamped.pose.position.z + body_height_",fsm)
        launch=(ROOT/"src/n01_offline_integration/launch/g4_offline_scan_reference.launch.py").read_text()
        self.assertIn('"scan_body_height_m":.4',launch)
        node=(ROOT/"src/n01_offline_integration/n01_offline_integration/g4_reference_node.py").read_text()
        self.assertIn('z_gap=abs(self.latest_body[2]-(pts[0][2]+self.body_height))',node)
        self.assertIn('if gap>self.limit or z_gap>self.z_limit:',node)
        ast.parse((ROOT/"src/n01_offline_integration/n01_offline_integration/g411_mode3_z_alignment.py").read_text())

if __name__=="__main__":
    unittest.main()
