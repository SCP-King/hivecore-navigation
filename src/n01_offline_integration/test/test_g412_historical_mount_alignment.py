"""G4.12: original-map vs motion-stack mount consistency, pure geometry.

This is not a localization implementation, does not modify TF, and never
calls ROS or a motor controller.
"""
import ast
import math
from pathlib import Path
import sys
import unittest

REPO=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(REPO/"src/n01_offline_integration"))
from n01_offline_integration.g412_historical_mount_alignment import (
    audit_mapping_mount_alignment,
)
from n01_offline_integration.historical_tf import mapped_mounts


def report(quat=None):
    if quat is None:
        quat=[0.,0.,0.,1.]
    return {
        "schema":"n01.offline.g48_bag_anchored_preflight.v1",
        "ground_verified":False,
        "first_accepted_bag_pose":{
            "map_to_base_footprint_xyz_m":[-.002764,-.000130,.003491],
            "map_to_base_footprint_quaternion_xyzw":quat,
        }
    }


class MappingMountCompatibilityTest(unittest.TestCase):
    def test_level_0p30_vs_0p90_exact_shift_and_compensation(self):
        d=audit_mapping_mount_alignment(report())
        self.assertAlmostEqual(d["historical_candidate_map_base_link_xyz_m"][2],.303491)
        self.assertAlmostEqual(d["legacy_hive2_mount_unmodified_map_base_link_xyz_m"][2],.903491)
        self.assertAlmostEqual(d["legacy_lidar_position_shift_magnitude_m"],.6)
        self.assertAlmostEqual(d["hypothetical_corrected_map_base_footprint_xyz_m"][2],-.596509)
        self.assertAlmostEqual(d["hypothetical_corrected_lidar_xyz_m"][2],.303491)
        self.assertEqual(d["hypothetical_new_lidar_residual_m"],0.)
        self.assertEqual(d["local_floor_candidate_from_user_report_map_z_interval_m"],[-.6,-.5])
        self.assertFalse(d["automatic_live_tf_adjustment_allowed"])
        self.assertFalse(d["true_floor_z_map_verified"])
        self.assertEqual(d["actuator_output"],"DISABLED")
        self.assertEqual(d["safety_authorization"],"NEVER")

    def test_tilt_requires_full_orientation_not_only_subtracting_world_z(self):
        theta=.3
        q=[0.,math.sin(theta/2),0.,math.cos(theta/2)]
        d=audit_mapping_mount_alignment(report(q))
        self.assertGreater(abs(d["hypothetical_correction_map_xyz_m"][0]),.1)
        self.assertAlmostEqual(d["legacy_lidar_position_shift_magnitude_m"],.6,places=9)
        self.assertLess(d["hypothetical_new_lidar_residual_m"],1e-9)

    def test_invalid_bad_conflict_reports_fail_closed(self):
        with self.assertRaises(ValueError):
            audit_mapping_mount_alignment({})
        p=report()
        del p["first_accepted_bag_pose"]["map_to_base_footprint_quaternion_xyzw"]
        with self.assertRaises(ValueError):
            audit_mapping_mount_alignment(p)
        with self.assertRaises(ValueError):
            audit_mapping_mount_alignment(report([0.,0.,0.,0.]))

    def test_replay_stays_legacy_while_explicit_mapping_profile_is_available(self):
        pose=((0.,0.,0.),(0.,0.,0.,1.))
        self.assertEqual(mapped_mounts(pose)[0][0][2],.9)
        self.assertEqual(mapped_mounts(pose,mount_height_m=.3)[0][0][2],.3)
        with self.assertRaises(ValueError):
            mapped_mounts(pose,mount_height_m=.8)

    def test_historical_mapping_replay_opt_in_has_provenance_and_default_preserved(self):
        node=(REPO/"src/n01_offline_integration/n01_offline_integration/historical_bag_node.py").read_text()
        ast.parse(node)
        self.assertIn('param("historical_mount_profile","LEGACY_HIVE2_0P90")',node)
        self.assertIn('self.mount_profile=="MAPPING_SLAM_DYNAMIC_0P30"',node)
        self.assertIn('MAPPING_PROFILE_REQUIRES_ORIGINAL_DYNAMIC_0P30_TF',node)
        self.assertIn('mapped_mounts(pose,mount_height_m=self.mount_height_m)',node)
        self.assertNotIn('create_publisher(Twist',node)
        for launchname in ("bag_pipeline.launch.py", "g4_offline_scan_reference.launch.py"):
            src=(REPO/"src/n01_offline_integration/launch"/launchname).read_text()
            ast.parse(src)
            self.assertIn('historical_mount_profile',src)
            self.assertIn('default_value="LEGACY_HIVE2_0P90"',src)

    def test_g48_preflight_exports_independent_evidence(self):
        path=REPO/"scripts/n01_g4_bag_start_preflight.py"
        source=path.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('"tf_mount_edge_evidence_by_topic"',source)
        self.assertIn('"historical_mapping_candidate_mount_height_m": 0.30',source)
        self.assertIn('"g412_historical_mount_alignment.json"',source)
        self.assertIn('"map_to_base_footprint_quaternion_xyzw"',source)
        self.assertNotIn("create_publisher(",source)


if __name__=="__main__":
    unittest.main()
