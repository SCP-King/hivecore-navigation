"""G4.13 source contracts: historical 0.30m map profile + live read-only audit.

G4.13 is isolated historical playback ONLY; it is NOT permission to drive.
"""
import ast
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"scripts"))
from n01_live_localization_readonly_audit import classify


class MappingProfileAndLiveAuditTest(unittest.TestCase):
    def _snapshot(self,mount_edges):
        return {"mount_edges":mount_edges,"tf_inspection_errors":[],
                "map_to_base_footprint_seen":True,
                "last_localization_status":{
                    "outer":"localization","inner":"tracking",
                    "registration_tf_published":True,
                    "received_during_session":True,"age_at_receipt_s":0.1},
                "last_filtered_cloud":{"frame":"lidar_link","stamp_ns":123},
                "cmd_vel_publishers":[]}

    def test_live_history_double_mount_is_blocked(self):
        s=self._snapshot([{"topic":"/tf","height_m":0.30},
                          {"topic":"/tf_static","height_m":0.90}])
        self.assertEqual(classify(s)["result"],"BLOCKED_CONFLICTING_MOUNT_TF")

    def test_online_single_observed_edge_not_claim_single_author(self):
        s=self._snapshot([{"topic":"/tf_static","height_m":0.90}])
        assessment=classify(s)
        self.assertIn("PRECHECK_PASSED",assessment["result"])
        self.assertFalse(assessment["unique_mount_tf_publisher_attributed"])
        self.assertFalse(assessment["pcd_map_alignment_verified"])
        self.assertFalse(assessment["robot_motion_allowed"])
        s["last_localization_status"]["registration_tf_published"]=False
        self.assertIn("STATUS",classify(s)["result"])
        s["last_localization_status"]["registration_tf_published"]=True
        s["last_localization_status"]["age_at_receipt_s"]=20
        self.assertIn("STATUS",classify(s)["result"])

    def test_historical_mode3_planner_and_reference_share_z_offset(self):
        p=ROOT/"src/n01_offline_integration/launch/bag_pipeline.launch.py"
        q=ROOT/"src/n01_offline_integration/launch/g4_offline_scan_reference.launch.py"
        for file in (p,q):
            s=file.read_text()
            ast.parse(s)
            self.assertIn('LaunchConfiguration("scan_body_height_m")',s)
            self.assertIn('LaunchConfiguration("scan_ground_height_m")',s)
            self.assertIn('historical_mount_profile',s)
        script=(ROOT/"scripts/run_n01_g4_full_chain.sh").read_text()
        self.assertIn('N01_G4_MAPPING_SMOKE',script)
        self.assertIn('N01_G4_GROUND_Z_MAP_M="-0.55"',script)
        self.assertIn('N01_G4_SCAN_BODY_HEIGHT_M="0.85"',script)
        self.assertIn('N01_G4_SCAN_GROUND_HEIGHT_M="-0.55"',script)
        self.assertIn('MAPPING_SLAM_DYNAMIC_0P30',script)
        self.assertIn('RESEARCH_MAPPING_SMOKE',script)
        self.assertIn('BLOCKED_MIXED_REAL_GROUND',script)
        self.assertIn('"floor_verified"]=False',script)
        runner=(ROOT/"scripts/run_n01_bag_functional.sh").read_text()
        self.assertIn('historical_mount_profile:=',runner)
        self.assertIn('scan_ground_height_m:=',runner)
        self.assertIn('scan_body_height_m:=',runner)

    def test_static_map_bounds_and_true_window_filter(self):
        cfg=(ROOT/"src/n01_static_map/config/static_map.yaml").read_text()
        self.assertIn("min_z: -1.75",cfg)
        self.assertIn("max_z: 3.35",cfg)
        node=(ROOT/"src/n01_static_map/n01_static_map/static_map_node.py").read_text()
        ast.parse(node)
        self.assertIn("in_window = (zs >= self._min_z) & (zs <= self._max_z)",node)
        self.assertIn('excluded={excluded}',node)

    def test_live_probe_is_read_only_and_records_provenance(self):
        p=ROOT/"scripts/n01_live_localization_readonly_audit.py"
        src=p.read_text()
        ast.parse(src)
        self.assertNotIn("create_publisher(",src)
        self.assertNotIn("create_client(",src)
        self.assertNotIn("set_parameters(",src)
        self.assertNotIn("cmd_vel_pub",src)
        self.assertIn('"/localization_status"',src)
        self.assertIn('"/tf_static"',src)
        self.assertIn('"unique_mount_tf_publisher_attributed":False',src)


if __name__=="__main__":
    unittest.main()
