"""G4.15 minimal regression for real native SCAN static PCD obstacle consumption.

No robot, no Bag rerun; this checks the source connections and independent
evidence object's behavior. Actual ROS C++ build is done on Ubuntu Humble.
"""
import ast
import pathlib
import sys
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"src/n01_offline_integration"))
from n01_offline_integration.g4_chain_evidence import ChainEvidence

def reference():
    return {
      "schema":"n01.offline.scan_reference.v1","frame_id":"map",
      "path_xyz_m":[[0.,0.,-.55],[1.,0.,-.55]],
      "safety_authorization":"NEVER","actuator_output":"DISABLED",
      "physical_execution_forbidden":True,
    }

class NativeStaticOverlayFeatureTest(unittest.TestCase):
    def test_native_scan_consumes_actual_static_map_not_just_publishes_it(self):
        cpp=(ROOT/"src/scan_planner/plan_env/src/grid_map.cpp").read_text()
        hdr=(ROOT/"src/scan_planner/plan_env/include/plan_env/grid_map.h").read_text()
        self.assertIn('grid_map.static_cloud_enabled',cpp)
        self.assertIn('"/grid_map/static_occupancy"',cpp)
        self.assertIn('GridMap::staticCloudCallback',cpp)
        self.assertIn('static_source_cloud_ = cloud;',cpp)
        self.assertIn('GridMap::rebuildStaticOverlay()',cpp)
        self.assertIn('rebuildStaticOverlay(); // retained PCD reprojected',cpp)
        self.assertIn('"/grid_map/static_overlay_ready"',cpp)
        self.assertIn('static_occupancy_buffer_[addr]',hdr)
        self.assertIn('static_inflate_buffer_[adr]',hdr)
        self.assertIn('getInflateOccupancyFromBuffer(front, static_inflate_buffer_)',hdr)

    def test_native_static_map_must_load_before_research_reference(self):
        chain=ChainEvidence(reference())
        chain.static_pcd_required=True
        chain.initial_path("map",reference()["path_xyz_m"])
        r=chain.finalize()
        self.assertIn('NATIVE_SCAN_STATIC_PCD_NOT_READY_BEFORE_G4_REFERENCE',r["issues"])
        self.assertFalse(r["native_scan_static_pcd_occupancy_loaded_before_reference"])
        chain2=ChainEvidence(reference())
        chain2.static_pcd_required=True
        chain2.native_static_pcd_status(True)
        chain2.initial_path("map",reference()["path_xyz_m"])
        r2=chain2.finalize()
        self.assertNotIn('NATIVE_SCAN_STATIC_PCD_NOT_READY_BEFORE_G4_REFERENCE',r2["issues"])
        self.assertTrue(r2["native_scan_static_pcd_occupancy_loaded_before_reference"])
        self.assertFalse(r2["static_full_pcd_integrated_into_scan_grid"])
        self.assertFalse(r2["terrain_and_collision_verified"])

    def test_isolated_bag_ros_launch_builds_supplier_and_planner(self):
        launch=(ROOT/"src/n01_offline_integration/launch/g4_offline_scan_reference.launch.py").read_text()
        ref=(ROOT/"src/n01_offline_integration/n01_offline_integration/g4_reference_node.py").read_text()
        ast.parse(launch);ast.parse(ref)
        self.assertIn('executable="static_map_node"',launch)
        self.assertIn('scan_static_pcd_enable',launch)
        self.assertIn('static_pcd_path',launch)
        self.assertIn('require_static_overlay',launch)
        self.assertIn('self.static_ready',ref)
        self.assertIn('WAITING_NATIVE_SCAN_STATIC_PCD_OVERLAY',ref)
        full=(ROOT/"scripts/run_n01_g4_full_chain.sh").read_text()
        runner=(ROOT/"scripts/run_n01_bag_functional.sh").read_text()
        self.assertIn('N01_G4_STATIC_PCD_ENABLE="true"',full)
        self.assertIn('n01_static_map',runner)
        self.assertIn('scan_static_pcd_enable:=',runner)
        self.assertNotIn('cmd_vel',ref.split('self.pub=self.create_publisher(Path')[1].split('self.status_pub=')[0])

if __name__=="__main__":
    unittest.main()
