"""G4.10 offline route PCD ambiguity triage: never infer traversable ground."""
import ast
import pathlib
import sys
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"src/n01_offline_integration"))
from n01_offline_integration.g410_route_surface_ambiguity import audit_surface_ambiguity


def row(i, *, n, q=None):
    return {
        "index":i,"research_xy_m":[float(i),0.0],
        "pcd_samples_within_xy_radius":n,
        "surface_z_quantiles_m":q,
        "dominant_height_bins":[] if n==0 else [
            {"z_interval_m":[0.0,0.1],"pcd_samples":n}],
        "map_ground_z_m":None,
    }


def g49(rows):
    return {
        "schema":"n01.offline.g49_pcd_route_surface_height_evidence.v1",
        "rows":rows,"route_waypoints":len(rows),
        "ground_z_selected_m":None,
        "support_continuity_verified":False,
    }


def q(minimum,p10,p25,med,p75,p90,maximum):
    return {"min":minimum,"p10":p10,"p25":p25,"median":med,
            "p75":p75,"p90":p90,"max":maximum}


class G410SurfaceAmbiguityTests(unittest.TestCase):
    def test_high_layer_spread_and_unknown_do_not_become_floor(self):
        evidence=g49([
            row(0,n=711,q=q(-.566,-.527,-.501,2.594,2.693,2.715,2.755)),
            row(1,n=15,q=q(-.51,-.505,-.5,-.49,-.48,-.47,-.465)),
            row(2,n=0)])
        audit=audit_surface_ambiguity(evidence)
        self.assertEqual(audit["route_waypoints"],3)
        self.assertEqual(audit["waypoints_multi_height_or_high_z_variation"],1)
        self.assertEqual(audit["waypoints_no_nearby_pcd"],1)
        self.assertEqual(audit["waypoints_other_surface_evidence_not_ground_verified"],1)
        self.assertTrue(audit["classifications_account_for_all_waypoints"])
        self.assertEqual(audit["rows"][0]["surface_evidence_classification"],
                         "MULTI_LEVEL_OR_HIGH_VARIATION_SURFACE_EVIDENCE")
        self.assertEqual(audit["rows"][2]["surface_evidence_classification"],
                         "UNKNOWN_NO_LOCAL_PCD_SAMPLES")
        self.assertIsNone(audit["ground_z_map_m"])
        self.assertFalse(audit["route_to_scan_allowed"])
        self.assertFalse(audit["wheel_ground_contact_verified"])
        self.assertFalse(audit["full_body_collision_clearance_verified"])
        self.assertEqual(audit["safety_authorization"],"NEVER")

    def test_bad_samples_guess_floor_or_inconsistent_quantiles_block(self):
        rows=[
            row(0,n=100,q=q(-1.,-.8,-.6,0.,.1,.3,.4)),
            row(1,n=20,q=q(-1.,-.8,-.6,-.5,-.4,-.3,-.2))
        ]
        data=g49(rows)
        data["ground_z_selected_m"]=-.5
        with self.assertRaises(ValueError):
            audit_surface_ambiguity(data)
        data["ground_z_selected_m"]=None
        data["rows"][0]["map_ground_z_m"]=-.5
        with self.assertRaises(ValueError):
            audit_surface_ambiguity(data)
        data["rows"][0]["map_ground_z_m"]=None
        data["rows"][0]["surface_z_quantiles_m"]["median"]=float("nan")
        with self.assertRaises(ValueError):
            audit_surface_ambiguity(data)

    def test_not_enough_usable_evidence_is_not_accepted(self):
        with self.assertRaises(ValueError):
            audit_surface_ambiguity(g49([row(0,n=0)]))
        with self.assertRaises(ValueError):
            audit_surface_ambiguity(g49([row(0,n=0),row(1,n=1)]))
        with self.assertRaises(ValueError):
            audit_surface_ambiguity(g49([row(0,n=0),row(1,n=0)]),
                                    wide_iqr_threshold_m=0.0)

    def test_g4_preflight_adds_g410_but_does_not_unlock_scan(self):
        source=(ROOT/"scripts/n01_g4_bag_start_preflight.py").read_text()
        ast.parse(source)
        self.assertIn("g410_route_surface_ambiguity.json",source)
        self.assertIn('manifest["g410_route_to_scan_allowed"] = False',source)
        self.assertIn('"route_to_scan_executed": False',source)
        self.assertNotIn("cmd_vel",source)


if __name__=="__main__":
    unittest.main()
