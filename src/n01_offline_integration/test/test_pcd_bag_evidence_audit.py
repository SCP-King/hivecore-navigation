"""G4.3 offline PCD/Bag grid conflict + A* audit regression tests."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from n01_offline_integration.pcd_bag_evidence_audit import (
    audit, read_export, FREE, UNKNOWN, OCC)


def export_grid(prefix, raster, schema, origin=[0., 0.], resolution=.25):
    w = h = 6
    prefix = Path(prefix)
    # Input pixels are top row first; PGM P5 exactly as produced by G4.0/G4.2.
    prefix.with_suffix(".pgm").write_bytes(
        f"P5\n{w} {h}\n255\n".encode() + bytes(raster))
    prefix.with_suffix(".json").write_text(json.dumps({
        "schema": schema, "width": w, "height": h,
        "origin": origin, "resolution": resolution,
        "free_cells": raster.count(254),
        "unknown_cells": raster.count(205),
        "inflated_occupied_cells": raster.count(0),
    }), encoding="utf-8")


class PcdBagEvidenceAuditTest(unittest.TestCase):
    def _inputs(self, folder):
        ray = Path(folder) / "ray"
        pcd = Path(folder) / "pcd"
        # A 5x5 free patch. One PCD slice obstacle conflicts with ray free.
        a = [205] * 36
        b = [205] * 36
        for y in range(5):
            for x in range(5):
                a[y*6+x] = 254
        b[2*6+2] = 0
        export_grid(ray, a, "n01.offline.bag_ray_visibility.v1")
        export_grid(pcd, b, "n01.offline.pcd_goal_preview.v1")
        return ray, pcd

    def test_conflict_is_unknown_not_free_and_route_avoids_it(self):
        with tempfile.TemporaryDirectory() as temp:
            ray, pcd = self._inputs(temp)
            report, grid, path, conflicts = audit(ray, pcd, research_route=True)
            self.assertEqual(report["ray_free_pcd_slice_obstacle_conflicts"], 1)
            self.assertEqual(report["free_after_conflict_veto"], 24)
            self.assertEqual(report["largest_free_component_cells"], 24)
            self.assertTrue(path)
            self.assertFalse(any(i in conflicts for i in path))
            self.assertFalse(report["scan_native_planner_connected"])
            self.assertEqual(report["actuator_output"], "DISABLED")

    def test_start_goal_unknown_fail_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            ray, pcd = self._inputs(temp)
            report, _, _, _ = audit(ray, pcd, start_xy=(.61, .86), goal_xy=(.15,.15))
            self.assertIn("route_error", report)
            self.assertNotIn("research_path_length_m", report)

    def test_outside_map_reports_error_not_traceback(self):
        with tempfile.TemporaryDirectory() as temp:
            ray, pcd = self._inputs(temp)
            report, _, path, _ = audit(ray, pcd,
                                       start_xy=(99.,99.), goal_xy=(.15,.15))
            self.assertEqual(path, [])
            self.assertIn("outside map", report.get("route_error",""))

    def test_mismatched_origin_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            ray, pcd = self._inputs(temp)
            info = json.loads(pcd.with_suffix(".json").read_text())
            info["origin"] = [100., 0.]
            pcd.with_suffix(".json").write_text(json.dumps(info))
            with self.assertRaisesRegex(ValueError, "identical map XY lattice"):
                audit(ray, pcd)

    def test_bad_raster_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            ray, pcd = self._inputs(temp)
            ray.with_suffix(".pgm").write_bytes(b"P5\n6 6\n255\n" + b"\x00")
            with self.assertRaisesRegex(ValueError, "PGM"):
                read_export(ray)

    def test_source_never_issues_motion(self):
        code = (ROOT/"n01_offline_integration"/"pcd_bag_evidence_audit.py").read_text()
        self.assertNotIn("rclpy", code)
        self.assertNotIn("create_publisher", code)
        self.assertNotIn("/cmd_vel", code)


if __name__ == "__main__":
    unittest.main()
