#!/usr/bin/env python3
"""G4.3 offline evidence comparison from EXISTING PGM+JSON; no ROS, no Bag reread."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "n01_offline_integration"))
from n01_offline_integration.pcd_bag_evidence_audit import audit, write_outputs


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ray-prefix", type=Path, required=True,
                   help="G4.2 Bag ray PGM+JSON prefix (WITHOUT file extension)")
    p.add_argument("--pcd-prefix", type=Path, required=True,
                   help="G4.0 PCD height-slice PGM+JSON prefix")
    p.add_argument("--output-prefix", type=Path, required=True)
    p.add_argument("--research-route", action="store_true",
                   help="Automatically choose two distant connected ray-visible cells; NOT navigation goals")
    p.add_argument("--start", nargs=2, type=float, metavar=("X", "Y"))
    p.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"))
    args = p.parse_args()
    if (args.start is None) != (args.goal is None):
        p.error("--start and --goal must both contain numeric XY map coordinates")
    report, grid, path, conflict = audit(
        args.ray_prefix, args.pcd_prefix,
        research_route=args.research_route, start_xy=args.start, goal_xy=args.goal)
    write_outputs(args.output_prefix, report, grid, path, conflict)
    print("G4.3 REAL MAP EVIDENCE AUDIT - READ ONLY, NOT NAVIGATION")
    for key in ("ray_free_cells", "pcd_slice_free_cells",
                "ray_free_pcd_slice_obstacle_conflicts",
                "ray_hit_pcd_slice_free_conflicts",
                "free_after_conflict_veto", "largest_free_component_cells",
                "research_path_length_m", "route_error"):
        if key in report:
            print(key + ":", report[key])
    print("REPORT:", args.output_prefix.with_suffix(".json"))
    print("PREVIEW:", args.output_prefix.with_suffix(".svg"))
    print("No SCAN Bspline generated, no safe traversability asserted, no robot commands.")
    return 0 if not report.get("route_error") else 3


if __name__ == "__main__":
    raise SystemExit(main())
