#!/usr/bin/env python3
"""G4 offline PCD -> grid/map image and optional A* reference preview.

NO ROS node, no robot topic, not SCAN's native spline output.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "n01_offline_integration"))
from n01_offline_integration.pcd_static_preview import sample_pcd
from n01_offline_integration.pcd_goal_grid import project, route, save_outputs
from n01_offline_integration.pcd_floor_diagnostics import free_component_stats


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path,
                        help="Required for generating .pgm/.yaml/.svg/.json outputs")
    parser.add_argument("--floor-z", type=float,
                        help="Manually verified floor z in PCD map frame; omit for Z histogram only")
    parser.add_argument("--resolution", type=float, default=.25)
    parser.add_argument("--floor-tolerance", type=float, default=.12)
    parser.add_argument("--obstacle-min-z", type=float, default=.2)
    parser.add_argument("--obstacle-max-z", type=float, default=1.8)
    parser.add_argument("--clearance", type=float, default=.5,
                        help="Research-only assumed inflation, NOT Hive2 certified envelope")
    parser.add_argument("--start", nargs=2, type=float, metavar=("X", "Y"))
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"))
    parser.add_argument("--unknown-policy", choices=("block", "optimistic"),
                        default="block")
    parser.add_argument("--i-accept-unverified-unknown", action="store_true",
                        help="Allow A* through unobserved cells, visualization ONLY")
    args = parser.parse_args(argv)
    if (args.start is None) != (args.goal is None):
        parser.error("--start and --goal must be specified together")
    if args.unknown_policy == "optimistic" and not args.i_accept_unverified_unknown:
        parser.error("optimistic unknown routing requires --i-accept-unverified-unknown")
    points, meta = sample_pcd(args.pcd)
    hist = Counter(round(p[2]*4)/4 for p in points)
    print(json.dumps({
        "pcd_format": meta["format"], "source_points": meta["count"],
        "sampled_points": len(points), "bounds_xyz_m": meta["bounds_xyz_m"],
        "common_z_bins_0_25m": hist.most_common(16),
        "note": "Z histogram does not verify floor elevation or traversability",
    }, ensure_ascii=False, indent=2))
    if args.floor_z is None:
        print("INSPECTION ONLY: verify PCD floor z; then specify --floor-z for map projection.")
        return 0
    if not args.output_prefix:
        parser.error("--output-prefix required when --floor-z is provided")
    mapdata = project(
        points, resolution=args.resolution, floor_z=args.floor_z,
        floor_tolerance=args.floor_tolerance,
        obstacle_min_z=args.obstacle_min_z,
        obstacle_max_z=args.obstacle_max_z, clearance=args.clearance)
    component_stats = free_component_stats(mapdata)
    path = None
    error = None
    if args.start is not None:
        try:
            path = route(mapdata, args.start, args.goal,
                         unknown_policy=args.unknown_policy)
        except ValueError as exc:
            error = str(exc)
    summary = save_outputs(mapdata, args.output_prefix, path)
    reportfile = args.output_prefix.with_suffix(".json")
    summary.update(component_stats)
    summary.update({
        "requested_start_xy_m": args.start,
        "requested_goal_xy_m": args.goal,
        "unknown_policy": args.unknown_policy,
        "unknown_traversal_is_optimistic_unverified": args.unknown_policy == "optimistic",
        "route_error": error,
    })
    reportfile.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n",
                          encoding="utf-8")
    print(f"MAP: {args.output_prefix.with_suffix('.pgm')}")
    print(f"MAP YAML: {args.output_prefix.with_suffix('.yaml')}")
    print(f"PREVIEW: {args.output_prefix.with_suffix('.svg')}")
    print(f"REPORT: {reportfile}")
    print("SCAN NATIVE LOCAL PLANNER NOT YET CONNECTED; NO MOTION COMMANDS.")
    if component_stats["largest_free_component_cells"] < 25:
        print("MAP_QUALITY_WARNING: largest 4-connected candidate free region has "
              f"only {component_stats['largest_free_component_cells']} cells; "
              "do NOT pick arbitrary goals. Verify floor height and scan coverage.")
    if error:
        print("ROUTE_NOT_FOUND:", error, file=sys.stderr)
        return 3
    if path:
        print(f"A_STAR_REFERENCE_PREVIEW_ONLY length_m={path['length_m']}")
    else:
        print("MAP_ONLY; provide explicit --start X Y --goal X Y for a route preview.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
