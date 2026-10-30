#!/usr/bin/env python3
"""G4.4 read-only PCD surface geometry audit of a G4.5 path, not a safety pass."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"/"n01_offline_integration"))
from n01_offline_integration.pcd_static_preview import sample_pcd
from n01_offline_integration.pcd_route_geometry_audit import audit_pcd_path


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pcd",type=Path,required=True)
    p.add_argument("--reference-json",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--floor-tolerance",type=float,default=.15)
    p.add_argument("--floor-support-radius",type=float,default=.35)
    p.add_argument("--obstacle-height-min",type=float,default=.2)
    p.add_argument("--obstacle-height-max",type=float,default=1.8)
    p.add_argument("--lateral-clearance",type=float,default=.45)
    args=p.parse_args(argv)
    points,meta=sample_pcd(args.pcd)
    ref=json.loads(args.reference_json.read_text(encoding="utf-8"))
    result=audit_pcd_path(
        points,ref,floor_tolerance=args.floor_tolerance,
        floor_support_radius=args.floor_support_radius,
        obstacle_height_min=args.obstacle_height_min,
        obstacle_height_max=args.obstacle_height_max,
        lateral_clearance_radius=args.lateral_clearance)
    result["source_pcd"]=str(args.pcd.resolve())
    result["source_reference"]=str(args.reference_json.resolve())
    result["original_pcd_sampled_points"]=meta["sampled_points"]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("x",encoding="utf-8") as handle:
        json.dump(result,handle,ensure_ascii=False,indent=2,allow_nan=False)
        handle.write("\n")
    print("G4.4 GEOMETRY EVIDENCE ONLY:",args.output)
    print("waypoints:",result["waypoints"])
    print("PCD surface conflicts:",result["pcd_surface_conflict_waypoints"])
    print("unknown floor support:",result["unknown_ground_support_waypoints"])
    print("candidate floor with no sampled obstacle:",
          result["candidate_supported_no_sampled_obstacle_waypoints"])
    print("NEVER treat this as real robot collision/ground certification.")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
