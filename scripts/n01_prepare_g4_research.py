#!/usr/bin/env python3
"""G4 research preparation: existing PCD & Bag evidence -> route, geometry, SCAN JSON.

Pure offline file I/O. Does not launch SCAN, ROS, robot, or motion simulation.
All reported clearances are unverified observational heuristics.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"/"n01_offline_integration"))
from n01_offline_integration.pcd_static_preview import sample_pcd
from n01_offline_integration.pcd_bag_evidence_audit import audit,write_outputs
from n01_offline_integration.scan_reference_artifact import prepare_reference,save_reference
from n01_offline_integration.pcd_route_geometry_audit import audit_pcd_path


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ray-prefix",type=Path,required=True,
                   help="G4.2 ray visibility .pgm/.json prefix")
    p.add_argument("--pcd-prefix",type=Path,required=True,
                   help="G4.0 PCD floor-slice .pgm/.json prefix")
    p.add_argument("--pcd",type=Path,required=True,
                   help="Unmodified real PCD for G4.4 route height-band audit")
    p.add_argument("--output-dir",type=Path,required=True)
    p.add_argument("--ground-z-assumption",type=float,required=True,
                   help="NOT verified ground, e.g. -1.0 only if manually inspected")
    p.add_argument("--reference-z-basis",choices=(
        "MANUAL_INSPECTED_GROUND_Z_UNVERIFIED",
        "EXPLICIT_FLAT_MAP_Z0_OFFLINE_SMOKE_NOT_GROUND"),
        default="MANUAL_INSPECTED_GROUND_Z_UNVERIFIED",
        help="Must not imply that reference Z is verified ground")
    p.add_argument("--research-route",action="store_true",
                   help="Use farthest cells in G4.3 connected ray-visible research patch")
    p.add_argument("--start",type=float,nargs=2,metavar=("X","Y"))
    p.add_argument("--goal",type=float,nargs=2,metavar=("X","Y"))
    args=p.parse_args(argv)
    if (args.start is None)!=(args.goal is None):
        p.error("numeric --start and --goal must be supplied together")
    if not args.research_route and args.start is None:
        p.error("provide --research-route or explicit numeric --start X Y --goal X Y")
    # Refuse accidental overwrite of previous evidence/provenance.
    args.output_dir.mkdir(parents=True,exist_ok=False)
    evidence,grid,path,conflicts=audit(
        args.ray_prefix,args.pcd_prefix,
        research_route=args.research_route,
        start_xy=args.start,goal_xy=args.goal)
    write_outputs(args.output_dir/"g43_evidence",evidence,grid,path,conflicts)
    if not path:
        print("G4.3 route cannot be formed:",evidence.get("route_error","no reference"))
        print("Saved conflict map:",args.output_dir/"g43_evidence.svg")
        return 3
    if (args.reference_z_basis=="EXPLICIT_FLAT_MAP_Z0_OFFLINE_SMOKE_NOT_GROUND" and
        args.ground_z_assumption != 0.0):
        p.error("flat map Z0 smoke requires exactly --ground-z-assumption 0")
    ref=prepare_reference(evidence,ground_z=args.ground_z_assumption)
    ref["reference_z_basis"]=args.reference_z_basis
    ref["reference_z_is_measured_floor"]=False
    save_reference(args.output_dir/"g45_scan_reference.json",ref)
    points,meta=sample_pcd(args.pcd)
    geometry=audit_pcd_path(points,ref)
    geometry["source_pcd"]=str(args.pcd.resolve())
    geometry["source_pcd_point_count"]=meta["count"]
    with (args.output_dir/"g44_geometry_audit.json").open("x",encoding="utf-8") as f:
        json.dump(geometry,f,ensure_ascii=False,indent=2,allow_nan=False)
        f.write("\n")
    manifest={
        "schema":"n01.offline.g4_research_pipeline_manifest.v1",
        "result":"UNVERIFIED_REFERENCE_PREPARED_NOT_SCAN_EXECUTED",
        "g43_evidence_report":"g43_evidence.json",
        "g43_conflicts_preview":"g43_evidence.svg",
        "g44_pcd_geometry_report":"g44_geometry_audit.json",
        "g45_reference_for_isolated_scan":"g45_scan_reference.json",
        "reference_waypoints":ref["waypoint_count"],
        "research_length_m":ref["path_geometry_length_m"],
        "floor_z_assumption_m":ref["ground_z_assumption_map_m"],
        "reference_z_basis":ref["reference_z_basis"],
        "reference_z_is_measured_floor":False,
        "pcd_surface_conflict_waypoints":geometry["pcd_surface_conflict_waypoints"],
        "unknown_ground_support_waypoints":geometry["unknown_ground_support_waypoints"],
        "floor_verified":False,
        "full_robot_clearance_verified":False,
        "scan_native_bspline_recorded":False,
        "safe_for_real_robot":False,
        "actuator_output":"DISABLED",
        "safety_authorization":"NEVER",
    }
    with (args.output_dir/"manifest.json").open("x",encoding="utf-8") as f:
        json.dump(manifest,f,ensure_ascii=False,indent=2,allow_nan=False)
        f.write("\n")
    print("G4.3/G4.4/G4.5 RESEARCH ARTIFACTS CREATED:",args.output_dir)
    print("A-star reference length:",ref["path_geometry_length_m"],"m")
    print("Research reference samples:",ref["waypoint_count"])
    print("PCD surface-conflict path samples:",
          geometry["pcd_surface_conflict_waypoints"])
    print("Unknown floor-support path samples:",
          geometry["unknown_ground_support_waypoints"])
    print("SCAN not run. Never call this a verified driveable route.")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
