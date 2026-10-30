#!/usr/bin/env python3
"""Offline G4.3 evidence -> G4.5 SCAN initial_path JSON, no ROS graph."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"/"n01_offline_integration"))
from n01_offline_integration.scan_reference_artifact import load_and_prepare,save_reference


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--g43-report",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--ground-z-assumption",type=float,required=True,
                   help="UNVERIFIED experimental map-frame ground elevation, not robot body z")
    p.add_argument("--max-points",type=int,default=200)
    args=p.parse_args(argv)
    ref=load_and_prepare(args.g43_report,
                         ground_z=args.ground_z_assumption,max_points=args.max_points)
    save_reference(args.output,ref)
    print("G4.5 RESEARCH REFERENCE JSON:",args.output)
    print("WAYPOINTS:",ref["waypoint_count"],
          "GEOMETRIC_LENGTH_M:",ref["path_geometry_length_m"])
    print("SCAN did NOT yet optimize this line; floor, clearance and collision unverified.")
    print("Never send this research reference to physical robot control.")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
