#!/usr/bin/env python3
"""G4.1 offline PCD floor-slice diagnostics. Never selects a floor automatically."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"/"n01_offline_integration"))
from n01_offline_integration.pcd_static_preview import sample_pcd
from n01_offline_integration.pcd_floor_diagnostics import floor_height_sweep


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--step",type=float,default=.25)
    parser.add_argument("--resolution",type=float,default=.25)
    parser.add_argument("--floor-tolerance",type=float,default=.12)
    parser.add_argument("--obstacle-min-z",type=float,default=.2)
    parser.add_argument("--obstacle-max-z",type=float,default=1.8)
    parser.add_argument("--clearance",type=float,default=.5)
    args=parser.parse_args(argv)
    pts,meta=sample_pcd(args.pcd)
    report=floor_height_sweep(pts,z_step=args.step,resolution=args.resolution,
                            floor_tolerance=args.floor_tolerance,
                            obstacle_min_z=args.obstacle_min_z,
                            obstacle_max_z=args.obstacle_max_z,
                            clearance=args.clearance)
    report["source_pcd"]=str(args.pcd.resolve())
    report["source_pcd_points"]=meta["count"]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("x",encoding="utf-8") as f:
        json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
        f.write("\n")
    print("G4.1 FLOOR HYPOTHESIS DIAGNOSTICS ONLY; no automatic floor selection")
    print(" z(m)   floor_hits  free   largest_4conn   unknown%")
    for row in report["rows"]:
        print(f"{row['floor_z_hypothesis_m']:6.2f} "
              f"{row['floor_hits']:11d} {row['free_cells']:6d} "
              f"{row['largest_free_component_cells']:14d} "
              f"{row['unknown_percent']:9.2f}")
    print("Report:",args.output)
    print("Important: ceiling/roof/pipe-top may have a larger component than actual floor.")
    print("All candidates remain UNVERIFIED, no path or robot motion output.")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
