"""G4.5: prepare a bounded map-frame path artifact from G4.3 research evidence.

This creates INPUT DATA for SCAN reference-path mode; it does not run SCAN,
certify terrain, publish commands, or simulate execution.
"""
import json
import math
from pathlib import Path

SCHEMA = "n01.offline.scan_reference.v1"
SOURCE = "n01.offline.pcd_bag_evidence_audit.v1"


def prepare_reference(report, *, ground_z, max_points=200):
    if report.get("schema") != SOURCE:
        raise ValueError("requires original G4.3 PCD/Bag evidence JSON")
    if report.get("status") != "CONFLICT_AWARE_RAY_VISIBILITY_ONLY_NOT_TRAVERSABILITY":
        raise ValueError("wrong G4.3 evidence status")
    if report.get("safety_authorization") != "NEVER" or report.get("actuator_output") != "DISABLED":
        raise ValueError("invalid research-only safety provenance")
    if report.get("global_reference_algorithm") != (
        "A_STAR_ON_RAY_VISIBILITY_WITH_PCD_CONFLICT_VETO_NOT_SCAN"
    ):
        raise ValueError("G4.3 contains no completed A-star research reference")
    if report.get("route_error") or report.get("research_path_length_m") is None:
        raise ValueError("research path not successfully computed")
    if not isinstance(ground_z, (float, int)) or not math.isfinite(ground_z):
        raise ValueError("finite manually inspected ground_z is required")
    if not -3. <= ground_z <= 4.:
        raise ValueError("ground_z outside bounded experimental interval")
    if type(max_points) is not int or not 2 <= max_points <= 1000:
        raise ValueError("invalid bounded reference path point limit")
    cells = report.get("research_path_xy_m")
    if not isinstance(cells, list) or not 2 <= len(cells) <= max_points:
        raise ValueError("reference has too few or too many path samples")
    resolution = report.get("resolution_m")
    if not isinstance(resolution, (int,float)) or not .05 <= resolution <= 1.:
        raise ValueError("missing G4 grid resolution")
    points = []
    for sample in cells:
        if (not isinstance(sample, (tuple, list)) or len(sample) != 2 or
            not all(isinstance(n,(float,int)) and not isinstance(n,bool)
                    and math.isfinite(n) for n in sample)):
            raise ValueError("invalid path coordinates")
        points.append([float(sample[0]),float(sample[1]),float(ground_z)])
    for a,b in zip(points,points[1:]):
        length=math.dist(a[:2],b[:2])
        if not 1e-8 < length <= math.sqrt(2)*resolution+1e-3:
            raise ValueError("nonadjacent/duplicate reference cells; refusing interpolation")
    declared_len=report.get("research_path_length_m")
    calculated=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))
    if not isinstance(declared_len,(float,int)) or abs(declared_len-calculated)>max(.05,.01*calculated):
        raise ValueError("research path length and geometry disagree")
    return {
        "schema": SCHEMA,
        "status":"UNVERIFIED_RESEARCH_REFERENCE_FOR_OFFLINE_SCAN_ONLY",
        "source":"G4.3_BAG_RAY_VISIBILITY_PCD_CONFLICT_VETO",
        "source_g4_3_schema":SOURCE,
        "path_xyz_m":points,
        "frame_id":"map",
        "path_geometry_length_m":round(calculated,3),
        "waypoint_count":len(points),
        "ground_z_assumption_map_m":float(ground_z),
        "scan_path_callback_adds_body_height_m":True,
        "source_research_start_xy_m":report.get("research_start_xy_m"),
        "source_research_goal_xy_m":report.get("research_goal_xy_m"),
        "floor_support_verified":False,
        "robot_full_height_clearance_verified":False,
        "static_full_pcd_integrated_into_scan_grid":False,
        "g4_3_source_path_algorithm":report["global_reference_algorithm"],
        "reference_is_not_planned_scan_trajectory":True,
        "physical_execution_forbidden":True,
        "actuator_output":"DISABLED",
        "safety_authorization":"NEVER",
    }


def load_and_prepare(source, *, ground_z, max_points=200):
    content=json.loads(Path(source).read_text(encoding="utf-8"))
    return prepare_reference(content,ground_z=ground_z,max_points=max_points)


def save_reference(path, reference):
    target=Path(path)
    target.parent.mkdir(parents=True,exist_ok=True)
    with target.open("x",encoding="utf-8") as f:
        json.dump(reference,f,ensure_ascii=False,indent=2,allow_nan=False)
        f.write("\n")
    return target
