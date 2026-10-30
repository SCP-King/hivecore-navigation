"""G4.4 evidence-only PCD geometry checks along a G4.5 research route.

This checks a manually selected constant floor-z and *candidate* obstacle
height band against XYZ surface samples. It cannot prove ground support,
3D volume clearance, localization, locomotion or Hive2 safety.
"""
from collections import defaultdict
import math


def audit_pcd_path(points, reference, *, floor_tolerance=.15,
                   floor_support_radius=.35,
                   obstacle_height_min=.2, obstacle_height_max=1.8,
                   lateral_clearance_radius=.45):
    if (reference.get("schema")!="n01.offline.scan_reference.v1" or
        reference.get("safety_authorization")!="NEVER"):
        raise ValueError("requires bounded G4.5 unverified research reference")
    if not (.02<=floor_tolerance<=.4 and
            .1<=floor_support_radius<=1.5 and
            0.05<=obstacle_height_min<obstacle_height_max<=3.5 and
            0.1<=lateral_clearance_radius<=1.5 and
            all(math.isfinite(v) for v in (floor_tolerance,
                floor_support_radius,obstacle_height_min,
                obstacle_height_max,lateral_clearance_radius))):
        raise ValueError("invalid bounded geometry audit parameters")
    path=reference.get("path_xyz_m")
    if not isinstance(path,list) or not 2<=len(path)<=200:
        raise ValueError("invalid G4.5 path")
    if not points:
        raise ValueError("missing actual PCD surface samples")
    resolution=.25
    buckets=defaultdict(list)
    for p in points:
        if len(p)!=3 or not all(math.isfinite(v) for v in p):
            continue
        x,y,z=p
        key=(math.floor(x/resolution),math.floor(y/resolution))
        buckets[key].append(p)
    if not buckets:
        raise ValueError("no usable PCD samples")
    rows=[]
    floor_z=reference["ground_z_assumption_map_m"]
    span=max(floor_support_radius,lateral_clearance_radius)
    reach=math.ceil(span/resolution)
    for idx,p in enumerate(path):
        if len(p)!=3 or not all(math.isfinite(v) for v in p):
            raise ValueError("invalid route point")
        if abs(p[2]-floor_z)>1e-6:
            raise ValueError("G4.5 path has inconsistent floor_z assumption")
        cx,cy=math.floor(p[0]/resolution),math.floor(p[1]/resolution)
        supports=0; obstacles=0; nearest_obstacle_xy_m=None
        for dx in range(-reach,reach+1):
            for dy in range(-reach,reach+1):
                for q in buckets.get((cx+dx,cy+dy),()):
                    distance=math.hypot(q[0]-p[0],q[1]-p[1])
                    dz=q[2]-floor_z
                    if distance<=floor_support_radius and abs(dz)<=floor_tolerance:
                        supports+=1
                    if (distance<=lateral_clearance_radius and
                            obstacle_height_min<=dz<=obstacle_height_max):
                        obstacles+=1
                        nearest_obstacle_xy_m=(distance if
                            nearest_obstacle_xy_m is None else
                            min(nearest_obstacle_xy_m,distance))
        rows.append({
            "index":idx,"xy_m":p[:2],
            "floor_candidate_hits":supports,
            "pcd_points_in_candidate_body_height_band":obstacles,
            "closest_pcd_obstacle_xy_m":(
                round(nearest_obstacle_xy_m,4)
                if nearest_obstacle_xy_m is not None else None),
            "classification":(
                "PCD_SURFACE_CONFLICT" if obstacles else
                "FLOOR_CANDIDATE_WITH_NO_SAMPLED_OBSTACLE" if supports else
                "UNKNOWN_NO_FLOOR_CANDIDATE")
        })
    n_blocked=sum(r["classification"]=="PCD_SURFACE_CONFLICT" for r in rows)
    n_unknown=sum(r["classification"]=="UNKNOWN_NO_FLOOR_CANDIDATE" for r in rows)
    n_support=len(rows)-n_blocked-n_unknown
    return {
        "schema":"n01.offline.g4_pcd_route_geometry_audit.v1",
        "status":"RESEARCH_EVIDENCE_NOT_HIVE2_TRAVERSABILITY_CERTIFICATION",
        "ground_z_assumption_map_m":floor_z,
        "floor_tolerance_m":floor_tolerance,
        "floor_support_search_radius_m":floor_support_radius,
        "obstacle_height_band_above_ground_m":[obstacle_height_min,obstacle_height_max],
        "lateral_clearance_assumption_m":lateral_clearance_radius,
        "waypoints":len(rows),
        "pcd_surface_conflict_waypoints":n_blocked,
        "unknown_ground_support_waypoints":n_unknown,
        "candidate_supported_no_sampled_obstacle_waypoints":n_support,
        "pcd_surface_samples_indexed":sum(len(v) for v in buckets.values()),
        "rows":rows,
        "floor_physically_verified":False,
        "full_robot_clearance_verified":False,
        "unsampled_space_not_assumed_free":True,
        "scan_native_planner_connected":False,
        "safety_authorization":"NEVER","actuator_output":"DISABLED",
    }
