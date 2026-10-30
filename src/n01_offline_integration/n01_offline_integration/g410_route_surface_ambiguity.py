"""G4.10: research-only ambiguity/missing-data audit for G4.9 PCD route Z bands.

Multiple surface heights or low sample support are *not* floor identification.
Even one concentrated height band is NOT verified traversable ground. No ROS,
ground-Z selection, robot control or SCAN path publication.
"""
import math

SCHEMA = "n01.offline.g410_route_surface_ambiguity_audit.v1"
INPUT_SCHEMA = "n01.offline.g49_pcd_route_surface_height_evidence.v1"


def audit_surface_ambiguity(profile, *, wide_iqr_threshold_m=.5):
    """Summarize uncertainty along the *existing* Bag-anchored XY research route."""
    if (type(wide_iqr_threshold_m) is not float or
            not math.isfinite(wide_iqr_threshold_m) or
            not .1 <= wide_iqr_threshold_m <= 2.0):
        raise ValueError("invalid diagnostic Z spread threshold")
    if not isinstance(profile, dict) or profile.get("schema") != INPUT_SCHEMA:
        raise ValueError("requires G4.9 PCD route surface evidence")
    rows = profile.get("rows")
    if (not isinstance(rows, list) or not 2 <= len(rows) <= 200 or
            profile.get("route_waypoints") != len(rows) or
            profile.get("ground_z_selected_m") is not None or
            profile.get("support_continuity_verified") is not False):
        raise ValueError("G4.9 route must have unknown floor and unverified support")
    results = []
    missing = multi = other = 0
    for i, row in enumerate(rows):
        if not isinstance(row, dict) or row.get("index") != i:
            raise ValueError("route index is not sequential")
        xy=row.get("research_xy_m")
        if (not isinstance(xy,list) or len(xy)!=2 or
                any(type(v) not in (int,float) or not math.isfinite(v)
                    for v in xy)):
            raise ValueError("invalid research route XY")
        n=row.get("pcd_samples_within_xy_radius")
        if type(n) is not int or n < 0 or row.get("map_ground_z_m") is not None:
            raise ValueError("invalid surface count or guessed ground Z")
        q=row.get("surface_z_quantiles_m")
        if n==0:
            if q is not None or row.get("dominant_height_bins") not in ([],None):
                raise ValueError("absent samples cannot contain surface bands")
            label="UNKNOWN_NO_LOCAL_PCD_SAMPLES"
            spread=None
            missing+=1
        else:
            if not isinstance(q,dict):
                raise ValueError("nonempty PCD requires finite ordered quantiles")
            names=("min","p10","p25","median","p75","p90","max")
            values=[q.get(k) for k in names]
            if (any(type(v) not in (int,float) or not math.isfinite(v)
                    for v in values) or any(a>b for a,b in zip(values,values[1:]))):
                raise ValueError("invalid ordered surface Z quantiles")
            spread=round(q["p75"]-q["p25"],4)
            if spread>=wide_iqr_threshold_m:
                label="MULTI_LEVEL_OR_HIGH_VARIATION_SURFACE_EVIDENCE"
                multi+=1
            else:
                label="LOCAL_SURFACE_BAND_NOT_VALIDATED_AS_GROUND"
                other+=1
        results.append({
            "index":i,"research_xy_m":xy,"local_pcd_samples":n,
            "interquartile_surface_z_spread_m":spread,
            "surface_evidence_classification":label,
            "ground_z_map_m":None,
            "support_verified":False,
            "robot_clearance_verified":False,
            "wheel_contact_verified":False,
            "route_waypoint_traversable":None,
        })
    return {
        "schema":SCHEMA,
        "result":"RESEARCH_SURFACE_AMBIGUITY_MAPPED_GROUND_UNVERIFIED",
        "input_schema":INPUT_SCHEMA,
        "surface_variation_threshold_m_for_diagnostic_only":wide_iqr_threshold_m,
        "route_waypoints":len(results),
        "waypoints_no_nearby_pcd":missing,
        "waypoints_multi_height_or_high_z_variation":multi,
        "waypoints_other_surface_evidence_not_ground_verified":other,
        "classifications_account_for_all_waypoints":missing+multi+other==len(results),
        "rows":results,
        "ground_z_map_m":None,
        "ground_elevation_surface_function_verified":False,
        "local_ground_normals_verified":False,
        "wheel_ground_contact_verified":False,
        "full_body_collision_clearance_verified":False,
        "route_to_scan_allowed":False,
        "why_not_traversable":"A lidar Z histogram cannot identify supporting floor, longitudinal slope, steps, wheel contact or whole-body clearance.",
        "safety_authorization":"NEVER",
        "actuator_output":"DISABLED",
    }
