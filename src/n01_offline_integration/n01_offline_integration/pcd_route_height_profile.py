"""G4.9: profile real PCD XYZ surface evidence beside a Bag-anchored XY route.

A histogram of measured heights is NOT a floor classifier. Zero points are
UNKNOWN, not clear space. This module never publishes ROS or robot commands.
"""
from collections import Counter, defaultdict
import math

PROFILE_SCHEMA = "n01.offline.g49_pcd_route_surface_height_evidence.v1"


def _quantiles(values):
    if not values:
        return None
    data = sorted(values)
    return {
        key: round(data[round((len(data) - 1) * p)], 4)
        for key, p in (
            ("min", 0.), ("p10", .1), ("p25", .25),
            ("median", .5), ("p75", .75), ("p90", .9), ("max", 1.))
    }


def profile_route_surfaces(points, path_xy, *, radius_m=.35, height_bin_m=.1):
    if (not .1 <= radius_m <= 1.5 or not .05 <= height_bin_m <= .5 or
        not all(math.isfinite(v) for v in (radius_m, height_bin_m))):
        raise ValueError("invalid surface evidence sampling bounds")
    if not isinstance(path_xy, list) or not 2 <= len(path_xy) <= 200:
        raise ValueError("bounded research XY route required")
    for p in path_xy:
        if (not isinstance(p, (list, tuple)) or len(p) != 2 or
            any(type(v) not in (float, int) or not math.isfinite(v) for v in p)):
            raise ValueError("invalid research XY waypoint")
    if not points:
        raise ValueError("PCD surface points unavailable")
    scale = radius_m
    buckets = defaultdict(list)
    finite = 0
    for point in points:
        if (len(point) != 3 or
            any(type(v) not in (float, int) or not math.isfinite(v) for v in point)):
            continue
        x, y, z = point
        buckets[math.floor(x / scale), math.floor(y / scale)].append((x, y, z))
        finite += 1
    if finite == 0:
        raise ValueError("PCD contains no finite XYZ evidence")
    rows = []
    all_hits = 0
    for i, (x, y) in enumerate(path_xy):
        cx, cy = math.floor(x / scale), math.floor(y / scale)
        zvals = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for px, py, pz in buckets.get((cx + dx, cy + dy), ()):
                    if (px - x)**2 + (py - y)**2 <= radius_m**2:
                        zvals.append(pz)
        all_hits += len(zvals)
        bins = Counter(math.floor(z / height_bin_m) for z in zvals)
        # Dominant vertical bands are descriptive ONLY. We do NOT choose the
        # lowest bin as ground: it can be clutter, walls, noise or a drop.
        ranked = sorted(bins.items(), key=lambda item: (-item[1], item[0]))[:4]
        rows.append({
            "index": i,
            "research_xy_m": [float(x), float(y)],
            "pcd_samples_within_xy_radius": len(zvals),
            "surface_z_quantiles_m": _quantiles(zvals),
            "dominant_height_bins": [
                {
                    "z_interval_m": [round(k * height_bin_m, 4),
                                     round((k + 1) * height_bin_m, 4)],
                    "pcd_samples": count
                }
                for k, count in ranked
            ],
            "map_ground_z_m": None,
            "ground_classification": "NOT_DETERMINED_FROM_PCD_Z_HISTOGRAM",
        })
    return {
        "schema": PROFILE_SCHEMA,
        "status": "MEASURED_SURFACE_EVIDENCE_NOT_GROUND_OR_TRAVERSABILITY",
        "route_waypoints": len(rows),
        "sample_radius_xy_m": radius_m,
        "height_bin_m": height_bin_m,
        "source_finite_pcd_points": finite,
        "waypoints_without_nearby_pcd_samples": sum(
            row["pcd_samples_within_xy_radius"] == 0 for row in rows),
        "total_local_samples_counted_with_overlap": all_hits,
        "rows": rows,
        "ground_z_selected_m": None,
        "flat_floor_verified": False,
        "support_continuity_verified": False,
        "robot_clearance_verified": False,
        "unknown_space_never_assumed_free": True,
        "route_to_scan_executed": False,
        "actuator_output": "DISABLED",
        "safety_authorization": "NEVER",
    }
