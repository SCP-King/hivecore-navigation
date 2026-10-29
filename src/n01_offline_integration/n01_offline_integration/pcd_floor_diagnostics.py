"""Research-only diagnostics of PCD floor height hypotheses.

No floor inference or navigation safety claim. Results require map inspection.
"""
from collections import deque
import math

from .pcd_goal_grid import FREE, project


def free_component_stats(grid):
    """Count four-connected candidate free regions, not robot traversability."""
    cells = grid["grid"]
    w, h = grid["width"], grid["height"]
    if len(cells) != w * h:
        raise ValueError("invalid occupancy grid dimensions")
    visited = bytearray(len(cells))
    regions = []
    for start, value in enumerate(cells):
        if value != FREE or visited[start]:
            continue
        pending = deque([start])
        visited[start] = 1
        n = 0
        min_x, max_x = w, 0
        min_y, max_y = h, 0
        while pending:
            i = pending.popleft()
            n += 1
            x, y = i % w, i // w
            min_x, max_x = min(min_x, x), max(max_x, x)
            min_y, max_y = min(min_y, y), max(max_y, y)
            for j in (i - 1, i + 1, i - w, i + w):
                if (j == i - 1 and x == 0 or
                    j == i + 1 and x == w - 1 or
                    j < 0 or j >= len(cells) or
                    cells[j] != FREE or visited[j]):
                    continue
                visited[j] = 1
                pending.append(j)
        regions.append((n, (max_x-min_x+1)*grid["resolution"],
                        (max_y-min_y+1)*grid["resolution"]))
    regions.sort(reverse=True)
    largest = regions[0] if regions else (0, 0., 0.)
    return {
        "free_component_count_4_connected": len(regions),
        "largest_free_component_cells": largest[0],
        "largest_free_component_bbox_m": [round(largest[1],3), round(largest[2],3)],
        "top_free_component_sizes": [n for n,_,_ in regions[:8]],
        "candidate_free_area_m2": round(grid["free_cells"]*grid["resolution"]**2,3),
        "largest_component_area_m2": round(largest[0]*grid["resolution"]**2,3),
    }


def floor_height_sweep(points, *, z_step=.25, resolution=.25,
                       floor_tolerance=.12, obstacle_min_z=.2,
                       obstacle_max_z=1.8, clearance=.5):
    """Compare hypotheses; highest free count may be CEILING, not ground."""
    if not (.1 <= z_step <= .5) or not math.isfinite(z_step):
        raise ValueError("invalid floor sweep step")
    if not points or any(len(p) < 3 or not all(math.isfinite(x) for x in p[:3])
                         for p in points):
        raise ValueError("empty or invalid PCD points")
    minimum = min(p[2] for p in points)
    maximum = max(p[2] for p in points)
    low = math.floor(minimum / z_step) * z_step
    high = math.ceil(maximum / z_step) * z_step
    count = int(round((high - low) / z_step)) + 1
    if count > 80:
        raise ValueError("too many height hypotheses")
    rows = []
    for k in range(count):
        z = round(low + k*z_step,5)
        m = project(points, resolution=resolution, floor_z=z,
                    floor_tolerance=floor_tolerance,
                    obstacle_min_z=obstacle_min_z,
                    obstacle_max_z=obstacle_max_z,clearance=clearance)
        comp = free_component_stats(m)
        n = m["width"] * m["height"]
        rows.append({
            "floor_z_hypothesis_m": z,
            "floor_hits": m["floor_hits"],
            "obstacle_hits": m["obstacle_hits"],
            "free_cells": m["free_cells"],
            "inflated_occupied_cells": m["inflated_occupied_cells"],
            "unknown_cells": m["unknown_cells"],
            "free_percent": round(100*m["free_cells"]/n,3),
            "unknown_percent": round(100*m["unknown_cells"]/n,3),
            "largest_free_component_cells": comp["largest_free_component_cells"],
            "free_component_count": comp["free_component_count_4_connected"],
            "largest_free_bbox_m": comp["largest_free_component_bbox_m"],
        })
    return {
        "schema": "n01.offline.pcd_floor_sweep.v1",
        "status": "HEIGHT_HYPOTHESES_ONLY_NOT_FLOOR_VALIDATED",
        "notes": [
            "Candidate free means a point exists close to the assumed floor plane.",
            "Largest free component may be a roof or machine top; DO NOT select automatically.",
            "Unknown remains unknown. No ROS, SCAN, or robot control is used."
        ],
        "source_point_count":len(points),
        "resolution_m":resolution,
        "clearance_assumption_m":clearance,
        "rows":rows,
        "safety_authorization":"NEVER",
        "actuator_output":"DISABLED",
    }
