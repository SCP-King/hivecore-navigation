"""G4.8: select a *research-only* goal reachable from the first accepted Bag pose.

No arbitrary synthetic A/B points. FREE is observed lidar-height ray-visibility
after PCD slice conflict veto, NOT certified ground or a robot-safe corridor.
No ROS nodes, actuators, map alignment inference or guessed floor Z.
"""
from collections import deque
import math
from .pcd_bag_evidence_audit import FREE, astar, map_cell


def choose_research_goal(m, first_pose_xy, *, min_length_m=0.5, max_waypoints=200):
    if (not isinstance(first_pose_xy, (tuple, list)) or len(first_pose_xy) != 2 or
        not all(type(v) in (float, int) and math.isfinite(v) for v in first_pose_xy)):
        raise ValueError("first Bag pose XY must contain finite measured coordinates")
    if (not .25 <= min_length_m <= 10.0 or
        not 2 <= max_waypoints <= 200):
        raise ValueError("invalid research route bounds")
    try:
        start = map_cell(m, first_pose_xy)
    except ValueError as exc:
        raise ValueError("BAG_START_OUTSIDE_MAP: " + str(exc)) from exc
    grid, width, height = m["grid"], m["width"], m["height"]
    if grid[start] != FREE:
        raise ValueError("BAG_START_NOT_OBSERVED_FREE: cannot move or snap real start into a different cell")
    # A 4-connected BFS ensures that the chosen research endpoint belongs
    # to the same actually observed component as the BAG start.
    q = deque([start])
    depth = {start: 0}
    while q:
        here = q.popleft()
        x, y = here % width, here // width
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            xx, yy = x + dx, y + dy
            if 0 <= xx < width and 0 <= yy < height:
                neighbor = yy * width + xx
                if grid[neighbor] == FREE and neighbor not in depth:
                    depth[neighbor] = depth[here] + 1
                    q.append(neighbor)
    # Do not manufacture a route outside this component, or to an arbitrary
    # goal in a different connected component.
    for goal in sorted(depth, key=lambda index: (-depth[index], index)):
        if goal == start:
            continue
        path, length = astar(m, start, goal)
        if len(path) <= max_waypoints and length >= min_length_m:
            return {
                "schema": "n01.offline.bag_anchored_goal_selection.v1",
                "selection": "FIRST_ACCEPTED_BAG_SCAN_POSE_TO_FARTHEST_CONNECTED_RAY_VISIBLE_CELL",
                "first_bag_pose_xy_m": [float(v) for v in first_pose_xy],
                "start_cell": start,
                "goal_cell": goal,
                "goal_xy_m": [
                    round(m["origin"][0] + (goal % width + .5) * m["resolution"], 5),
                    round(m["origin"][1] + (goal // width + .5) * m["resolution"], 5),
                ],
                "reachable_4_connected_free_cells": len(depth),
                "research_path_length_m": length,
                "research_path_cells": len(path),
                "start_cell_grid_center_offset_m": round(math.hypot(
                    m["origin"][0]+((start % width)+.5)*m["resolution"]-first_pose_xy[0],
                    m["origin"][1]+((start // width)+.5)*m["resolution"]-first_pose_xy[1]), 5),
                "floor_z_map_m": None,
                "floor_verified": False,
                "robot_clearance_verified": False,
                "true_traversability_verified": False,
                "safety_authorization": "NEVER",
                "actuator_output": "DISABLED",
            }
    raise ValueError("NO_USABLE_BAG_ANCHORED_RESEARCH_GOAL: insufficient observed connected free area")
