"""Pure-Python offline collision checks matching SCAN's inflated-grid lookup.

The planner's two cylindrical body centres are offset +/-0.18m along heading.
The map is already inflated by the body radius. This only tests known
published occupied voxels and does not certify unknown-space safety.
"""
import math

RESOLUTION = 0.05
COLLISION_RADIUS_M = 0.09

def cloud_voxel_index(points):
    # SCAN GridMap::posToIndex uses floor(pos/resolution), never round().
    # Inflated cloud publishes the *centre* of occupied grid cells.
    return {(math.floor(x / RESOLUTION), math.floor(y / RESOLUTION),
             math.floor(z / RESOLUTION)) for x, y, z in points}


def path_margin_proximity(path, voxels):
    """Supplementary conservative 9-cm proximity; diagnostic, NOT SCAN's collision query."""

    near_offsets = [(i, j, k) for i in range(-2, 3)
                    for j in range(-2, 3) for k in range(-2, 3)]
    collisions = []
    radius2 = COLLISION_RADIUS_M * COLLISION_RADIUS_M
    for t, (x, y, z) in enumerate(path):
        c = (math.floor(x / RESOLUTION), math.floor(y / RESOLUTION), math.floor(z / RESOLUTION))
        hit = False
        for dx, dy, dz in near_offsets:
            if (c[0] + dx, c[1] + dy, c[2] + dz) not in voxels:
                continue
            cx, cy, cz = ((c[0] + dx + 0.5) * RESOLUTION,
                          (c[1] + dy + 0.5) * RESOLUTION,
                          (c[2] + dz + 0.5) * RESOLUTION)
            if (x-cx)**2 + (y-cy)**2 + (z-cz)**2 <= radius2:
                hit = True
                break
        if hit:
            collisions.append(t)
    return collisions


def path_collision(path, voxels):
    """Reproduce SCAN GridMap::getInflateOccupancy's double-cylinder query.

    The planner checks the two cylinder centres +/- 0.18m along the local
    heading against already-inflated voxels (radius 0.25m is in the map).
    Return indices and examples; unobserved/unknown space remains unverified.
    """
    hits = []
    for idx, (x, y, z) in enumerate(path):
        if idx + 1 < len(path):
            target = path[idx + 1]
        elif idx > 0:
            target = path[idx - 1]
        else:
            target = (x + 1.0, y, z)
        dx = target[0] - x
        dy = target[1] - y
        if idx + 1 == len(path):
            dx, dy = x - target[0], y - target[1]
        norm = math.hypot(dx, dy)
        ux, uy = (dx / norm, dy / norm) if norm > 1e-9 else (1.0, 0.0)
        hit_centres = []
        for direction in (-1.0, 1.0):
            cx = x + direction * 0.18 * ux
            cy = y + direction * 0.18 * uy
            cell = (math.floor(cx / RESOLUTION), math.floor(cy / RESOLUTION),
                    math.floor(z / RESOLUTION))
            if cell in voxels:
                hit_centres.append({"side": "front" if direction > 0 else "rear",
                                    "voxel": list(cell)})
        if hit_centres:
            hits.append({"sample_idx": idx,
                         "point": [round(x, 4), round(y, 4), round(z, 4)],
                         "centres": hit_centres})
    return hits

