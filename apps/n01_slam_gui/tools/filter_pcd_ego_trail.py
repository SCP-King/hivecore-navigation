#!/usr/bin/env python3
"""Remove historical ego-point trails from a PCD using the mapping trajectory.

Only points inside the robot footprint at a recorded pose (plus margin) are
removed. Without a trajectory the tool refuses to run: there is no reliable way
to distinguish historical self returns from real obstacles.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def read_pcd_xyz(path):
    raw = Path(path).read_bytes()
    marker = raw.find(b"DATA")
    if marker < 0:
        raise ValueError("PCD missing DATA line")
    head = raw[:marker].decode("latin-1")
    lines = head.splitlines()
    hdr = {}
    for line in lines:
        fields = line.split()
        if len(fields) >= 2:
            hdr[fields[0]] = fields[1:]
    if "FIELDS" not in hdr or hdr["FIELDS"][:3] != ["x", "y", "z"]:
        raise ValueError("PCD must have x y z fields")
    count = int(hdr.get("POINTS", hdr.get("WIDTH", ["0"]))[0])
    data_line = raw[marker:].split(b"\n", 1)
    if len(data_line) < 2:
        raise ValueError("PCD data missing")
    data_type = hdr.get("TYPE", ["F", "F", "F"])[:3]
    size = hdr.get("SIZE", ["4", "4", "4"])[:3]
    if data_type != ["F", "F", "F"] or size != ["4", "4", "4"]:
        raise ValueError("only float32 xyz PCD is supported")
    payload = data_line[1]
    data = np.frombuffer(payload, dtype="<f4")
    if data.size < count * 3:
        raise ValueError("PCD payload shorter than POINTS")
    return data[: count * 3].reshape(-1, 3).astype(np.float64)


def write_pcd_xyz(path, points):
    points = np.asarray(points, dtype="<f4")
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    ).encode("ascii")
    Path(path).write_bytes(header + points.tobytes())


def read_trajectory(path):
    poses = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = [x for x in line.replace(",", " ").split() if x]
        try:
            values = [float(x) for x in fields]
        except ValueError:
            continue  # header
        if len(values) >= 8:
            t, x, y, z, qx, qy, qz, qw = values[:8]
            yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))
        elif len(values) >= 4:
            t, x, y, z = values[:4]
            yaw = 0.0
        elif len(values) >= 3:
            x, y, yaw = values[:3]
            z = 0.0
        else:
            continue
        poses.append((x, y, z, yaw))
    return poses


def footprint_from_args(args):
    if args.footprint_json:
        data = json.loads(Path(args.footprint_json).read_text(encoding="utf-8"))
        if isinstance(data, dict):
            data = data.get("footprint") or data.get("vertices_m")
        polygon = np.asarray(data, dtype=float)
        if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3:
            raise ValueError("footprint JSON must be [[x,y], ...]")
        return polygon
    half_l = args.length / 2.0
    half_w = args.width / 2.0
    return np.asarray([[-half_l, -half_w], [half_l, -half_w],
                       [half_l, half_w], [-half_l, half_w]], dtype=float)


def _point_in_polygon(points, polygon):
    x = points[:, 0]
    y = points[:, 1]
    inside = np.zeros(len(points), dtype=bool)
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        crossing = ((yi > y) != (yj > y)) & (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
        inside ^= crossing
        j = i
    return inside


def _distance_to_polygon(points, polygon):
    best = np.full(len(points), np.inf)
    for i in range(len(polygon)):
        a = polygon[i]
        b = polygon[(i + 1) % len(polygon)]
        ab = b - a
        denom = float(ab @ ab)
        t = np.clip(((points - a) @ ab) / max(denom, 1e-12), 0.0, 1.0)
        closest = a + t[:, None] * ab
        best = np.minimum(best, np.linalg.norm(points - closest, axis=1))
    return best


def build_removed_cells(poses, polygon, margin, resolution, z_down, z_up):
    cells = {}
    for x, y, z, yaw in poses:
        c, s = math.cos(yaw), math.sin(yaw)
        world = polygon @ np.array([[c, s], [-s, c]]) + np.array([x, y])
        lo = world.min(axis=0) - margin - resolution
        hi = world.max(axis=0) + margin + resolution
        xs = np.arange(lo[0], hi[0] + resolution, resolution)
        ys = np.arange(lo[1], hi[1] + resolution, resolution)
        gx, gy = np.meshgrid(xs, ys)
        pts = np.column_stack((gx.ravel(), gy.ravel()))
        keep = (_point_in_polygon(pts, world) |
                (_distance_to_polygon(pts, world) <= margin))
        for px, py in pts[keep]:
            key = (int(math.floor(px / resolution)),
                   int(math.floor(py / resolution)))
            z0, z1 = z - z_down, z + z_up
            old = cells.get(key)
            if old is None:
                cells[key] = [z0, z1]
            else:
                old[0] = min(old[0], z0)
                old[1] = max(old[1], z1)
    return cells


def filter_points(points, poses, polygon, margin=.06, resolution=.10,
                  z_down=.80, z_up=.90):
    if not poses:
        raise ValueError("trajectory_required_for_historical_ego_filter")
    cells = build_removed_cells(poses, polygon, margin, resolution, z_down, z_up)
    ix = np.floor(points[:, 0] / resolution).astype(np.int64)
    iy = np.floor(points[:, 1] / resolution).astype(np.int64)
    keep = np.ones(len(points), dtype=bool)
    removed = 0
    for i, (x, y, z) in enumerate(points):
        cell = cells.get((int(ix[i]), int(iy[i])))
        if cell is not None and cell[0] <= z <= cell[1]:
            keep[i] = False
            removed += 1
    return points[keep], removed, len(cells)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--trajectory", required=True,
                        help="CSV: timestamp x y z qx qy qz qw (header allowed)")
    parser.add_argument("--output", required=True)
    parser.add_argument("--footprint-json", default="",
                        help="body-frame [[x,y], ...]; default rectangle")
    parser.add_argument("--length", type=float, default=.58)
    parser.add_argument("--width", type=float, default=.34)
    parser.add_argument("--margin", type=float, default=.06)
    parser.add_argument("--resolution", type=float, default=.10)
    parser.add_argument("--z-down", type=float, default=.80)
    parser.add_argument("--z-up", type=float, default=.90)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if Path(args.output).resolve() == Path(args.pcd).resolve():
        raise SystemExit("refusing to overwrite input PCD")
    points = read_pcd_xyz(args.pcd)
    poses = read_trajectory(args.trajectory)
    if not poses:
        raise SystemExit("trajectory has no poses; refusing to filter")
    polygon = footprint_from_args(args)
    kept, removed, cells = filter_points(
        points, poses, polygon, args.margin, args.resolution,
        args.z_down, args.z_up)
    print(f"input={len(points)} kept={len(kept)} removed={removed} "
          f"poses={len(poses)} cells={cells}")
    if not args.dry_run:
        write_pcd_xyz(args.output, kept)
        print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
