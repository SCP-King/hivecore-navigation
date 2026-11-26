#!/usr/bin/env python3
"""Export a PCD to a map_server grid + red-boundary PNG.

Requires the ground plane recorded for that PCD in
config/n01_map_ground_profiles.json (or --ground-z/--ground-a/--ground-b).
The PCD remains the source of occupied cells. Free-space reachability is a
conservative 4-connected flood fill: only the free component containing the
map origin (or the largest free component) stays free; other enclosed free
components are written as occupied. The convex hull of the occupied points is
the red known-map boundary. This is an offline artifact, not a planner layer.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import ConvexHull
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'n01_static_map'))
from n01_static_map.pcd_xyz import parse_pcd_xyz


def load_ground(pcd: Path, args):
    if args.ground_z is not None:
        return float(args.ground_a), float(args.ground_b), float(args.ground_z)
    profiles = json.loads((ROOT / 'config' / 'n01_map_ground_profiles.json').read_text())
    entry = profiles.get('maps', {}).get(str(pcd))
    if not entry:
        raise SystemExit(f'no ground profile for {pcd}; pass --ground-z')
    return float(entry['a']), float(entry['b']), float(entry['c'])


def rasterize(x, y, res, pad):
    x0 = math.floor((x.min() - pad) / res) * res
    y0 = math.floor((y.min() - pad) / res) * res
    x1 = math.ceil((x.max() + pad) / res) * res
    y1 = math.ceil((y.max() + pad) / res) * res
    w = max(1, int(round((x1 - x0) / res)))
    h = max(1, int(round((y1 - y0) / res)))
    ix = np.floor((x - x0) / res).astype(np.int64)
    iy = np.floor((y - y0) / res).astype(np.int64)
    ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    occ = np.zeros((h, w), bool)
    occ[iy[ok], ix[ok]] = True
    return occ, x0, y0, w, h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--out-dir', default='/home/demo/lyy/slam_data/occupancy_grid_maps')
    ap.add_argument('--resolution', type=float, default=.05)
    ap.add_argument('--pad', type=float, default=.50)
    ap.add_argument('--ground-z', type=float)
    ap.add_argument('--ground-a', type=float, default=0.)
    ap.add_argument('--ground-b', type=float, default=0.)
    args = ap.parse_args()
    pcd = Path(args.pcd).resolve()
    if not pcd.is_file():
        raise SystemExit(f'PCD not found: {pcd}')
    if not (.02 <= args.resolution <= .20):
        raise SystemExit('resolution must be 0.02..0.20 m')
    a, b, c = load_ground(pcd, args)
    xs, ys, zs = parse_pcd_xyz(str(pcd))
    ground = a * xs + b * ys + c
    keep = (zs > ground + .06) & (zs <= ground + 1.60)
    x, y, z = xs[keep], ys[keep], zs[keep]
    if len(x) < 20:
        raise SystemExit(f'too few obstacle points after z band: {len(x)}')
    occ, x0, y0, w, h = rasterize(x, y, args.resolution, args.pad)

    # Known boundary: convex hull of observed occupied points, rasterized as
    # filled cells only for the reachability flood fill. The red line in PNG is
    # the hull itself, not a fake occupancy cell.
    pts = np.column_stack((x, y))
    try:
        hull = pts[ConvexHull(pts).vertices]
    except Exception:
        raise SystemExit('cannot build known-map boundary')
    mask = Image.new('L', (w, h), 0)
    d = ImageDraw.Draw(mask)
    poly = [((px - x0) / args.resolution, (py - y0) / args.resolution)
            for px, py in hull]
    d.polygon(poly, fill=255)
    inside = np.asarray(mask, bool)
    # The known-map boundary is an obstacle, not just a red annotation.
    boundary_mask = Image.new('L', (w, h), 0)
    bd = ImageDraw.Draw(boundary_mask)
    bd.line(poly + [poly[0]], fill=255, width=1)
    boundary = np.asarray(boundary_mask, bool)

    free = inside & ~occ & ~boundary
    # 4-connected components of free space.
    labels = np.zeros((h, w), np.int32)
    current = 0
    for yy in range(h):
        for xx in range(w):
            if not free[yy, xx] or labels[yy, xx]:
                continue
            current += 1
            stack = [(yy, xx)]
            labels[yy, xx] = current
            while stack:
                cy, cx = stack.pop()
                for ny, nx in ((cy-1,cx),(cy+1,cx),(cy,cx-1),(cy,cx+1)):
                    if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not labels[ny, nx]:
                        labels[ny, nx] = current
                        stack.append((ny, nx))
    if current == 0:
        raise SystemExit('no free component inside known boundary')
    origin_cell = (int(round((0. - y0) / args.resolution)),
                   int(round((0. - x0) / args.resolution)))
    seed = labels[origin_cell] if 0 <= origin_cell[0] < h and 0 <= origin_cell[1] < w else 0
    if seed == 0:
        counts = np.bincount(labels.ravel())
        counts[0] = 0
        seed = int(counts.argmax())
    reachable = labels == seed
    enclosed = free & ~reachable
    # Per requirement: enclosed unreachable free space becomes occupied.
    occ_out = occ | enclosed | boundary

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = pcd.stem
    pgm = out_dir / f'{stem}.pgm'
    yaml = out_dir / f'{stem}.yaml'
    png = out_dir / f'{stem}.png'
    meta = out_dir / f'{stem}.json'
    # map_server PGM: 0 occupied, 254 free, 205 unknown.
    img = np.full((h, w), 205, np.uint8)
    img[reachable] = 254
    img[occ_out] = 0
    # map_server expects row 0 at top; our y0 is the lower map edge.
    Image.fromarray(np.flipud(img), mode='L').save(pgm)
    yaml.write_text(
        f'image: {pgm.name}\nresolution: {args.resolution:.6f}\n'
        f'origin: [{x0:.6f}, {y0:.6f}, 0.0]\nnegate: 0\n'
        f'occupied_thresh: 0.65\nfree_thresh: 0.196\n', encoding='utf-8')
    rgb = np.empty((h, w, 3), np.uint8)
    # Outside-boundary unknown is gray, matching the GUI legend. Occupied cells
    # and the convex boundary remain black; the boundary is drawn red below.
    rgb[:] = (128, 128, 128)
    rgb[reachable] = (250, 250, 245)
    rgb[occ_out] = (0, 0, 0)
    pim = Image.fromarray(np.flipud(rgb), mode='RGB')
    pd = ImageDraw.Draw(pim)
    hull_px = [((px - x0) / args.resolution,
                h - 1 - (py - y0) / args.resolution) for px, py in hull]
    pd.line(hull_px + [hull_px[0]], fill=(255, 0, 0), width=2)
    pim.save(png)
    meta.write_text(json.dumps({
        'schema': 'n01.occupancy_grid.v1',
        'pcd': str(pcd), 'pgm': str(pgm), 'png': str(png),
        'resolution_m': args.resolution, 'origin': [x0, y0, 0.0],
        'size_px': [w, h], 'ground_plane': [a, b, c],
        'z_band': [.06, 1.60],
        'occupied_cells': int(occ_out.sum()),
        'enclosed_unreachable_cells': int(enclosed.sum()),
        'free_cells': int(reachable.sum()),
        'boundary_method': 'convex_hull_of_occupied_points',
        'boundary_vertices_m': [[float(px), float(py)] for px, py in hull],
        'boundary_is_obstacle': True,
        'robot_drawn': False,
        'unknown_policy': 'enclosed_unknown_to_obstacle_unenclosed_to_free',
        'reachability_seed': 'map_origin_or_largest_free_component',
    }, indent=2) + '\n', encoding='utf-8')
    print(f'wrote {pgm} {yaml} {png} {meta}')
    print(f'occupied={int(occ_out.sum())} enclosed={int(enclosed.sum())} free={int(reachable.sum())}')


if __name__ == '__main__':
    main()
