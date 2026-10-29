#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROI 自动选择:从全图膨胀后的 FREE 连通分量中筛选一块 3~5m 局部区域。

规则(用户指定:不以 (0,0) 为中心):
  1. PCD 全图 0.10m 栅格 + FusionGrid(robot_h/floor_max/obs_bottom 参数化);
  2. 静态 OCC 按 safety_radius=0.35m 做 L1 膨胀(名义安全距离=inflate_cells*res);
  3. 取膨胀后 FREE 的最大连通分量;
  4. 分量内选 start,再选距 start 1.5~2.8m 且与 start 同分量的 goal(A* 直连可达);
  5. 输出 ROI(以 start/goal 外接框 + 1m 边距,边 3~6m):真实 map 坐标、自由面积、
     障碍占比、可达距离、名义安全距离。

输出 stdout + JSON(供 config/测试脚本引用)。
"""
import os
import sys
import json
import numpy as np
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
sys.path.insert(0, PKG)
from n01_map_fusion.pcd_xyz import parse_pcd_xyz
from n01_map_fusion.fusion_core import FusionGrid, FREE, OCCUPIED, UNKNOWN
from n01_map_fusion.a_star_core import astar

PCD = '/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd'


def largest_component(free_mask, sx, sy):
    n = sx * sy
    vis = np.zeros(n, bool)
    best, best_cells = None, []
    for seed in np.argwhere(free_mask).ravel():
        if vis[seed]:
            continue
        q = deque([int(seed)])
        vis[int(seed)] = True
        cells = [int(seed)]
        while q:
            c = q.popleft()
            iy, ix = divmod(c, sx)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = ix + dx, iy + dy
                    if 0 <= nx < sx and 0 <= ny < sy:
                        cc = ny * sx + nx
                        if free_mask[cc] and not vis[cc]:
                            vis[cc] = True
                            q.append(cc)
                            cells.append(cc)
        if len(cells) > len(best_cells):
            best, best_cells = int(seed), cells
    return best, np.asarray(best_cells, dtype=np.int64)


def main():
    xs, ys, zs = parse_pcd_xyz(PCD)
    xyz = np.stack([xs, ys, zs], axis=1)
    fg = FusionGrid(robot_h=1.5, floor_max=0.5, obs_bottom=0.15,
                    safety_radius=0.35, min_band_points=1)
    fg.classify_static(xyz)
    fg.fuse()
    merged = fg.fused
    sx, sy = fg.sx, fg.sy

    free_infl = merged == FREE
    n_free = int(free_infl.sum())
    n_occ = int((merged == OCCUPIED).sum())
    n_unk = sx * sy - n_free - n_occ
    print(f'[全图] FREE(infl)={n_free} OCC(infl)={n_occ} UNK={n_unk} '
          f'(名义安全距离={fg.inflate_cells*fg.resolution:.2f}m)')

    seed, cells = largest_component(free_infl, sx, sy)
    print(f'[最大FREE连通分量] {len(cells)} 格 = {len(cells)*fg.resolution*fg.resolution:.0f} m^2')
    iy, ix = np.unravel_index(cells, (sy, sx))
    x0, x1 = fg.ox + ix.min()*fg.resolution, fg.ox + (ix.max()+1)*fg.resolution
    y0, y1 = fg.oy + iy.min()*fg.resolution, fg.oy + (iy.max()+1)*fg.resolution
    print(f'  bbox(map): x[{x0:.2f},{x1:.2f}] y[{y0:.2f},{y1:.2f}]')

    # start = 分量质心附近的 FREE 格
    cx, cy = float(ix.mean()), float(iy.mean())
    start_col = cells[np.argmin((ix-cx)**2 + (iy-cy)**2)]
    sx0 = fg.col_to_center(np.asarray([start_col, start_col]))
    w_sx, w_sy = sx0[0][0], sx0[1][0]

    # goal = 距 start 1.5~2.8m 且同分量的格(多选一,取最近优先)
    dist = (ix - start_col % sx)**2 + (iy - start_col // sx)**2
    cand = np.where((dist >= (1.5/fg.resolution)**2) & (dist <= (2.8/fg.resolution)**2))[0]
    goal_col = None
    if len(cand):
        k = cand[np.argmin(dist[cand])]
        goal_col = int(cells[k])
    if goal_col is None:
        print('[ROI] 分量太小,无法取 1.5~2.8m 目标'); sys.exit(2)

    path, stats = astar(fg.fused, sx, sy, int(start_col), int(goal_col))
    if path is None:
        print(f'[ROI] A* 失败:{stats["reason"]}'); sys.exit(3)
    gx0, gy0 = fg.col_to_center(np.asarray([goal_col, goal_col]))
    w_gx, w_gy = gx0[0], gy0[0]

    # ROI bbox = start/goal 外接 + 1m 边距,且每条边扩展至 >=3m(偏好 3~5m 局部区域)
    cx_box = (min(w_sx, w_gx) + max(w_sx, w_gx)) / 2.0
    cy_box = (min(w_sy, w_gy) + max(w_sy, w_gy)) / 2.0
    halfx = max((max(w_sx, w_gx) - min(w_sx, w_gx)) / 2.0 + 1.0, 1.5)
    halfy = max((max(w_sy, w_gy) - min(w_sy, w_gy)) / 2.0 + 1.0, 1.5)
    minx = cx_box - halfx
    maxx = cx_box + halfx
    miny = cy_box - halfy
    maxy = cy_box + halfy
    # 按 0.10m 对齐栅格
    minx = float(np.floor(minx / fg.resolution) * fg.resolution)
    miny = float(np.floor(miny / fg.resolution) * fg.resolution)
    maxx = float(np.ceil(maxx / fg.resolution) * fg.resolution)
    maxy = float(np.ceil(maxy / fg.resolution) * fg.resolution)

    # ROI 内部统计(minx/miny 已对齐栅格 -> 偏移为整数;ix0/iy0 即栅格内索引)
    roi_sx = int(round((maxx - minx) / fg.resolution))
    roi_sy = int(round((maxy - miny) / fg.resolution))
    gi0 = int(round((minx - fg.ox) / fg.resolution))
    gj0 = int(round((miny - fg.oy) / fg.resolution))
    ii, jj = np.meshgrid(np.arange(roi_sx), np.arange(roi_sy))
    cols = (gj0 + jj) * sx + (gi0 + ii)
    inside = (cols >= 0) & (cols < sx * sy)
    roi_occ = merged[cols[inside]]
    r_free = int((roi_occ == FREE).sum())
    r_occ = int((roi_occ == OCCUPIED).sum())
    r_unk = roi_occ.size - r_free - r_occ

    # 路径实测安全距离:路径每格到融合 OCC(已含膨胀)的最小 L1 距离(格->m)
    if len(path):
        pmask = np.zeros(sx * sy, dtype=bool)
        pmask[np.asarray(path, dtype=np.int64)] = True
        d2d = fg._l1_distance((merged == OCCUPIED).reshape(sy, sx))
        path_safe = float(d2d[pmask.reshape(sy, sx)].min() * fg.resolution)
    else:
        path_safe = -1.0

    out = dict(
        roi=dict(minx=minx, miny=miny, maxx=maxx, maxy=maxy,
                 sx=roi_sx, sy=roi_sy, resolution=fg.resolution,
                 free_area_m2=round(r_free * fg.resolution**2, 2),
                 occ_cells=r_occ, unk_cells=r_unk, occ_frac=round(r_occ/max(1, roi_occ.size), 4),
                 safety_m=fg.inflate_cells * fg.resolution),
        start=dict(x=round(w_sx, 3), y=round(w_sy, 3)),
        goal=dict(x=round(w_gx, 3), y=round(w_gy, 3)),
        astar=dict(reason=stats['reason'], n=len(path), cost=round(stats['cost'], 2),
                   expanded=stats['expanded'], dist_m=round(float(np.hypot(w_gx-w_sx, w_gy-w_sy)), 2),
                   path_min_safe_m=round(path_safe, 2)),
    )
    print(f'[ROI] start=({w_sx:.2f},{w_sy:.2f}) goal=({w_gx:.2f},{w_gy:.2f}) '
          f'dist={out["astar"]["dist_m"]:.2f}m path_n={len(path)} cost={stats["cost"]:.2f}')
    print(f'[ROI映射] minx={minx:.2f} miny={miny:.2f} maxx={maxx:.2f} maxy={maxy:.2f} '
          f'({roi_sx}x{roi_sy} 格={roi_sx*roi_sy*fg.resolution**2:.0f} m^2)')
    print(f'[ROI内部] FREE={r_free} OCC={r_occ} UNK={r_unk} 自由面积={r_free*fg.resolution**2:.0f} m^2')
    print(f'[路径安全] 路径每格到融合OCC最小L1距离={path_safe:.2f}m '
          f'(名义安全距离={fg.inflate_cells*fg.resolution:.2f}m, 应 >= 名义值)')
    print('\n' + json.dumps(out, ensure_ascii=False, indent=2))

    with open(os.path.join(HERE, '..', 'output', 'roi.json'), 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print('\n已写 src/n01_map_fusion/output/roi.json')


if __name__ == '__main__':
    main()