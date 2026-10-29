#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安全距离组合分析:对 safety_radius 做扫描,量化膨胀对 FREE 连通性的影响。

回答:在当前假设(robot_h=1.5, floor_max=0.5, obs_bottom=0.15, min_band=1)下,
0.35m 安全距离是否符合真实 PCD 的通行性?若不符合,最大可行安全距离是多少?
输出:每个半径下 FREE(膨胀后)格数、最大连通分量、能否取出 1.5~2.8m 起终点。
"""
import os
import sys
import numpy as np
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
sys.path.insert(0, PKG)
from n01_map_fusion.pcd_xyz import parse_pcd_xyz
from n01_map_fusion.fusion_core import FusionGrid, FREE, OCCUPIED

PCD = '/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd'


def largest_component(free_mask, sx, sy):
    n = sx * sy
    vis = np.zeros(n, bool)
    best_cells = []
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
            best_cells = cells
    return best_cells


def feasible_pair(cells, sx, sy, res):
    """分量内是否存在两点间距 1.5~2.8m(8-连通分量内 A* 必可达)。

    办法:以分量质心最近格为 start,扫全分量格找距 start 1.5~2.8m 的目标;
    若无,再在分量 bbox 内按长轴两端附近抽样多组 start 重试(防止质心孤点)。
    返回 (ok, start_col, goal_col) 或 (False, None, None)。
    """
    arr = np.asarray([int(c) for c in cells], dtype=np.int64)
    iy, ix = np.unravel_index(arr, (sy, sx))
    min_d2 = (1.5 / res) ** 2
    max_d2 = (2.8 / res) ** 2

    def try_from(sx0, sy0):
        d2 = (ix - sx0) ** 2 + (iy - sy0) ** 2
        idx = np.where((d2 >= min_d2) & (d2 <= max_d2))[0]
        if len(idx) == 0:
            return None
        k = int(idx[np.argmin(d2[idx])])
        return int(arr[0]), int(arr[k])

    cy, cx = float(iy.mean()), float(ix.mean())
    k0 = int(np.argmin((ix - cx) ** 2 + (iy - cy) ** 2))
    r = try_from(int(ix[k0]), int(iy[k0]))
    if r:
        return True, r
    # 备选:bbox 四角顶点作为 start
    for ax, ay in ((ix.min(), iy.min()), (ix.max(), iy.min()),
                   (ix.min(), iy.max()), (ix.max(), iy.max())):
        r = try_from(int(ax), int(ay))
        if r:
            return True, r
    return False, None


def main():
    xs, ys, zs = parse_pcd_xyz(PCD)
    xyz = np.stack([xs, ys, zs], axis=1)

    # 先分类一次(静态层与 safety 无关,仅膨胀半径变)
    fg0 = FusionGrid(robot_h=1.5, floor_max=0.5, obs_bottom=0.15,
                     safety_radius=0.35, min_band_points=1)
    fg0.classify_static(xyz)
    sx, sy, n = fg0.sx, fg0.sy, fg0.n
    static_occ = fg0.static_occ
    static_free = fg0.static_free
    base_free = int(static_free.sum())
    base_occ = int(static_occ.sum())
    print('[静态层] FREE(未膨胀)={} OCC={} UNK={}'.format(base_free, base_occ, n - base_free - base_occ))
    print('=' * 70)
    print('{0:>7} {1:>10} {2:>7} {3:>8} {4:>9} {5:>9} {6:>13}'.format(
        'radius', 'infl_cells', 'FREE', 'OCC', 'max_comp', 'max_area', '1.5-2.8m pair'))

    for radius in (0.10, 0.15, 0.20, 0.25, 0.30, 0.35):
        fg = FusionGrid(robot_h=1.5, floor_max=0.5, obs_bottom=0.15,
                        safety_radius=radius, min_band_points=1)
        # 复用静态分类结果
        fg.static_occ = static_occ.copy()
        fg.static_free = static_free.copy()
        fg.fuse()
        free_infl = fg.fused == FREE
        nfree = int(free_infl.sum())
        cells = largest_component(free_infl, sx, sy)
        nocc_infl = n - nfree - int((fg.fused == -1).sum())

        ok = 'NO(comp<2)' if len(cells) < 2 else 'NO'
        if len(cells) >= 2:
            okf, pair = feasible_pair(cells, sx, sy, fg0.resolution)
            ok = 'YES' if okf else 'NO'
        area_m2 = len(cells) * fg0.resolution ** 2
        print('{0:>7.2f} {1:>10} {2:>7} {3:>8} {4:>9} {5:>9.1f} {6:>13}'.format(
            radius, fg.inflate_cells, nfree, nocc_infl, len(cells), area_m2, ok))


if __name__ == '__main__':
    main()