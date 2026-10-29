#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""核心模块单元测试(无 ROS):
  1) FusionGrid.classify_static 与朴素参考实现逐格一致(PCD 全量交叉验证);
  2) A* 在合成可通行网格上按期望出路径且避障/避UNKNOWN;
  3) A* 在合成堵塞网格上正确失败;
  4) nearest_free 吸附正确。
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
sys.path.insert(0, PKG)

from n01_map_fusion.fusion_core import FusionGrid, FREE, OCCUPIED, UNKNOWN
from n01_map_fusion.a_star_core import astar, nearest_free

PCD = '/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd'


def reference_static(xyz, res, ox, oy, sx, sy, floor_max, obs_bottom, robot_h, min_band):
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    ix = np.floor((x - ox) / res).astype(np.int64)
    iy = np.floor((y - oy) / res).astype(np.int64)
    ok = (ix >= 0) & (ix < sx) & (iy >= 0) & (iy < sy)
    by = {}
    for c, zz in zip(iy[ok] * sx + ix[ok], z[ok]):
        by.setdefault(int(c), []).append(float(zz))
    grid = np.full(sx * sy, UNKNOWN, dtype=np.int8)
    for c, zs in by.items():
        zmin = min(zs)
        if zmin > floor_max:
            continue
        nband = sum(1 for zz in zs if zmin + obs_bottom < zz <= zmin + obs_bottom + robot_h)
        grid[c] = OCCUPIED if nband >= min_band else FREE
    return grid


def test_fusion_matches_reference():
    raw = open(PCD, 'rb').read()
    d = raw.find(b'DATA')
    h = raw.find(b'\n', d) + 1
    arr = np.frombuffer(raw[h:], dtype=np.float32)
    xyz = arr[:arr.size // 3 * 3].reshape(-1, 3)
    xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
    fg = FusionGrid(robot_h=1.5, floor_max=0.5, obs_bottom=0.15, safety_radius=0.35)
    fg.classify_static(xyz)
    ref = reference_static(xyz, fg.resolution, fg.ox, fg.oy, fg.sx, fg.sy,
                           fg.floor_max, fg.obs_bottom, fg.robot_h, fg.min_band_points)
    n_diff = int((ref != fg.static).sum())
    print(f'[fusion] 向量化 vs 参考: 不一致格数={n_diff} ({"PASS" if n_diff == 0 else "FAIL"})')
    if n_diff:
        raise SystemExit(1)
    print(f'[fusion] PCD 全量: FREE={(fg.static == FREE).sum()} '
          f'OCC={(fg.static == OCCUPIED).sum()} UNK={(fg.static == UNKNOWN).sum()}')
    assert (fg.static == FREE).sum() == 10768
    assert (fg.static == OCCUPIED).sum() == 4826


def grid_from_ascii(rows, sx, sy):
    """'.'=FREE '#'=OCC '?'=UNKNOWN """
    g = np.full(sx * sy, UNKNOWN, dtype=np.int8)
    for iy, row in enumerate(rows):
        for ix, ch in enumerate(row):
            c = iy * sx + ix
            if ch == '.':
                g[c] = FREE
            elif ch == '#':
                g[c] = OCCUPIED
    return g


def test_astar_open_corridor():
    # 10x10,中间一行全空走横道,起(0,5) 终(9,5)
    g = np.full(100, OCCUPIED, dtype=np.int8)
    for iy in range(4, 7):
        for ix in range(10):
            g[iy * 10 + ix] = FREE
    path, s = astar(g, 10, 10, 5 * 10 + 0, 5 * 10 + 9)
    assert s['reason'] == 'ok', s
    assert path[0] == 5 * 10 + 0 and path[-1] == 5 * 10 + 9
    assert len(path) == 10
    print('[astar] 开放走廊 PASS len=%d cost=%.2f' % (len(path), s['cost']))


def test_astar_diagonal_shortcut():
    # 全空 5x5:曼哈顿 vs 欧氏 -> 应走对角
    g = np.full(25, FREE, dtype=np.int8)
    path, s = astar(g, 5, 5, 0, 24)  # (0,0)->(4,4)
    assert s['reason'] == 'ok', s
    assert abs(s['cost'] - 4 * 2 ** 0.5) < 1e-9, s['cost']
    print('[astar] 对角捷径 PASS cost=%.2f' % s['cost'])


def test_astar_blocked_goal():
    g = np.full(25, FREE, dtype=np.int8)
    g[24] = OCCUPIED
    path, s = astar(g, 5, 5, 0, 24)
    assert path is None and s['reason'] == 'start_or_goal_not_free'
    print('[astar] 目标占用拒绝 PASS:', s['reason'])


def test_astar_unknown_is_blocked():
    # 铁律:UNKNOWN 不视为自由 —— 用 ? 堵死两侧,只留已知自由缝
    rows = ['....?....',
            '#..#?#..#',
            '#..#?..#.',
            '###.?####',
            '...?.....']
    sx, sy = 9, 5
    g = grid_from_ascii(rows, sx, sy)
    # 把第三行整行做成 UNKNOWN(缝被未知堵住)-> 上区到下区不可达
    g[4 * 9: 4 * 9 + 9] = UNKNOWN  # 第三行
    path, s = astar(g, sx, sy, 0, 8)
    assert path is None, 'UNKNOWN 行被当可通行了!'
    print('[astar] UNKNOWN=封闭 PASS:', s['reason'])


def test_astar_unknown_blocks_corridor():
    # 起点(0,0)终点(4,0),中途一格 UNKNOWN,下方两行完全封死绕行
    g = np.full(25, FREE, dtype=np.int8)
    g[2] = UNKNOWN
    g[5:15] = OCCUPIED
    path, s = astar(g, 5, 5, 0, 4)
    assert path is None, 'UNKNOWN 中缝应不可达'
    print('[astar] 未知一格阻断 PASS:', s['reason'])


def test_nearest_free_snap():
    g = np.full(25, OCCUPIED, dtype=np.int8)
    g[13] = FREE  # (ix=3,iy=2)
    c, d = nearest_free(g, 5, 5, 24, radius_cells=3)   # 起点 (4,4),距离 √(1²+2²)=√5
    assert c == 13 and abs(d - 5 ** 0.5) < 1e-9, (c, d)
    c, d = nearest_free(g, 5, 5, 24, radius_cells=1)
    assert c is None and d is None, '半径外不可达应返回 None'
    print('[snap] 吸附 PASS')


def main():
    test_fusion_matches_reference()
    test_astar_open_corridor()
    test_astar_diagonal_shortcut()
    test_astar_blocked_goal()
    test_astar_unknown_is_blocked()
    test_astar_unknown_blocks_corridor()
    test_nearest_free_snap()
    print('\n全部单元测试通过')


if __name__ == '__main__':
    main()