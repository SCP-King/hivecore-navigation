#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修正版静态地图分析 v2:
  - 正确分组:lexsort((z,col)),列内 z 升序,切片=该列全部点;并与逐列参考实现交叉验证。
  - 分类参数扫描:robot_h(车体可通过高度) x min_band_points(带内噪声门) x 是否膨胀。
  - 连通性测度:在"原始自由集"和"按 safety_radius 膨胀后的自由集"上分别做 BFS,
    并统计轨迹点落格与最远可达轨迹目标(不预设任何可达结论)。

参考:docs/N-01静态动态融合与A星路径规划_设计说明书_20261028.md §3
"""
import numpy as np
import sqlite3
import json
from collections import deque

PCD = '/home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd'
BAG_DB = '/home/demo/lyy/slam_data/bags/bag_20261027_225100/bag_20261027_225100_0.db3'
RES = 0.10
OX, OY = -20.0, -18.0
SX, SY = 650, 560
UNKNOWN, FREE, OCC = -1, 0, 100


def load_pcd():
    raw = open(PCD, 'rb').read()
    d = raw.find(b'DATA')
    h = raw.find(b'\n', d) + 1
    arr = np.frombuffer(raw[h:], dtype=np.float32)
    xyz = arr[:arr.size // 3 * 3].reshape(-1, 3)
    return xyz[np.all(np.isfinite(xyz), axis=1)]


def load_traj():
    import tf2_msgs.msg
    from rclpy.serialization import deserialize_message
    con = sqlite3.connect(BAG_DB)
    cur = con.cursor()
    rows = cur.execute(
        "select m.data from messages m join topics t on t.id=m.topic_id "
        "where t.name='/tf' order by m.timestamp").fetchall()
    tr = []
    for (d,) in rows:
        tf = deserialize_message(d, tf2_msgs.msg.TFMessage)
        for t in tf.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'base_footprint':
                tr.append((t.transform.translation.x, t.transform.translation.y))
    con.close()
    return np.asarray(tr)


def column_index(x, y):
    ix = np.floor((x - OX) / RES).astype(np.int64)
    iy = np.floor((y - OY) / RES).astype(np.int64)
    ok = (ix >= 0) & (ix < SX) & (iy >= 0) & (iy < SY)
    col = np.full(x.shape, -1, dtype=np.int64)
    col[ok] = iy[ok] * SX + ix[ok]
    return col, ok


def col_centers(col):
    col = np.asarray(col)
    iy = col // SX
    ix = col % SX
    return OX + (ix + 0.5) * RES, OY + (iy + 0.5) * RES


def reference_classify(xyz, floor_max, obs_bottom, robot_h, min_band_points):
    """逐列参考实现(简单循环,无分组技巧),用于交叉验证向量化实现。"""
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    col, ok = column_index(x, y)
    by = {}
    for c, zz in zip(col[ok], z[ok]):
        by.setdefault(int(c), []).append(float(zz))
    grid = np.full(SY * SX, UNKNOWN, dtype=np.int8)
    for c, zs in by.items():
        zmin = min(zs)
        if zmin > floor_max:
            continue
        nband = sum(1 for zz in zs if zmin + obs_bottom < zz <= zmin + obs_bottom + robot_h)
        grid[c] = OCC if nband >= min_band_points else FREE
    return grid


def classify(xyz, floor_max=0.5, obs_bottom=0.15, robot_h=1.5, min_band_points=1):
    """向量化分类(修正分组:列内 z 升序,切片=列内全部点)。"""
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    col, ok = column_index(x, y)
    grid = np.full(SY * SX, UNKNOWN, dtype=np.int8)
    if ok.sum() == 0:
        return grid
    order = np.lexsort((z[ok], col[ok]))
    cs = col[ok][order]
    zs = z[ok][order]
    u, starts, counts = np.unique(cs, return_index=True, return_counts=True)
    zmin = zs[starts]                       # 列内已按 z 升序 -> 首元素即列最低
    col_zmin = np.repeat(zmin, counts)
    band = (zs > col_zmin + obs_bottom) & (zs <= col_zmin + obs_bottom + robot_h)
    nband = np.bincount(cs[band & (col_zmin <= floor_max)], minlength=SY * SX)
    fr = zmin <= floor_max
    occ_m = fr & (nband[u] >= min_band_points)
    free_m = fr & ~occ_m
    grid[u[occ_m]] = OCC
    grid[u[free_m]] = FREE
    return grid


def inflate(grid, safety_radius):
    """L1 距离膨胀:返回原始 free 集减去(膨胀后的占用/未知)。返回自由 bool 集合。"""
    occ_free = (grid == FREE)
    if safety_radius <= 0:
        return occ_free
    occ = (grid == OCC)
    # L1 距离变换(每格到最近障碍的曼哈顿距离)
    G = occ.reshape(SY, SX)
    d = np.where(G, 0, 1e9).astype(np.float64)
    for i in range(1, SY):
        np.minimum(d[i], d[i - 1] + 1.0, out=d[i])
    for j in range(1, SX):
        np.minimum(d[:, j], d[:, j - 1] + 1.0, out=d[:, j])
    for i in range(SY - 2, -1, -1):
        np.minimum(d[i], d[i + 1] + 1.0, out=d[i])
    for j in range(SX - 2, -1, -1):
        np.minimum(d[:, j], d[:, j + 1] + 1.0, out=d[:, j])
    k = int(np.ceil(safety_radius / RES))
    ok_free = occ_free.reshape(SY, SX) & (d > k)
    return ok_free.ravel()


def nearest_free(grid_free, x, y, radius_m=1.5):
    c, ok = column_index(np.asarray([x]), np.asarray([y]))
    c = int(c[0])
    if c < 0:
        return None, None, None, None
    cx, cy = c % SX, c // SX
    r = int(radius_m / RES)
    best, best_d = None, None
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if dx * dx + dy * dy > r * r:
                continue
            ix_, iy_ = cx + dx, cy + dy
            if 0 <= ix_ < SX and 0 <= iy_ < SY:
                cc = iy_ * SX + ix_
                if grid_free[cc]:
                    d2 = dx * dx + dy * dy
                    if best_d is None or d2 < best_d:
                        best, best_d = cc, d2
    if best is None:
        return None, None, None, None
    bx, by = col_centers(np.asarray([best]))
    return best, float(bx[0]), float(by[0]), float(np.sqrt(best_d) * RES)


def bfs_component(grid_free, start_col):
    q = deque([start_col])
    seen = {start_col}
    while q:
        c = q.popleft()
        iy, ix = divmod(c, SX)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = ix + dx, iy + dy
                if 0 <= nx < SX and 0 <= ny < SY:
                    n = ny * SX + nx
                    if n not in seen and grid_free[n]:
                        seen.add(n)
                        q.append(n)
    return seen


def main():
    xyz = load_pcd()
    traj = load_traj()
    print(f"PCD {len(xyz)} pts; 轨迹 {len(traj)} 点")

    # ---- 修正分组正确性交叉验证 ----
    ref = reference_classify(xyz, 0.5, 0.15, 1.5, 1)
    vec = classify(xyz, 0.5, 0.15, 1.5, 1)
    agree = (ref == vec)
    n_diff = int((~agree).sum())
    print(f"[交叉验证] 参考循环实现 vs 向量化实现: 不一致格数={n_diff} "
          f"({'PASS' if n_diff == 0 else 'FAIL'})")

    # ---- 分类统计 + 连通性扫描 ----
    goals = dict(g12=(16.728, 12.553, 'bag穿行最远点'), gx16=(16.728, 0.0, 'x最大点'),
                 gmid=(8.0, 6.0, '中部点'))
    print("\n=== 连通性扫描(ravel/safety=0.35m) 全在膨胀后的自由集上测量 ===")
    summary = {}
    for robot_h in (1.2, 1.5):
        for mband in (1, 2):
            grid = classify(xyz, 0.5, 0.15, robot_h, mband)
            n_free = int((grid == FREE).sum()); n_occ = int((grid == OCC).sum())
            free_raw = (grid == FREE)
            free_infl = inflate(grid, 0.35)
            s_col, sx0, sy0, sd = nearest_free(free_infl, 0.0, 0.0)
            comp = bfs_component(free_infl, s_col) if s_col is not None else set()
            traj_col, _ = column_index(traj[:, 0], traj[:, 1])
            in_comp = sum(1 for c in traj_col if c in comp)
            # 出发点在最远两个目标上的可达性
            reach = {}
            for name, (gx, gy, _) in goals.items():
                gc, ggx, ggy, gd = nearest_free(free_infl, gx, gy)
                reach[name] = bool(gc is not None) and (gc in comp) and bool(gd is not None and gd <= 2.0)
            summary[f"h{robot_h}_m{mband}"] = dict(free=n_free, occ=n_occ,
                                                   free_infl=int(free_infl.sum()),
                                                   comp_n=len(comp), traj_in=n_free and in_comp,
                                                   reach=reach)
            print(f"robot_h={robot_h} mband={mband}: FREE({n_free}) OCC({n_occ}) 膨胀后自由({int(free_infl.sum())}) "
                  f"| 起点分量 {len(comp)} | 轨迹点在其中 {in_comp}/{len(traj)} "
                  f"| 目标可达: " + ", ".join(f"{k}:{v}" for k, v in reach.items()))

    # ---- 默认参数详细剖面 ----
    grid = classify(xyz, 0.5, 0.15, 1.5, 1)
    free_infl = inflate(grid, 0.35)
    s_col, sx0, sy0, sd = nearest_free(free_infl, 0.0, 0.0)
    comp = bfs_component(free_infl, s_col)
    traj_col, _ = column_index(traj[:, 0], traj[:, 1])
    # 最远可达轨迹点
    best = None
    for c in traj_col:
        if c in comp:
            cx, cy = col_centers(np.asarray([c]))
            dist = float(np.hypot(cx[0] - 0.0, cy[0] - 0.0))
            if best is None or dist > best[0]:
                best = (dist, cx[0], cy[0])
    print(f"\n[默认 robot_h=1.5,mband=1,safety=0.35] 起点吸附 {sd:.2f}m -> 分量 {len(comp)} 格 "
          f"({len(comp)*RES*RES:.0f} m^2)")
    print(f"轨迹点落分量: {sum(1 for c in traj_col if c in comp)}/{len(traj)}; "
          f"最远可达轨迹点距原点 {best[0]:.1f}m @ ({best[1]:.1f},{best[2]:.1f})")

    # 各目标在未膨胀自由集上的原始可达性(诊断:膨胀是不是当前不可达的主要原因)
    print("\n[诊断] 未膨胀自由集上的目标可达性(robot_h=1.5):")
    free_raw = (grid == FREE)
    s0, _, _, _ = nearest_free(free_raw, 0.0, 0.0)
    comp_raw = bfs_component(free_raw, s0)
    for name, (gx, gy, desc) in goals.items():
        gc, _, _, gd = nearest_free(free_raw, gx, gy)
        reach = gc is not None and gc in comp_raw
        print(f"  {name} ({desc}): 吸附 {gd:.2f}m, 原始自由集可达={reach}")

    json.dump(summary, open('/home/demo/lyy/hivecore-navigation/n01_static_map/output/analysis_v2.json', 'w'),
              indent=1)
    print("\n结果已写 n01_static_map/output/analysis_v2.json")


if __name__ == '__main__':
    main()