"""最小、独立的 8 邻域 A* 核心(纯 Python,无 ROS)。

网格几何与 OccupancyGrid 一致:
  - sx = 沿 +X 的格数(width),sy = 沿 +Y 的格数(height);
  - 列索引 col = iy*sx + ix(data 行主序,y 在外层);
  - iy = col//sx, ix = col%sx;合法范围 0<=ix<sx, 0<=iy<sy。
"""
import heapq
import math


def astar(grid, sx, sy, start, goal, max_expansions=2000000):
    """在 1D int8 OccupancyGrid 上规划;仅 FREE(0) 可通行。

    返回 (path_cols, stats);失败 path_cols=None,stats['reason'] 说明原因。
    """
    start = int(start)
    goal = int(goal)
    n = sx * sy
    if not (0 <= start < n and 0 <= goal < n):
        return None, {'reason': 'start_or_goal_out_of_bounds', 'expanded': 0}
    if int(grid[start]) != 0 or int(grid[goal]) != 0:
        return None, {'reason': 'start_or_goal_not_free', 'expanded': 0}
    if start == goal:
        return [start], {'reason': 'ok', 'expanded': 0, 'cost': 0.0}

    gx = goal % sx
    gy = goal // sx
    inf = float('inf')
    g = [inf] * n
    parent = [-1] * n
    g[start] = 0.0
    heap = []

    def h(c):
        ix = c % sx
        iy = c // sx
        return math.hypot(ix - gx, iy - gy)

    heapq.heappush(heap, (h(start), 0.0, start))
    closed = bytearray(n)
    expanded = 0
    # (dix, diy, step)
    nbrs = ((-1, -1, math.sqrt(2.0)), (0, -1, 1.0), (1, -1, math.sqrt(2.0)),
            (-1, 0, 1.0), (1, 0, 1.0),
            (-1, 1, math.sqrt(2.0)), (0, 1, 1.0), (1, 1, math.sqrt(2.0)))
    while heap:
        _, cur_g, cur = heapq.heappop(heap)
        if closed[cur] or cur_g != g[cur]:
            continue
        closed[cur] = 1
        expanded += 1
        if expanded > max_expansions:
            return None, {'reason': 'max_expansions', 'expanded': expanded}
        if cur == goal:
            path = []
            p = cur
            while p >= 0:
                path.append(p)
                p = parent[p]
            path.reverse()
            return path, {'reason': 'ok', 'expanded': expanded, 'cost': g[cur]}
        ix = cur % sx
        iy = cur // sx
        for dx, dy, step in nbrs:
            nx, ny = ix + dx, iy + dy
            if nx < 0 or nx >= sx or ny < 0 or ny >= sy:
                continue
            if dx and dy:
                # 对角步:两个正交邻格必须也 FREE,否则直线路径会穿 OCC/UNKNOWN 格角
                if int(grid[iy * sx + nx]) != 0 or int(grid[ny * sx + ix]) != 0:
                    continue
            nxt = ny * sx + nx
            if closed[nxt] or int(grid[nxt]) != 0:
                continue
            cand = cur_g + step
            if cand < g[nxt]:
                g[nxt] = cand
                parent[nxt] = cur
                heapq.heappush(heap, (cand + h(nxt), cand, nxt))
    return None, {'reason': 'no_path', 'expanded': expanded}


def nearest_free(grid, sx, sy, col, radius_cells):
    """在半径内找欧氏距离最近的 FREE(0) 格;返回 (col, distance_cells),(None,None) 表示无。"""
    col = int(col)
    ix0 = col % sx
    iy0 = col // sx
    r = int(radius_cells)
    if r < 0:
        r = 0
    best = None
    best_d2 = None
    for iy in range(max(0, iy0 - r), min(sy, iy0 + r + 1)):
        for ix in range(max(0, ix0 - r), min(sx, ix0 + r + 1)):
            d2 = (ix - ix0) ** 2 + (iy - iy0) ** 2
            if d2 > r * r:
                continue
            c = iy * sx + ix
            if int(grid[c]) == 0:
                if best_d2 is None or d2 < best_d2:
                    best, best_d2 = c, d2
    if best is None:
        return None, None
    return best, math.sqrt(best_d2)