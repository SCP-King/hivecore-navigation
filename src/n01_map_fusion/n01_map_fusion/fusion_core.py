"""N-01 地图融合核心(纯 numpy,无 ROS 依赖)。

职责:
  - 把静态占用点云(全 3D,含地面)按 0.1m 列分类为 FREE / OCCUPIED / UNKNOWN;
  - 把实时占用点云(地面已过滤)投影为动态占用;
  - 合成融合栅格(占用=静态∪动态,自由只来自静态地面证据,其余一律 UNKNOWN),
    并按车体安全半径做 2D 距离膨胀。

判定依据(与设计说明书 §3 一致):
  - UNKNOWN: 列内无任何点;或列最低点 zmin > floor_max(只有高层结构/悬空,
    无地面证据)。**PCD 无点的位置一律不视为自由空间**。
  - OCCUPIED(静态): 有地面证据(zmin<=floor_max)且带内
    (zmin+obs_bottom, zmin+robot_h] 存在静态点。
  - FREE: 有地面证据且带内无静态点;动态层永远不把列为自由。
  - 动态 OCCUPIED: 实时占用点落入该列。
"""
import numpy as np

UNKNOWN = np.int8(-1)
FREE = np.int8(0)
OCCUPIED = np.int8(100)


class FusionGrid:
    def __init__(self, origin_x=-20.0, origin_y=-18.0, size_x=65.0, size_y=56.0,
                 resolution=0.10, robot_h=1.5, floor_max=0.5,
                 obs_bottom=0.15, safety_radius=0.35, min_band_points=1):
        self.resolution = float(resolution)
        self.ox = float(origin_x)
        self.oy = float(origin_y)
        self.sx = int(round(float(size_x) / self.resolution))
        self.sy = int(round(float(size_y) / self.resolution))
        self.n = self.sx * self.sy
        self.wxmax = self.ox + self.sx * self.resolution
        self.wymax = self.oy + self.sy * self.resolution
        self.robot_h = float(robot_h)
        self.floor_max = float(floor_max)
        self.obs_bottom = float(obs_bottom)
        self.safety_radius = float(safety_radius)
        self.inflate_cells = int(np.ceil(self.safety_radius / self.resolution))
        self.min_band_points = int(min_band_points)

        # 层状态
        self.static = np.full(self.n, UNKNOWN, dtype=np.int8)      # 静态层(-1/0/100)
        self.static_occ = np.zeros(self.n, dtype=bool)             # 静态占用格
        self.static_free = np.zeros(self.n, dtype=bool)            # 静态自由格(地面证据)
        self.dynamic_occ = np.zeros(self.n, dtype=bool)            # 动态占用格
        self.fused = np.full(self.n, UNKNOWN, dtype=np.int8)       # 融合层(-1/0/100,已膨胀)
        self.ground_cols = np.zeros(self.n, dtype=bool)            # 有地面证据的列(诊断)
        self.log = {}                                              # 最近一次统计 dict

    # ---------- 索引 ----------
    # 列索引约定(与 OccupancyGrid 消息一致):
    #   col = iy * sx + ix  ->  data 行主序,std_row 沿 +Y(行数=sy=height),
    #   std_col 沿 +X(列数=sx=width)。发布时 width=sx, height=sy。
    def xy_to_col(self, xs, ys):
        """世界坐标 -> 列整型索引(越界 -> -1)。"""
        ix = np.floor((xs - self.ox) / self.resolution).astype(np.int64)
        iy = np.floor((ys - self.oy) / self.resolution).astype(np.int64)
        ok = (ix >= 0) & (ix < self.sx) & (iy >= 0) & (iy < self.sy)
        col = np.full(ix.shape, -1, dtype=np.int64)
        col[ok] = iy[ok] * self.sx + ix[ok]
        return col

    def col_to_center(self, col):
        if not isinstance(col, np.ndarray):
            col = np.asarray(col)
        iy = col // self.sx
        ix = col % self.sx
        return self.ox + (ix + 0.5) * self.resolution, self.oy + (iy + 0.5) * self.resolution

    # ---------- 静态层分类(全向量化) ----------
    def classify_static(self, pts):
        """pts:(N,3) float 世界系。更新静态层;返回统计 dict。"""
        xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]
        col = self.xy_to_col(xs, ys)
        valid = col >= 0
        xs, ys, zs, col = xs[valid], ys[valid], zs[valid], col[valid]
        n_total = int(pts.shape[0])
        n_dropped = n_total - int(valid.sum())

        self.static.fill(UNKNOWN)
        self.static_occ.fill(False)
        self.static_free.fill(False)
        self.ground_cols.fill(False)
        if col.size == 0:
            self.filled_upto = 0
            self.log = dict(n_static=n_total, n_dropped=n_dropped, free=0,
                            static_occ=0, unknown=self.n, ground=0)
            return dict(self.log)

        # 按列分组(lexsort:(z,col) -> 列内 z 升序,切片=该列全部点),reduceat 统计
        order = np.lexsort((zs, col))       # 主键 col,次键 z(升序)
        col_s = col[order]
        z_s = zs[order]
        keys, start = np.unique(col_s, return_index=True)
        counts = np.diff(np.r_[start, col_s.size])
        zmin = z_s[start]                    # 列内已按 z 升序 -> 首元素即列最低
        zmax = np.maximum.reduceat(z_s, start)
        col_zmin = np.repeat(zmin, counts)
        band = (z_s > col_zmin + self.obs_bottom) & \
               (z_s <= col_zmin + self.obs_bottom + self.robot_h)
        nband = np.bincount(col_s[(band) & (col_zmin <= self.floor_max)],
                            minlength=self.n)
        nband = nband[keys]

        fr = zmin <= self.floor_max
        occ_mask = fr & (nband >= self.min_band_points)
        free_mask = fr & ~occ_mask
        # 地面证据列:列最低点在 floor_max 以下
        ground = fr

        self.static[keys[free_mask]] = FREE
        self.static[keys[occ_mask]] = OCCUPIED
        self.static_occ[keys[occ_mask]] = True
        self.static_free[keys[free_mask]] = True
        self.ground_cols[keys[ground]] = True

        # 高结构列(有静态点但无地面证据)单独计数(诊断):它们= UNKNOWN 但确有观测
        struct = zmin > self.floor_max
        stats = dict(
            n_static=n_total, n_dropped=n_dropped,
            free=int(free_mask.sum()), static_occ=int(occ_mask.sum()),
            ground=int(ground.sum()),
            struct_no_floor=int(struct.sum()),
            unknown=self.n - int(free_mask.sum()) - int(occ_mask.sum()),
        )
        self.log = stats
        return dict(stats)

    # ---------- 动态层 ----------
    def set_dynamic(self, pts):
        """pts:(N,3) 实时占用点(map 系)。整层替换(上游滑窗自带遗忘)。"""
        self.dynamic_occ.fill(False)
        if pts.shape[0] == 0:
            return 0
        col = self.xy_to_col(pts[:, 0], pts[:, 1])
        col = col[col >= 0]
        if col.size:
            self.dynamic_occ[col] = True
        return int(col.size)

    # ---------- 2D L1 距离变换膨胀 ----------
    def _inflate_mask(self, occ):
        """occ:(n,) bool(列索引=iy*sx+ix)-> 返回膨胀掩码,含原占用格。"""
        if not occ.any():
            return np.zeros(self.n, dtype=bool)
        d = self._l1_distance(occ.reshape(self.sy, self.sx))
        return d.ravel() <= self.inflate_cells

    @staticmethod
    def _l1_distance(mask):
        """mask:(sy,sx) bool(rows 沿 +Y,cols 沿 +X)-> 每格到最近 True 的曼哈顿距离。"""
        h, w = mask.shape
        d = np.where(mask, 0, 1e9).astype(np.float64)
        for i in range(1, h):
            row = d[i - 1] + 1.0
            np.minimum(d[i], row, out=d[i])
        for j in range(1, w):
            prev = d[:, j - 1] + 1.0
            np.minimum(d[:, j], prev, out=d[:, j])
        for i in range(h - 2, -1, -1):
            row = d[i + 1] + 1.0
            np.minimum(d[i], row, out=d[i])
        for j in range(w - 2, -1, -1):
            nxt = d[:, j + 1] + 1.0
            np.minimum(d[:, j], nxt, out=d[:, j])
        return d

    # ---------- 融合 ----------
    def fuse(self):
        """合成融合层(OCC=静态∪动态∪膨胀环;FREE 只来自静态地面证据;其余 UNKNOWN)。"""
        occ = self.static_occ | self.dynamic_occ
        infl = self._inflate_mask(occ)
        occ_infl = occ | infl
        f = np.full(self.n, UNKNOWN, dtype=np.int8)
        fr = self.static_free & ~occ_infl
        f[fr] = FREE
        f[occ_infl] = OCCUPIED
        self.fused = f

        stats = dict(
            free=int(fr.sum()),
            inflated= int(occ_infl.sum()) - int(occ.sum()),
            unknown=self.n - int(fr.sum()) - int(occ_infl.sum()),
            inflate_cells=self.inflate_cells,
            inflate_m=self.inflate_cells * self.resolution,
        )
        self.log.update(stats)
        return dict(self.log)

    def grid_2d(self, which='fused'):
        """返回 2D 视图(sy 行 × sx 列),矩阵坐标 (row=y, col=x)。which: fused / static。"""
        if which == 'fused':
            return self.fused.reshape(self.sy, self.sx)
        return self.static.reshape(self.sy, self.sx)