"""Map-frame floor estimation; independent of robot/SLAM pose height."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class GroundPlane:
    a: float
    b: float
    c: float
    support: int
    residual: float

    def height(self, x, y):
        return self.a*x + self.b*y + self.c


def estimate_ground(points, xy, radius=3.0, body_z=None, height_hint=None):
    """Fit a broad low surface near the robot, rejecting ambiguous/sparse scenes.

    This is a candidate floor for locally planar scenes, not terrain mapping.
    Use cell minima to prevent dense walls from dominating the fit. Optional
    body_z only bounds the search below the chassis; it never supplies a floor
    value or imposes a nominal leg height. This same search must be used by
    offline PCD synchronization and live footprint projection.
    """
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3:
        raise ValueError('地图点云格式无效')
    p = p[np.isfinite(p).all(axis=1)]
    if body_z is not None:
        if not np.isfinite(float(body_z)):
            raise ValueError('机身参考高度无效，无法选择地面搜索区域')
        p = p[(p[:, 2] < float(body_z)-.15) & (p[:, 2] > float(body_z)-1.5)]
    p = p[np.linalg.norm(p[:, :2]-xy, axis=1) <= radius]
    if len(p) < (12 if height_hint is not None else 100):
        raise ValueError('附近地面候选点不足，无法判断地面')
    if height_hint is not None:
        height,tolerance=height_hint
        if not np.isfinite([height,tolerance]).all() or not 0<tolerance<=.2:
            raise ValueError('接地高度提示无效')
        p=p[np.abs(p[:,2]-height)<=tolerance]
        if len(p)<12:raise ValueError('轮模型附近的实测地面回波不足')
    cells = np.floor((p[:, :2]-xy)/.25).astype(int)
    order = np.argsort(p[:, 2])
    _, first = np.unique(cells[order], axis=0, return_index=True)
    low = p[order[first]]
    # Floor should occupy the lower portion, not a tabletop or ceiling.
    low = low[low[:, 2] <= np.percentile(low[:, 2], 60)+.1]
    minimum=12 if height_hint is not None else 30
    if len(low) < minimum:
        raise ValueError('低处表面覆盖不足，无法判断地面')
    rng = np.random.default_rng(42)
    best = None
    hypotheses = []
    for _ in range(512):
        sample = low[rng.choice(len(low), 3, replace=False)]
        n = np.cross(sample[1]-sample[0], sample[2]-sample[0])
        if abs(n[2]) < 1e-8 or np.linalg.norm(n[:2])/abs(n[2]) > np.tan(np.deg2rad(10)):
            continue
        coeff = np.array([-n[0]/n[2], -n[1]/n[2], np.dot(n, sample[0])/n[2]])
        mask = np.abs(low[:, 2] - np.column_stack((low[:, :2], np.ones(len(low))))@coeff) < .03
        hypotheses.append((coeff,mask))
        # Furniture changes the denominator without weakening the same floor.
        # Select only broad, near-level candidates; a larger sloped patch must
        # not suppress a smaller valid floor.
        if mask.sum()>=minimum and np.linalg.norm(coeff[:2])<=np.tan(np.deg2rad(2)):
            spread=np.linalg.eigvalsh(np.cov(low[mask,:2].T))
            if spread.min()>=.06 and spread.max()>=.20:
                if best is None or mask.sum()>best.sum():best=mask
    if best is None:
        raise ValueError(f'尚无覆盖充分的近水平地面（候选网格{len(low)}）；'
                         '请增加地面可见范围，不会按固定比例拒绝候选')
    q = low[best]
    spread=np.linalg.eigvalsh(np.cov(q[:, :2].T))
    if spread.min()<.06 or spread.max()<.20:
        raise ValueError('地面覆盖过窄，无法可靠拟合')
    coeff = np.linalg.lstsq(np.column_stack((q[:, :2], np.ones(len(q)))), q[:, 2], rcond=None)[0]
    # A minority plane can still be the floor amid furniture. Reject a second
    # similarly supported, spatially broad, distinct navigable level instead
    # of treating every low global support ratio as a multi-floor scene.
    center_height=float(np.dot(coeff,(*xy,1.)))
    for candidate,mask in hypotheses:
        if (mask.sum()<max(minimum,.75*best.sum())
                or np.linalg.norm(candidate[:2])>np.tan(np.deg2rad(2))
                or np.count_nonzero(mask & best)>.25*min(mask.sum(),best.sum())):
            continue
        if (abs(float(np.dot(candidate,(*xy,1.)))-center_height)>.12
                and np.linalg.eigvalsh(np.cov(low[mask,:2].T)).min()>=.06):
            raise ValueError('存在高度不同且支持相近的两个平面，无法唯一确定地面')
    # Cell minima locate the surface but bias height downward in noisy scans.
    # Refine against original returns close to that surface.
    design = np.column_stack((p[:, :2], np.ones(len(p))))
    for tolerance in (.06, .04, .03):
        inliers = np.abs(p[:, 2]-design@coeff) < tolerance
        if inliers.sum()<minimum:
            raise ValueError('地面细化后支持不足，无法可靠拟合')
        coeff = np.linalg.lstsq(design[inliers], p[inliers, 2], rcond=None)[0]
    q = p[inliers]
    residual = float(np.percentile(np.abs(q[:, 2]-np.column_stack((q[:, :2], np.ones(len(q))))@coeff), 95))
    if np.linalg.norm(coeff[:2]) > np.tan(np.deg2rad(2)):
        raise ValueError('地图地面倾斜超过2°，请先检查标定/地图水平，不能用单一平面导航')
    return GroundPlane(*map(float, coeff), len(q), residual)
