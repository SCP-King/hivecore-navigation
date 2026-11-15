"""Bounded forward-only SE(2) lattice search; no actuator or ROS dependencies.

Curvature is ramped along arc length, not jumped at primitive boundaries.
The caller supplies a fresh effective obstacle layer and its valid window.
"""
import heapq
import math
import time
from dataclasses import dataclass
import numpy as np
from scipy.spatial import cKDTree

FOOTPRINT_CENTERS = (-.43, -.10, .23)
FOOTPRINT_RADIUS = .30
MAX_CURVATURE = 1.0
ENVIRONMENT_MARGIN = .06


def footprint_description(extra_margin=0.):
    return {'model':'three_disks','centers_x_m':list(FOOTPRINT_CENTERS),
            'physical_radius_m':FOOTPRINT_RADIUS,
            'radius_m':FOOTPRINT_RADIUS+extra_margin,
            'environment_margin_m':ENVIRONMENT_MARGIN,
            'collision_query_radius_m':FOOTPRINT_RADIUS+ENVIRONMENT_MARGIN+extra_margin,
            'basis':'heading_search_collision_query'}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class FootprintCollision:
    def __init__(self, points, plane, z, bounds, voxel_size=.05, footprint=None, height_margin=0.):
        p = np.asarray(points, float).reshape((-1, 3))
        plane = np.asarray(plane, float)
        self.bounds = np.asarray(bounds, float)
        if (plane.shape != (3,) or self.bounds.shape != (2, 2)
                or not np.isfinite(plane).all() or not np.isfinite(self.bounds).all()
                or not math.isfinite(z) or not math.isfinite(height_margin) or not 0<=height_margin<=.05 or not 0 <= voxel_size <= .1
                or np.any(self.bounds[1] <= self.bounds[0])):
            raise ValueError('invalid_collision_geometry')
        p = p[np.isfinite(p).all(axis=1)]
        floor = p[:, :2] @ plane[:2] + plane[2]
        p = p[(p[:, 2] > floor+.03) & (p[:, 2] >= z-.90-height_margin) & (p[:, 2] <= z+.25+height_margin)]
        self.points=p[:,:2]
        self.polygon=None if footprint is None else np.asarray(footprint,float)
        if self.polygon is not None:
            from .robot_footprint import polygon_hits
            polygon_hits([[0.,0.]],self.polygon)
        self.tree = cKDTree(p[:, :2]) if len(p) else None
        # Occupancy points are voxel centres. Cover their half diagonal plus
        # the maximum displacement between 2 cm collision samples.
        self.radius = (ENVIRONMENT_MARGIN if self.polygon is not None else
                       FOOTPRINT_RADIUS + max(ENVIRONMENT_MARGIN,math.sqrt(2)*voxel_size/2+.02))

    def free_batch(self, trajectories):
        s = np.asarray(trajectories, float)
        if s.ndim != 3 or s.shape[2] < 3 or not np.isfinite(s).all():
            raise ValueError('invalid_collision_trajectories')
        if self.polygon is not None:
            from .robot_footprint import polygon_hits
            rho=float(np.linalg.norm(self.polygon,axis=1).max())+self.radius
            flat=s.reshape(-1,s.shape[-1]);c=np.cos(flat[:,2]);sn=np.sin(flat[:,2])
            px=self.polygon[:,0];py=self.polygon[:,1]
            wx=flat[:,0,None]+c[:,None]*px-sn[:,None]*py
            wy=flat[:,1,None]+sn[:,None]*px+c[:,None]*py
            hit=((wx.min(axis=1)-self.radius<self.bounds[0,0]) |
                 (wx.max(axis=1)+self.radius>self.bounds[1,0]) |
                 (wy.min(axis=1)-self.radius<self.bounds[0,1]) |
                 (wy.max(axis=1)+self.radius>self.bounds[1,1]))
            if self.tree is not None:
                groups=self.tree.query_ball_point(flat[:,:2],rho)
                counts=np.array([len(g)for g in groups])
                if counts.sum():
                    owners=np.repeat(np.arange(len(flat)),counts)
                    ids=np.concatenate([g for g in groups if len(g)]).astype(int)
                    delta=self.points[ids]-flat[owners,:2]
                    local=np.column_stack((c[owners]*delta[:,0]+sn[owners]*delta[:,1],
                                           -sn[owners]*delta[:,0]+c[owners]*delta[:,1]))
                    for begin in range(0,len(local),1024):
                        mask=polygon_hits(local[begin:begin+1024],self.polygon,self.radius)
                        hit[owners[begin:begin+1024][mask]]=True
            return ~hit.reshape(s.shape[0],s.shape[1]).any(axis=1)
        centers = np.asarray(FOOTPRINT_CENTERS)
        direction = np.stack((np.cos(s[:, :, 2]), np.sin(s[:, :, 2])), axis=-1)
        xy = s[:, :, None, :2] + direction[:, :, None, :] * centers[None, None, :, None]
        outside = np.any((xy-self.radius < self.bounds[0]) |
                         (xy+self.radius > self.bounds[1]), axis=(1,2,3))
        if self.tree is None:
            return ~outside
        # One bounded-radius KD-tree call for all five successors. Avoid five
        # nearest-neighbour searches extending far beyond the collision radius.
        distance = self.tree.query(xy.reshape((-1,2)), distance_upper_bound=self.radius)[0]
        hit = np.any(np.isfinite(distance.reshape(s.shape[0], s.shape[1], 3)), axis=(1,2))
        return ~(outside | hit)

    def free(self, states):
        s = np.asarray(states, float)
        if s.ndim != 2 or s.shape[1] < 3 or not np.isfinite(s).all():
            return False
        return bool(self.free_batch(s[None, ...])[0])


def primitive(state, target_curvature, length=.20, ds=.02):
    x, y, yaw, curvature = state
    samples = []
    count = int(math.ceil(length/ds)); step = length/count
    for _ in range(count):
        next_k = curvature + max(-2*step, min(2*step, target_curvature-curvature))
        angle = .5*(curvature+next_k)*step
        mid = yaw+.5*angle
        x += step*math.cos(mid); y += step*math.sin(mid); yaw += angle
        curvature = next_k
        samples.append((x, y, yaw, curvature))
    return samples


@dataclass
class SearchResult:
    states: list
    reason: str
    expanded: int
    elapsed_ms: float


def search(start, goal, collision, goal_yaw=None, budget_s=.12, max_expansions=2500, capability=None):
    """Reach a local waypoint within 6 cm, without snapping the final pose.

    Failure never returns a partial path for execution. Search is bounded in
    both CPU time and nodes; native four-wheel validation is still mandatory.
    """
    began = time.monotonic()
    start = tuple(map(float, start))
    if len(start) != 4 or not all(math.isfinite(v) for v in (*start, *goal)) or abs(start[3]) > (capability.max_curvature if capability is not None else MAX_CURVATURE):
        raise ValueError('invalid_search_start_or_goal')
    if not collision.free([start]):
        return SearchResult([], 'start_footprint_occupied_or_outside_window', 0, 0.)
    key = lambda s: (round(s[0]/.06), round(s[1]/.06), round(wrap(s[2])/.0872665), round(s[3]/.25))
    distance = lambda s: math.hypot(s[0]-goal[0], s[1]-goal[1])
    nodes = [(start, -1, [], 0.)]
    best = {key(start): 0.}
    heap = [(distance(start), 0)]
    expanded = 0; found = None; reason = 'no_forward_route'
    while heap and expanded < max_expansions:
        if time.monotonic()-began > budget_s:
            reason = 'heading_search_time_budget'; break
        _, idx = heapq.heappop(heap)
        state, parent, segment, cost = nodes[idx]
        if cost > best.get(key(state), math.inf)+1e-9:
            continue
        expanded += 1
        if distance(state) <= .06 and idx != 0 and (goal_yaw is None or abs(wrap(state[2]-goal_yaw)) <= math.radians(25)):
            found = idx; break
        choices = capability.choices if capability is not None else (-1., -.5, 0., .5, 1.)
        segments = [primitive(state, k, length=min(.20, max(.04, distance(state)))) for k in choices]
        # Curvature slew makes many distant target curvatures produce the
        # exact same short primitive. Check its geometry once, keeping the
        # lowest original cost; never discard a distinct swept trajectory.
        unique={}
        for k,segment in zip(choices,segments):
            signature=tuple(np.asarray(segment).round(10).ravel())
            penalty=.01*abs(k)+.02*abs(k-state[3])
            if signature not in unique or penalty<unique[signature][0]:
                unique[signature]=(penalty,k,segment)
        choices=[v[1] for v in unique.values()]
        segments=[v[2] for v in unique.values()]
        safe = collision.free_batch(segments)
        for curvature, segment, is_free in zip(choices, segments, safe):
            if not is_free or (capability is not None and not capability.segment_allowed(segment)):
                continue
            end = segment[-1]
            new_cost = cost+len(segment)*.02+.01*abs(curvature)+.02*abs(curvature-state[3])
            k = key(end)
            if new_cost >= best.get(k, math.inf)-1e-9:
                continue
            best[k] = new_cost
            nodes.append((end, idx, segment, new_cost))
            heapq.heappush(heap, (new_cost+1.2*distance(end), len(nodes)-1))
    states = []
    if found is not None:
        segments = []
        while found > 0:
            state, parent, segment, cost = nodes[found]
            segments.append(segment); found = parent
        states = [start] + [s for seg in reversed(segments) for s in seg]
        reason = 'forward_heading_path'
    elif expanded >= max_expansions:
        reason = 'heading_search_expansion_budget'
    return SearchResult(states, reason, expanded, (time.monotonic()-began)*1000)


def revalidate_cached(states, start, collision, max_offset=.02):
    """Trim only; never invent a connector or move old geometry to a new pose.

    Offset uncertainty is added to collision radius. Heading and curvature
    compatibility are necessary, and the caller must repeat native IK review.
    """
    if len(states) < 4:
        return None
    idx = min(range(len(states)), key=lambda i: math.dist(states[i][:2], start[:2]))
    tail = states[idx:]
    offset = math.dist(tail[0][:2], start[:2])
    if (len(tail) < 4 or offset > max_offset or abs(wrap(tail[0][2]-start[2])) > math.radians(2)
            or abs(tail[0][3]-start[3]) > .15):
        return None
    radius = collision.radius
    try:
        collision.radius += offset
        return tail if collision.free(tail) else None
    finally:
        collision.radius = radius
