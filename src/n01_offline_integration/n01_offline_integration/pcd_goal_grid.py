"""Research-only PCD 2D projection + global A* preview; NOT SCAN's local planner.

PCD alone cannot establish floor/unknown free-space truth. Optimistic unknown
traversal MUST be an explicit choice, never a navigation safety claim.
"""
from heapq import heappush, heappop
import json
import math
from pathlib import Path

UNKNOWN, FREE, OBSTACLE = -1, 0, 100


def project(points, *, resolution=.25, floor_z=0., floor_tolerance=.12,
            obstacle_min_z=.2, obstacle_max_z=1.8, clearance=.5):
    if (not .05 <= resolution <= 1.0 or not 0 <= floor_tolerance <= .5 or
            not 0.05 <= obstacle_min_z < obstacle_max_z <= 4 or
            not 0. <= clearance <= 2. or
            not all(math.isfinite(v) for v in (resolution, floor_z,
                  floor_tolerance, obstacle_min_z, obstacle_max_z, clearance))):
        raise ValueError('invalid projection geometry')
    if not points:
        raise ValueError('empty PCD sample')
    xmin=min(p[0] for p in points)-resolution
    ymin=min(p[1] for p in points)-resolution
    xmax=max(p[0] for p in points)+resolution
    ymax=max(p[1] for p in points)+resolution
    w=math.ceil((xmax-xmin)/resolution)
    h=math.ceil((ymax-ymin)/resolution)
    if w*h>2_000_000:
        raise ValueError('projection exceeds 2 million cells')
    grid=[UNKNOWN]*(w*h)
    blocked=set()
    floor_hits=0
    obstacle_hits=0
    for x,y,z in points:
        ix=min(w-1,max(0,int((x-xmin)/resolution)))
        iy=min(h-1,max(0,int((y-ymin)/resolution)))
        i=iy*w+ix
        dz=z-floor_z
        if abs(dz)<=floor_tolerance:
            floor_hits+=1
            grid[i]=FREE
        if obstacle_min_z<=dz<=obstacle_max_z:
            blocked.add((ix,iy))
            obstacle_hits+=1
    radius=math.ceil(clearance/resolution)
    offsets=[(dx,dy) for dx in range(-radius,radius+1)
             for dy in range(-radius,radius+1)
             if math.hypot(dx,dy)*resolution<=clearance+1e-9]
    for x,y in blocked:
        for dx,dy in offsets:
            nx=x+dx;ny=y+dy
            if 0<=nx<w and 0<=ny<h:
                grid[ny*w+nx]=OBSTACLE
    return dict(grid=grid, width=w, height=h, origin=[xmin,ymin],
                resolution=resolution, floor_hits=floor_hits,
                obstacle_hits=obstacle_hits, inflated_occupied_cells=sum(v==OBSTACLE for v in grid),
                free_cells=sum(v==FREE for v in grid),
                unknown_cells=sum(v==UNKNOWN for v in grid),
                clearance_assumption_m=clearance,
                floor_z_assumption_m=floor_z,
                floor_tolerance_m=floor_tolerance,
                obstacle_height_band_m=[obstacle_min_z,obstacle_max_z])


def world_cell(m, xy):
    x,y=xy
    if not all(math.isfinite(v) for v in (x,y)):
        raise ValueError('nonfinite XY')
    ix=math.floor((x-m['origin'][0])/m['resolution'])
    iy=math.floor((y-m['origin'][1])/m['resolution'])
    if not (0<=ix<m['width'] and 0<=iy<m['height']):
        raise ValueError('start/goal outside sampled PCD extents')
    return ix,iy


def route(m,start,goal, *, unknown_policy='block'):
    if unknown_policy not in ('block','optimistic'):
        raise ValueError('unknown policy must be block or optimistic')
    a=world_cell(m,start);b=world_cell(m,goal)
    w=m['width'];h=m['height'];data=m['grid']
    def traversable(x,y):
        c=data[y*w+x]
        return c==FREE or (unknown_policy=='optimistic' and c==UNKNOWN)
    if not traversable(*a) or not traversable(*b):
        raise ValueError('start/goal blocked or unknown under selected policy')
    def heuristic(p):
        dx=abs(p[0]-b[0]);dy=abs(p[1]-b[1]);return max(dx,dy)+(math.sqrt(2)-1)*min(dx,dy)
    current={a:0.0}; parent={};heap=[(heuristic(a),0.0,a)]
    neighbors=[(dx,dy,math.hypot(dx,dy)) for dx in (-1,0,1)
               for dy in (-1,0,1) if dx or dy]
    while heap:
        _,cost,u=heappop(heap)
        if cost>current.get(u,float('inf'))+1e-9:continue
        if u==b:
            cells=[b]
            while cells[-1]!=a:cells.append(parent[cells[-1]])
            cells.reverse()
            path=[[m['origin'][0]+(x+.5)*m['resolution'],
                   m['origin'][1]+(y+.5)*m['resolution']] for x,y in cells]
            return dict(path_xy_m=path, grid_cells=cells,
                        length_m=round(cost*m['resolution'],3))
        x,y=u
        for dx,dy,step in neighbors:
            nx=x+dx;ny=y+dy
            if not (0<=nx<w and 0<=ny<h and traversable(nx,ny)):continue
            # Avoid squeezing diagonally through two blocked corners.
            if dx and dy and (not traversable(x+dx,y) or not traversable(x,y+dy)):
                continue
            v=(nx,ny);newcost=cost+step
            if newcost+1e-9<current.get(v,float('inf')):
                current[v]=newcost;parent[v]=u
                heappush(heap,(newcost+heuristic(v),newcost,v))
    raise ValueError('no route found with selected unknown policy')


def save_outputs(m,outbase,result=None):
    """Portable map preview files; zero ROS publishers, no motion interface."""
    base=Path(outbase)
    base.parent.mkdir(parents=True,exist_ok=True)
    w=m['width'];h=m['height'];g=m['grid']
    # Map-server PGM rows are top-to-bottom; YAML origin is lower-left.
    px={UNKNOWN:205,FREE:254,OBSTACLE:0}
    body=bytes(px[g[y*w+x]] for y in range(h-1,-1,-1) for x in range(w))
    base.with_suffix('.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode()+body)
    yaml=(f'image: {base.name}.pgm\nresolution: {m["resolution"]:.6f}\n'
          f'origin: [{m["origin"][0]:.6f}, {m["origin"][1]:.6f}, 0.0]\n'
          'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')
    base.with_suffix('.yaml').write_text(yaml,encoding='utf-8')
    report={k:v for k,v in m.items() if k!='grid'}
    report.update({'schema':'n01.offline.pcd_goal_preview.v1',
                   'status':'RESEARCH_ONLY_NOT_SCAN_OR_HIVE2_SAFETY',
                   'global_reference_algorithm':'A_STAR_ON_PROJECTED_GRID_NOT_SCAN',
                   'scan_local_optimizer_connected':False,
                   'collision_safety_verified':False,
                   'actuator_output':'DISABLED','safety_authorization':'NEVER',
                   'path':result['path_xy_m'] if result else None,
                   'length_m':result['length_m'] if result else None})
    base.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    # Static SVG carries exact raster occupancy plus optional route overlay.
    parts=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="1100">',
           f'<rect width="{w}" height="{h}" fill="#9da2a6"/>']
    for y in range(h):
        for x in range(w):
            v=g[y*w+x]
            if v==UNKNOWN:continue
            color='#111820' if v==OBSTACLE else '#f1f4f2'
            parts.append(f'<rect x="{x}" y="{h-1-y}" width="1" height="1" fill="{color}"/>')
    if result:
        centers=result['grid_cells']
        coords=' '.join(f'{x+.5:.2f},{h-y-.5:.2f}' for x,y in centers)
        parts.append(f'<polyline points="{coords}" fill="none" stroke="#11bad4" stroke-width="1.8"/>')
        for (x,y),color in ((centers[0],'#35bb66'),(centers[-1],'#fc493e')):
            parts.append(f'<circle cx="{x+.5}" cy="{h-y-.5}" r="2.0" fill="{color}"/>')
    parts.append('</svg>')
    base.with_suffix('.svg').write_text('\n'.join(parts),encoding='utf-8')
    return report
