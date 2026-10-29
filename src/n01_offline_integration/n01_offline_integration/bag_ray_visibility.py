"""Offline 2D ray-visibility evidence from true sensor origins and endpoints.

A free ray cell means observed at one near-horizontal laser height only;
it is NOT a robot footprint, ground or full vertical clearance certificate.
No ROS, publishers, actuator APIs, or global navigation safety assertions.
"""
from array import array
import math

UNKNOWN, FREE, OCCUPIED = -1, 0, 100


class RayVisibilityGrid:
    def __init__(self, bounds_xy, resolution=.25, max_range_m=5.,
                 max_vertical_delta_m=.35, min_free_rays=3):
        if (not 0.05 <= resolution <= 1.0 or
                not .5 <= max_range_m <= 30. or
                not 0.05 <= max_vertical_delta_m <= 1.0 or
                type(min_free_rays) is not int or not 1 <= min_free_rays <= 100 or
                len(bounds_xy) != 2 or
                any(len(ab)!=2 or not all(math.isfinite(v) for v in ab)
                    for ab in bounds_xy)):
            raise ValueError("invalid ray visibility grid parameters")
        (x0,x1),(y0,y1)=bounds_xy
        if x1 <= x0 or y1 <= y0:
            raise ValueError("invalid bounding box")
        self.resolution=resolution
        self.origin=(x0-resolution,y0-resolution)
        self.width=math.ceil((x1-x0+2*resolution)/resolution)
        self.height=math.ceil((y1-y0+2*resolution)/resolution)
        if self.width*self.height>2_000_000:
            raise ValueError("grid exceeds 2 million cells")
        self.free_votes=array("I",[0])*(self.width*self.height)
        self.hit_votes=array("I",[0])*(self.width*self.height)
        self.max_range_m=max_range_m
        self.max_vertical_delta_m=max_vertical_delta_m
        self.min_free_rays=min_free_rays
        self.stats=dict(rays_considered=0,rays_accepted=0,
                        rejected_vertical=0,rejected_out_of_bounds=0,
                        rejected_range=0,rejected_nonfinite=0)

    def cell(self, p):
        x,y=p[:2]
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        a=math.floor((x-self.origin[0])/self.resolution)
        b=math.floor((y-self.origin[1])/self.resolution)
        return (a,b) if 0<=a<self.width and 0<=b<self.height else None

    def ray(self, sensor_xyz, endpoint_xyz):
        """Each 3D endpoint must be transformed using that scan's TF stamp."""
        self.stats["rays_considered"]+=1
        if (len(sensor_xyz)!=3 or len(endpoint_xyz)!=3 or
                not all(math.isfinite(v) for v in sensor_xyz+endpoint_xyz)):
            self.stats["rejected_nonfinite"]+=1
            return False
        if abs(sensor_xyz[2]-endpoint_xyz[2])>self.max_vertical_delta_m:
            self.stats["rejected_vertical"]+=1
            return False
        dx=endpoint_xyz[0]-sensor_xyz[0]
        dy=endpoint_xyz[1]-sensor_xyz[1]
        length=math.hypot(dx,dy)
        if length<self.resolution or length>self.max_range_m:
            self.stats["rejected_range"]+=1
            return False
        start=self.cell(sensor_xyz)
        stop=self.cell(endpoint_xyz)
        if start is None or stop is None:
            self.stats["rejected_out_of_bounds"]+=1
            return False
        x0,y0=start; x1,y1=stop
        if start==stop:
            self.stats["rejected_range"]+=1
            return False
        # Grid bresenham visits the same sequence regardless of point density.
        count=abs(x1-x0); row=abs(y1-y0)
        sx=1 if x0<x1 else -1
        sy=1 if y0<y1 else -1
        error=count-row
        x,y=x0,y0
        while (x,y)!=(x1,y1):
            i=y*self.width+x
            if self.free_votes[i]<0xffffffff:
                self.free_votes[i]+=1
            twice=error*2
            if twice>-row:
                error-=row;x+=sx
            if twice<count:
                error+=count;y+=sy
        i=y1*self.width+x1
        if self.hit_votes[i]<0xffffffff:
            self.hit_votes[i]+=1
        self.stats["rays_accepted"]+=1
        return True

    def project(self, static_obstacle_xy=(), clearance_m=0.):
        if not (0<=clearance_m<=2. and math.isfinite(clearance_m)):
            raise ValueError("invalid clearance assumption")
        n=self.width*self.height
        # Never clear a recorded endpoint; never call unknown cells free.
        grid=[UNKNOWN]*n
        for i in range(n):
            if self.hit_votes[i]:
                grid[i]=OCCUPIED
            elif self.free_votes[i]>=self.min_free_rays:
                grid[i]=FREE
        for point in static_obstacle_xy:
            idx=self.cell(point)
            if idx is not None:
                x,y=idx
                grid[y*self.width+x]=OCCUPIED
        if clearance_m>0:
            radius=math.ceil(clearance_m/self.resolution)
            offsets=[(dx,dy) for dx in range(-radius,radius+1)
                     for dy in range(-radius,radius+1)
                     if math.hypot(dx,dy)*self.resolution<=clearance_m+1e-9]
            occupied=[(i%self.width,i//self.width)
                      for i,v in enumerate(grid) if v==OCCUPIED]
            for x,y in occupied:
                for dx,dy in offsets:
                    nx,ny=x+dx,y+dy
                    if 0<=nx<self.width and 0<=ny<self.height:
                        grid[ny*self.width+nx]=OCCUPIED
        return {
            "grid":grid,"width":self.width,"height":self.height,
            "origin":list(self.origin),"resolution":self.resolution,
            "free_cells":grid.count(FREE),
            "unknown_cells":grid.count(UNKNOWN),
            "inflated_occupied_cells":grid.count(OCCUPIED),
            "ray_visibility_evidence_only_not_ground_clearance":True,
            "ray_count_stats":dict(self.stats),
            "minimum_free_votes":self.min_free_rays,
            "near_horizontal_ray_max_dz_m":self.max_vertical_delta_m,
            "maximum_ray_range_m":self.max_range_m,
            "static_obstacle_overlay_user_selected":bool(static_obstacle_xy),
            "clearance_assumption_m":clearance_m,
            "safety_authorization":"NEVER",
            "actuator_output":"DISABLED",
        }
