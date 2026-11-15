"""Bounded display-only occupied-voxel surfaces. Never fills unobserved volume."""
import numpy as np

NORMALS = np.array([(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)])
FACES = np.array([
    [(1,0,0),(1,1,0),(1,1,1),(1,0,1)],
    [(0,0,0),(0,0,1),(0,1,1),(0,1,0)],
    [(0,1,0),(0,1,1),(1,1,1),(1,1,0)],
    [(0,0,0),(1,0,0),(1,0,1),(0,0,1)],
    [(0,0,1),(1,0,1),(1,1,1),(0,1,1)],
    [(0,0,0),(0,1,0),(1,1,0),(1,0,0)],
], dtype=float)


def visible_geometry(points, ground, center=(0.,0.), radius=12., resolution=.12,
                     max_faces=24000, max_resolution=.30, max_height=2.2):
    points = np.asarray(points, dtype=float).reshape((-1,3))
    a,b,c = ground
    valid = np.isfinite(points).all(axis=1)
    heights = points[:,2] - (a*points[:,0]+b*points[:,1]+c)
    valid &= (heights>.03) & (heights<=max_height)
    valid &= ((points[:,:2]-np.asarray(center))**2).sum(axis=1)<=radius**2
    points = points[valid]
    if not (.04<=resolution<=max_resolution<=.4 and max_faces>0):
        raise ValueError('invalid_display_geometry_bounds')
    while True:
        raw = np.floor(points/resolution).astype(np.int64)
        # A padded mixed-radix key permits vectorized neighbor lookup without
        # Python per-voxel set operations or sorting structured xyz records.
        minimum = raw.min(axis=0)-1 if len(raw) else np.zeros(3,dtype=np.int64)
        extent = raw.max(axis=0)-minimum+2 if len(raw) else np.ones(3,dtype=np.int64)
        strides = np.array([extent[1]*extent[2],extent[2],1],dtype=np.int64)
        keys,indices = np.unique((raw-minimum)@strides,return_index=True)
        voxels = raw[indices]
        faces = []
        count = 0
        for direction, corners in zip(NORMALS,FACES):
            neighbors = keys+direction@strides
            positions = np.searchsorted(keys,neighbors)
            exposed = positions>=len(keys)
            within = ~exposed
            exposed[within] = keys[positions[within]]!=neighbors[within]
            boundary = voxels[exposed]
            count += len(boundary)
            faces.append((boundary[:,None,:]+corners[None,:,:])*resolution)
        if count<=max_faces:
            break
        if resolution>=max_resolution:
            raise ValueError('display_surface_budget_exceeded')
        resolution = min(max_resolution,resolution*1.35)
    quads = np.concatenate(faces,axis=0)
    triangles = quads[:,[0,1,2,0,2,3],:].reshape((-1,3))
    # Ground projection of observed obstacle columns, not a claimed solid wall
    # extending from ground to desk height. Interior edges are omitted.
    cells = np.unique(voxels[:,:2],axis=0)
    keys = {tuple(cell) for cell in cells}
    outlines = []
    for dx,dy,start,end in [(1,0,(1,0),(1,1)),(-1,0,(0,1),(0,0)),
                           (0,1,(1,1),(0,1)),(0,-1,(0,0),(1,0))]:
        edge_cells = np.asarray([cell for cell in cells if (cell[0]+dx,cell[1]+dy) not in keys],dtype=float).reshape((-1,2))
        xy = (edge_cells[:,None,:]+np.array([start,end]))*resolution
        xy = xy.reshape((-1,2))
        outlines.append(np.column_stack((xy,a*xy[:,0]+b*xy[:,1]+c+.015)))
    return dict(triangles=triangles, outlines=np.concatenate(outlines,axis=0),
                resolution=resolution, voxels=len(voxels), faces=count, points=len(points))


def ground_grid(ground, center=(0.,0.), radius=12., spacing=1.):
    a,b,c = ground
    x,y = np.floor(np.asarray(center)/spacing)*spacing
    lines=[]
    for offset in np.arange(-radius,radius+spacing/2,spacing):
        lines.extend([(x+offset,y-radius),(x+offset,y+radius),
                      (x-radius,y+offset),(x+radius,y+offset)])
    xy=np.asarray(lines)
    return np.column_stack((xy,a*xy[:,0]+b*xy[:,1]+c+.005))


def contour_geometry(points, ground, center=(0.,0.), radius=12., resolution=.06,
                     max_patches=4000, max_height=2.2):
    """Small oriented patches on measured surfaces, never extruded voxel cubes.

    Patch centers are actual returns; local PCA estimates a surface tangent.
    No edges connect separate patches or span gaps in unobserved geometry.
    """
    from scipy.spatial import cKDTree
    if not .02<=resolution<=.3 or max_patches<1:
        raise ValueError('invalid_contour_bounds')
    xyz=np.asarray(points,dtype=float).reshape((-1,3))
    a,b,c=ground
    h=xyz[:,2]-(a*xyz[:,0]+b*xyz[:,1]+c)
    keep=np.isfinite(xyz).all(axis=1)&(h>.03)&(h<=max_height)
    keep&=np.sum((xyz[:,:2]-center)**2,axis=1)<=radius**2
    xyz=xyz[keep];step=resolution
    while len(xyz):
        _,idx=np.unique(np.floor(xyz/step).astype(np.int64),axis=0,return_index=True)
        if len(idx)<=max_patches:break
        step*=1.25
    centers=xyz[idx] if len(xyz) else np.empty((0,3))
    if not len(centers):
        return dict(triangles=np.empty((0,3)),outlines=np.empty((0,3)),
                    resolution=step,voxels=0,faces=0,points=0)
    tree=cKDTree(xyz)
    distances,neighbors=tree.query(centers,k=min(12,len(xyz)))
    if distances.ndim==1:distances=distances[:,None];neighbors=neighbors[:,None]
    local=xyz[neighbors]-centers[:,None,:]
    weights=(distances<=min(.22,step*2.5)).astype(float)
    mean=np.sum(local*weights[:,:,None],axis=1)/np.maximum(weights.sum(axis=1),1)[:,None]
    delta=local-mean[:,None,:]
    cov=np.einsum('nki,nkj,nk->nij',delta,delta,weights)
    _,basis=np.linalg.eigh(cov)
    u,v=basis[:,:,2],basis[:,:,1]
    # Keep patch size bounded even when the budget makes the sampling coarse.
    patch_radius=min(.065,resolution*.7)
    angles=np.arange(6)*np.pi/3
    ring=centers[:,None,:]+patch_radius*(np.cos(angles)[None,:,None]*u[:,None,:]
                                      +np.sin(angles)[None,:,None]*v[:,None,:])
    triangles=np.stack((np.repeat(centers[:,None,:],6,axis=1),ring,np.roll(ring,-1,axis=1)),axis=2).reshape((-1,3))
    return dict(triangles=triangles,outlines=np.empty((0,3)),resolution=step,
                voxels=len(centers),faces=len(centers)*6,points=len(xyz))
