"""GUI rendering of the shared runtime URDF projection."""
import sys,math
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFilter
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/n01_live_navigation'))
from n01_live_navigation.urdf_projection import RobotProjection

def paint_robot(image, view, pose, polygons, envelope=None, environment=None, observed=None):
    """White URDF body; purple reviewed robot envelope; amber actual map inflation."""
    n=image.width;x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    def pixel(px,py):return ((px-view['x0'])/view['size_m']*n,(1-(py-view['y0'])/view['size_m'])*n)
    def visible_pixels(points):
        # Clip and deduplicate in NumPy before drawing, rather than iterating
        # every voxel in the entire map on every browser frame.
        points=np.asarray(points)
        pixels=(points[:,:2]-np.array([view['x0'],view['y0']]))/view['size_m']*n
        pixels=pixels[np.isfinite(pixels).all(axis=1)]
        pixels=pixels[(pixels[:,0]>=0)&(pixels[:,0]<n)&(pixels[:,1]>=0)&(pixels[:,1]<n)]
        pixels=np.unique(np.floor(pixels).astype(np.int32),axis=0)
        return pixels
    if environment is not None and len(environment):
        mask=Image.new('L',image.size);d=ImageDraw.Draw(mask)
        r=max(.5,.025/view['size_m']*n)
        for a,b in visible_pixels(environment):
            b=n-b
            d.rectangle((a-r,b-r,a+r,b+r),fill=170)
        if observed is not None:
            # Keep measured solids cyan; amber denotes the surrounding margin.
            for a,b in visible_pixels(observed):
                b=n-b
                d.rectangle((a-r,b-r,a+r,b+r),fill=0)
        image=Image.composite(Image.new('RGB',image.size,(232,160,47)),image,mask)
    if envelope and (envelope.get("vertices_m") or envelope.get("centers_x_m")):
        mask=Image.new('L',image.size);d=ImageDraw.Draw(mask)
        vertices=envelope.get('vertices_m')
        if vertices:
            world=np.asarray(vertices)@np.array([[c,s],[-s,c]])+np.array([x,y])
            d.polygon([pixel(*p)for p in world],fill=255)
        for offset in envelope.get('centers_x_m',[]):
            px,py=pixel(x+offset*c,y+offset*s);r=envelope['radius_m']/view['size_m']*n
            d.ellipse((px-r,py-r,px+r,py+r),fill=255)
        rim=Image.fromarray(np.asarray(mask)-np.asarray(mask.filter(ImageFilter.MinFilter(3))))
        glow=rim.filter(ImageFilter.GaussianBlur(5))
        image=Image.composite(Image.new('RGB',image.size,(183,101,255)),image,glow.point(lambda a:min(150,a*5)))
        image=Image.composite(Image.new('RGB',image.size,(202,150,255)),image,rim)
    d=ImageDraw.Draw(image)
    for link,poly in polygons:
        world=poly@np.array([[c,s],[-s,c]])+np.array([x,y])
        pts=[pixel(*p)for p in world]
        color=(223,235,246) if 'wheel' not in link else (74,100,121)
        d.polygon(pts,fill=color,outline=(105,167,192))
    px,py=pixel(x,y)
    d.line((px,py,px+12*c,py-12*s),fill=(255,255,255),width=3)
    return image
