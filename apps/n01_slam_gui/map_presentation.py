"""Presentation only: source selection and solid measured obstacle rendering."""
import math
import numpy as np
from scipy.ndimage import binary_closing, distance_transform_edt


def scan_freshness(stamp_ns, now_s, limit_s=3.0):
    """Source scan age, never the receive time of a republished map heartbeat."""
    if not stamp_ns:
        return dict(scan_ready=False, scan_age_s=None, scan_reason='occupancy_missing')
    age=float(now_s)-int(stamp_ns)/1e9
    ready=math.isfinite(age) and -.1<=age<=limit_s
    return dict(scan_ready=ready, scan_age_s=age,
                scan_reason='ready' if ready else 'occupancy_stale')


def choose_map(requested, navigation_live, mapping_live, localization_live):
    if requested!='自动':return requested
    if navigation_live:return '导航栅格'
    if mapping_live:return '建图'
    if localization_live:return '定位地图'
    return '等待'


def _bbox(mask, pad=0):
    """Tight bounding box of a boolean mask; None when empty."""
    ys,xs=np.nonzero(mask)
    if len(ys)==0:return None
    h,w=mask.shape
    return (max(0,int(ys.min())-pad),min(h,int(ys.max())+pad+1),
            max(0,int(xs.min())-pad),min(w,int(xs.max())+pad+1))


def occupancy_colors(occupied, live, resolution):
    # Close single-cell sampling gaps only. Do not fill unobserved interiors or
    # treat the map's bounding rectangle as observed free space.
    # Morphology and the distance transform run on the occupied/live bounding
    # box instead of the whole view: on a 480px grid the full-frame version
    # costs ~45 ms per redraw, most of it over empty background.
    occupied=np.asarray(occupied,bool);live=np.asarray(live,bool)
    rgb=np.full((*occupied.shape,3),(12,20,31),dtype=np.uint8)
    box=_bbox(occupied,1)
    if box:
        y0,y1,x0,x1=box
        sub=occupied[y0:y1,x0:x1]
        reference=binary_closing(sub,iterations=1)|sub
        view=rgb[y0:y1,x0:x1]
        view[reference]=(190,150,255)
    if live.any():
        pad=int(math.ceil(.22/max(float(resolution),1e-6)))
        y0,y1,x0,x1=_bbox(live,pad)
        sub=live[y0:y1,x0:x1]
        current=binary_closing(sub,iterations=1)|sub
        distance=distance_transform_edt(~current)*resolution
        glow=np.exp(-distance/.07)*(distance<.22)
        view=rgb[y0:y1,x0:x1]
        blended=view.astype(float)+glow[...,None]*np.array([0,187,171.])*.5
        view[:]=np.clip(blended,0,255).astype(np.uint8)
        view[current]=(79,239,219)
    return rgb
