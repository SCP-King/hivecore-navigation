"""Presentation only: source selection and solid measured obstacle rendering."""
import numpy as np
from scipy.ndimage import binary_closing, distance_transform_edt


def choose_map(requested, navigation_live, mapping_live, localization_live):
    if requested!='自动':return requested
    if navigation_live:return '导航栅格'
    if mapping_live:return '建图'
    if localization_live:return '定位地图'
    return '等待'


def occupancy_colors(occupied, live, resolution):
    # Close single-cell sampling gaps only. Do not fill unobserved interiors or
    # treat the map's bounding rectangle as observed free space.
    occupied=np.asarray(occupied,bool);live=np.asarray(live,bool)
    reference=binary_closing(occupied,iterations=1)|occupied
    current=binary_closing(live,iterations=1)|live
    rgb=np.full((*occupied.shape,3),(12,20,31),dtype=np.uint8)
    rgb[reference]=(116,151,182)
    if current.any():
        distance=distance_transform_edt(~current)*resolution
        glow=np.exp(-distance/.07)*(distance<.22)
        color=np.array([0,187,171.])
        rgb=np.clip(rgb.astype(float)+glow[...,None]*color*.5,0,255).astype(np.uint8)
        rgb[current]=(79,239,219)
    return rgb
