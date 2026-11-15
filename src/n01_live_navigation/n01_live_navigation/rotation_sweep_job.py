"""Bounded-age swept-space certificates; worker never commands hardware."""
import math,time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
import numpy as np
from .heading_search import wrap
from .rotation_guard import rotation_arc_clear


def calculate(snapshot,decode):
    began=time.monotonic()
    raw,stamp=decode(snapshot,with_stamp=True)
    args=(snapshot.position,snapshot.plane,snapshot.start_yaw,snapshot.target_yaw)
    static_ok=rotation_arc_clear(snapshot.static,*args,min_observations=0,
                                footprint=snapshot.footprint,position_margin=.02)
    raw_ok=rotation_arc_clear(raw,*args,footprint=snapshot.footprint,position_margin=.02)
    return dict(snapshot=snapshot,clear=static_ok and raw_ok,raw_stamp=stamp,finished=time.monotonic(),elapsed_ms=(time.monotonic()-began)*1000.)


def applicable(result,now,stamp_ns,position,yaw,target,plane,footprint):
    if not result or now-result['finished']>.3 or not -.1<=(stamp_ns-result['raw_stamp'])/1e9<=.6:return False
    s=result['snapshot']
    if now-s.static_rx>1. or math.dist(position,s.position)>.01:return False
    if abs(wrap(target-s.target_yaw))>1e-5:return False
    if not np.allclose(plane,s.plane,atol=1e-6,rtol=0):return False
    from .robot_footprint import polygon_hits
    if not np.all(polygon_hits(np.asarray(footprint),s.footprint,.005)):return False
    arc=wrap(target-s.start_yaw);progress=wrap(yaw-s.start_yaw)
    # Tiny yaw noise may cross the original arc's start. Admit it only when
    # covered by the same 2cm Minkowski margin already collision-checked.
    extra=max(0.,min(0.,arc)-progress,progress-max(0.,arc))
    rho=float(np.linalg.norm(s.footprint,axis=1).max())
    return math.dist(position[:2],s.position[:2])+.005+rho*extra<=.02


def poll(node,now,decode):
    position=(*node.pose[:2],node.pose_z);yaw=node.pose[2];target=node.target_yaw
    footprint=node.rotation_footprint.get();plane=node.rotation_plane
    future=getattr(node,'rotation_sweep_future',None)
    if future is not None and future.done():
        node.rotation_sweep_future=None
        try:
            node.rotation_sweep_result=future.result();node.rotation_sweep_error=''
        except ValueError as exc:
            node.rotation_sweep_result=None;node.rotation_sweep_error=str(exc)
    result=getattr(node,'rotation_sweep_result',None)
    valid=applicable(result,now,node.get_clock().now().nanoseconds,position,yaw,target,plane,footprint)
    if getattr(node,'rotation_sweep_future',None) is None:
        if not hasattr(node,'rotation_sweep_pool'):
            node.rotation_sweep_pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='n01_rotation_sweep')
        snapshot=SimpleNamespace(position=position,start_yaw=yaw,target_yaw=target,
            plane=np.array(plane,copy=True),footprint=np.array(footprint,copy=True),
            static=node.rotation_static,static_rx=node.rotation_static_rx,
            rotation_raw=tuple(node.rotation_raw),rotation_pose_cache=tuple(node.rotation_pose_cache),
            readiness=getattr(node,'readiness',None),rotation_tf=node.rotation_tf,get_clock=node.get_clock)
        node.rotation_sweep_future=node.rotation_sweep_pool.submit(calculate,snapshot,decode)
    return result['clear'] if valid else None
