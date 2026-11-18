import ast,sys,threading,time,os
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_presentation import choose_map,occupancy_colors,scan_freshness


def test_automatic_priority_tracks_navigation_then_mapping_then_localization():
    assert choose_map('自动',True,True,True)=='导航栅格'
    assert choose_map('自动',False,True,True)=='建图'
    assert choose_map('自动',False,False,True)=='定位地图'
    assert choose_map('自动',False,False,False)=='等待'
    assert choose_map('建图',True,False,True)=='建图'


def test_solid_render_preserves_unknown_and_highlights_current_observations():
    occ=np.zeros((80,80),bool);occ[20:22,20:35]=True
    live=np.zeros_like(occ);live[40:45,40:45]=True
    rgb=occupancy_colors(occ,live,.06)
    assert tuple(rgb[0,0])==(12,20,31)
    assert tuple(rgb[41,41])==(79,239,219)
    assert tuple(rgb[20,20])==(116,151,182)
    assert tuple(rgb[39,41])!=(12,20,31)  # Local visual halo.
    assert tuple(rgb[30,30])==(12,20,31)  # No fictitious free bounding box.


def test_real_source_function_uses_control_pose_and_does_not_reenter_state_lock():
    source=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()
    f=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='_grid_source')
    lock=threading.Lock()
    def age(k):
        assert lock.acquire(blocking=False), 'state lock reentered'
        lock.release()
        return .1
    points=np.array([[1.,2.,.2]])
    state=S(lock=lock,age_of=age,loc_pts=points,cloud_pts=points,nav_static_pts=np.array([[99.,99.,.2]]),nav_fused_pts=points,
        nav_dyn_pts=points,live_pts=points,loc_seq=1,cloud_seq=1,nav_map_seq=1,live_seq=1,
        pose=(9,9,0),control_pose=(1,2,.1),pose_z=.2,body_pose_xyz=(1,2,.3),trail=[],
        loc={'session':'tracking'},mapping={'session':'mapping'},nav_scan_stamp_ns=int(time.time()*1e9))
    scope=dict(UI={'src':S(value='自动')},STATE=state,PROCS=S(running=lambda _:True,desk_mode='mapping'),
        choose_map=choose_map,scan_freshness=scan_freshness,np=np,time=time,os=os,_GRID_DIAG={},
        push_log=lambda *a,**k:None)
    exec(compile(ast.Module(body=[f],type_ignores=[]),'source_selection','exec'),scope)
    p,label,seq,dyn,pose,z,trail,grids,style=scope['_grid_source']()
    assert grids is None, 'no occupancy grid in this scenario'
    assert style is not None and style[2] is True, 'low-cost raster is the default'
    assert label.startswith('导航') and pose==(1,2,.1) and z==.3
    assert np.all(p[:,0]<99.), 'original historical cloud must not reappear over fused navigation view'


def test_republished_old_scan_cannot_select_navigation_over_live_slam():
    status=scan_freshness(100_000_000_000,175.)
    assert status['scan_age_s']==75. and not status['scan_ready']
    assert choose_map('自动',status['scan_ready'],True,False)=='建图'
    assert scan_freshness(0,175.)['scan_reason']=='occupancy_missing'
    assert scan_freshness(174_000_000_000,175.)['scan_ready']


def test_occupancy_grid_projection_colors_layers_without_scipy():
    source=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()
    f=next(n for n in ast.parse(source).body
           if isinstance(n,ast.FunctionDef) and n.name=='_project_occupancy')
    scope=dict(np=np,GRID_RES=.06,GRID_MAX_N=600,
               _GRID_UNKNOWN=(12,20,31),_GRID_FREE=(26,40,62),
               _GRID_REF=(116,151,182),_GRID_LIVE=(79,239,219))
    exec(compile(ast.Module(body=[f],type_ignores=[]),'projection','exec'),scope)
    data=np.zeros((20,20),np.int8)          # 0 = free
    data[2:6,2:6]=100                       # reference occupied block
    live=np.full((20,20),-1,np.int8)   # live layer only reports its own block
    live[10:14,10:14]=100
    ref=(data,0.,0.,.05,0.); live_layer=(live,0.,0.,.05,0.)
    rgb=scope['_project_occupancy'](ref,live_layer,{'x0':0.,'y0':0.,'size_m':2.0},600)
    colors=set(map(tuple,rgb.reshape(-1,3)))
    assert (116,151,182) in colors, 'reference layer must be drawn'
    assert (79,239,219) in colors, 'live layer must be drawn'
    assert (26,40,62) in colors, 'observed free must be distinguishable'
    assert (12,20,31) in colors, 'never-observed stays darkest'
