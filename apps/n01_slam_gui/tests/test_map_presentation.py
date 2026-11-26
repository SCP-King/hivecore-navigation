import ast,sys,threading,time,os
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
import math
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
    assert tuple(rgb[20,20])==(190,150,255)
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
        _GRID_BASE_PTS={'pts':None,'window':None},
        push_log=lambda *a,**k:None)
    exec(compile(ast.Module(body=[f],type_ignores=[]),'source_selection','exec'),scope)
    p,label,seq,dyn,pose,z,trail,grids,style=scope['_grid_source']()
    assert grids is None and p is None, 'low-cost mode waits for grids instead of point clouds'
    assert style is not None and style[2] is True, 'low-cost raster is the default'
    assert label.startswith('导航') and pose==(1,2,.1) and z==.3


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
               _GRID_UNKNOWN=(12,20,31),_GRID_FREE=(26,40,62),_GRID_BASE=(70,90,115),
               _GRID_REF=(190,150,255),_GRID_LIVE=(79,239,219),
               _GRID_REACHABLE=(246,246,242),_GRID_UNASSESSED=(76,86,105),_GRID_BASE_PTS={'pts':None})
    exec(compile(ast.Module(body=[f],type_ignores=[]),'projection','exec'),scope)
    data=np.zeros((20,20),np.int8)          # 0 = free
    data[2:6,2:6]=100                       # reference occupied block
    live=np.full((20,20),-1,np.int8)   # live layer only reports its own block
    live[10:14,10:14]=100
    ref=(data,0.,0.,.05,0.); live_layer=(live,0.,0.,.05,0.)
    rgb=scope['_project_occupancy'](ref,live_layer,{'x0':0.,'y0':0.,'size_m':2.0},600)
    colors=set(map(tuple,rgb.reshape(-1,3)))
    assert (190,150,255) in colors, 'reference layer must be drawn'
    assert (79,239,219) in colors, 'live layer must be drawn'
    assert (76,86,105) in colors, 'observed free without search coverage must be marked unassessed'
    assert (12,20,31) in colors, 'never-observed stays darkest'


def test_search_inflation_is_visible_and_live_obstacles_keep_priority():
    source=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()
    f=next(n for n in ast.parse(source).body
           if isinstance(n,ast.FunctionDef) and n.name=='_project_occupancy')
    scope=dict(np=np,GRID_RES=.06,GRID_MAX_N=600,
               _GRID_UNKNOWN=(12,20,31),_GRID_FREE=(26,40,62),_GRID_BASE=(70,90,115),
               _GRID_REF=(190,150,255),_GRID_LIVE=(79,239,219),
               _GRID_REACHABLE=(246,246,242),_GRID_UNASSESSED=(76,86,105),_GRID_BASE_PTS={'pts':None})
    exec(compile(ast.Module(body=[f],type_ignores=[]),'projection','exec'),scope)
    ref=np.full((20,20),-1,np.int8); ref[2:8,2:8]=100
    live=np.full((20,20),-1,np.int8); live[2:8,2:8]=100
    live[2:4,2:4]=0; live[10:14,10:14]=100
    live[0:2,0:2]=0
    search=np.zeros_like(ref); search[15:18,15:18]=100
    rgb=scope['_project_occupancy']((ref,0.,0.,.05,0.),(live,0.,0.,.05,0.),
                                    {'x0':0.,'y0':0.,'size_m':1.0},600,
                                    search_layer=(search,0.,0.,.05,0.))
    colors=set(map(tuple,rgb.reshape(-1,3)))
    assert (142,102,54) not in colors, 'search-occupied must not flood the diagnostic map'
    assert (246,246,242) in colors, 'known free and planner-connected must be white'
    assert (79,239,219) in colors, 'live-only must stay distinct'


def test_low_cost_navigation_never_falls_back_to_point_cloud():
    source=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()
    f=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='_grid_source')
    state=S(lock=threading.Lock(),age_of=lambda k:.1,loc_pts=None,cloud_pts=None,nav_static_pts=None,
        nav_fused_pts=None,nav_dyn_pts=None,live_pts=None,loc_seq=0,cloud_seq=0,nav_map_seq=0,live_seq=0,
        pose=(0,0,0),control_pose=(0,0,0),pose_z=.2,body_pose_xyz=None,trail=[],
        loc={},mapping={})
    scope=dict(UI={'src':S(value='导航'),'render':S(value='低耗栅格')},
        STATE=state,PROCS=S(running=lambda _:True,desk_mode='mapping'),
        choose_map=choose_map,scan_freshness=scan_freshness,np=np,time=time,os=os,_GRID_DIAG={},
        _GRID_BASE_PTS={'pts':None,'window':None},push_log=lambda *a,**k:None)
    exec(compile(ast.Module(body=[f],type_ignores=[]),'source_selection','exec'),scope)
    points,label,seq,dyn,pose,z,trail,grids,style=scope['_grid_source']()
    assert points is None and grids is None, 'low-cost navigation must not render point clouds'
    assert '等待局部栅格' in label
    scope['UI']['src'].value = '自动'
    state.loc['session'] = 'tracking'
    state.node_rows = [{'name': '/scan_planner_node', 'count': 1}]
    assert scope['_grid_source']()[1].startswith('导航')
    state.node_rows = []
    scope['PROCS'].running = lambda _: False
    state.age_of = lambda key: .1 if key == 'loc_map' else None
    scope['_points_to_grid_layer'] = lambda points: None
    assert scope['_grid_source']()[1].startswith('定位')



def test_points_to_grid_layer_bins_points_into_cells():
    import math as _math
    source=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()
    f=next(n for n in ast.parse(source).body
           if isinstance(n,ast.FunctionDef) and n.name=='_points_to_grid_layer')
    scope=dict(np=np,math=_math,time=time)
    exec(compile(ast.Module(body=[f],type_ignores=[]),'binning','exec'),scope)
    pts=np.array([[0.,0.,0.],[0.01,0.01,0.],[1.,1.,0.]],float)
    data,ox,oy,res,_stamp=scope['_points_to_grid_layer'](pts,res=.1)
    assert res>0 and data.ndim==2
    assert int((data==100).sum())>=1
    assert (data==-1).any()


def test_goal_footprint_status_marks_free_blocked_unknown():
    source=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()
    f=next(n for n in ast.parse(source).body
           if isinstance(n,ast.FunctionDef) and n.name=='_goal_footprint_status')
    scope=dict(np=np,math=math)
    exec(compile(ast.Module(body=[f],type_ignores=[]),'goal_fp','exec'),scope)
    check=scope['_goal_footprint_status']
    env={'vertices_m':[[-.2,-.1],[.2,-.1],[.2,.1],[-.2,.1]]}
    free=np.zeros((40,40),np.int8)
    layer=(free,0.,0.,.05,0.)
    assert check((0.5,0.5),0.,env,[layer])=='free'
    blocked=free.copy();blocked[10,10]=100
    assert check((0.5,0.5),0.,env,[(blocked,0.,0.,.05,0.)])=='blocked'
    unknown=np.full((40,40),-1,np.int8)
    assert check((0.5,0.5),0.,env,[(unknown,0.,0.,.05,0.)])=='unknown'
    live_hit=free.copy();live_hit[11,11]=100
    assert check((0.5,0.5),0.,env,[layer,(live_hit,0.,0.,.05,0.)])=='blocked'


def test_envelope_conflict_cells_marks_layers_inside_footprint():
    source=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()
    f=next(n for n in ast.parse(source).body
           if isinstance(n,ast.FunctionDef) and n.name=='_envelope_conflict_cells')
    scope=dict(np=np,math=math)
    exec(compile(ast.Module(body=[f],type_ignores=[]),'conf','exec'),scope)
    check=scope['_envelope_conflict_cells']
    env={'vertices_m':[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]]}
    ref=np.zeros((40,40),np.int8);ref[10,10]=100
    live=np.zeros((40,40),np.int8);live[9,9]=100
    cells=check((0.,0.),0.,env,[(ref,0.,0.,.05,0.),(live,0.,0.,.05,0.)])
    assert any(c[2]=='static' for c in cells)
    assert any(c[2]=='live' for c in cells)
    free=np.zeros((40,40),np.int8)
    assert check((0.,0.),0.,env,[(free,0.,0.,.05,0.)])==[]
