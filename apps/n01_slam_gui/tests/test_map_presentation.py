import ast,sys,threading
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_presentation import choose_map,occupancy_colors


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
    state=S(lock=lock,age_of=age,loc_pts=points,cloud_pts=points,nav_static_pts=points,
        nav_dyn_pts=points,live_pts=points,loc_seq=1,cloud_seq=1,nav_map_seq=1,live_seq=1,
        pose=(9,9,0),control_pose=(1,2,.1),pose_z=.2,body_pose_xyz=(1,2,.3),trail=[],
        loc={'session':'tracking'},mapping={'session':'mapping'})
    scope=dict(UI={'src':S(value='自动')},STATE=state,PROCS=S(running=lambda _:True,desk_mode='mapping'),
        choose_map=choose_map,np=np)
    exec(compile(ast.Module(body=[f],type_ignores=[]),'source_selection','exec'),scope)
    p,label,seq,dyn,pose,z,trail=scope['_grid_source']()
    assert label.startswith('导航') and pose==(1,2,.1) and z==.3
