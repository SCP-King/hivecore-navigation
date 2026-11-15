"""Run real recording handlers across browser namespaces without ROS or motion."""
import ast
import asyncio
import json
import math
import sys
import threading
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teach_routes import TeachingSession,RouteStore,map_identity,validate_recorded_stops
from navigation_session import NavigationSession

SOURCE=Path(__file__).resolve().parents[1]/'slam_lite_gui.py'


def measured(x=0.,yaw=0.):
    return {'source':'measured_stop','pose':{'stamp_ns':100,'frame':'map','child_frame':'base_link',
        'position':[x,0.,0.],'quaternion':[0.,0.,math.sin(yaw/2),math.cos(yaw/2)]},
        'odometry':{'linear':[0.,0.,0.],'angular':[0.,0.,0.]}}


def page(tmp_path,session,robot,store):
    root=tmp_path/'repo';(root/'config').mkdir(parents=True,exist_ok=True)
    geometry=root/'config/n01_sensor_geometry.json';geometry.write_text('{}')
    pcd=tmp_path/'map.pcd';pcd.write_bytes(b'map')
    state=S(lock=threading.Lock(),recv_t={'pose':1.})
    async def inline(fn,*args):return fn(*args)
    async def sleep(_):state.recv_t['pose']+=1
    def context():
        if robot['offline']:raise ValueError('no current localization')
        return str(pcd),(robot['x'],0.,0.,0.)
    ui={key:S(value=value)for key,value in [('teach_name','实测A-B'),('teach_return',False),
        ('teach_point_name','A'),('teach_wait',1.),('teach_saved',None)]}
    ui['teach_saved'].update=Mock()
    scope=dict(TEACH_SESSION=session,TEACH_STORE=store,TASK_STATE=NavigationSession(),STATE=state,
        REPO_ROOT=root,UI=ui,ui=S(notify=Mock()),_teach_context=context,_teach_map_path=lambda:str(pcd),
        _capture_station_state=lambda _:measured(robot['x']),map_identity=map_identity,
        validate_recorded_stops=validate_recorded_stops,_teach_view=Mock(),push_log=Mock(),
        asyncio=S(to_thread=inline,sleep=sleep),math=math)
    fn=next(n for n in ast.parse(SOURCE.read_text()).body if isinstance(n,ast.AsyncFunctionDef)and n.name=='teaching_action')
    exec(compile(ast.Module(body=[fn],type_ignores=[]),'recording_handler','exec'),scope)
    return scope


def test_real_two_stop_record_save_and_reload_survives_page_change_and_missing_live_pose(tmp_path):
    session=TeachingSession();store=RouteStore(tmp_path/'routes');robot={'x':0.,'offline':False}
    a=page(tmp_path,session,robot,store)
    asyncio.run(a['teaching_action']('begin'));asyncio.run(a['teaching_action']('record'))
    assert session.active and len(session.points)==1
    asyncio.run(a['teaching_action']('save'))
    assert not store.list() and len(session.points)==1 and session.active
    # A second browser page continues the same recorded A, rather than resetting it.
    b=page(tmp_path,session,robot,store);robot['x']=1.;b['UI']['teach_point_name'].value='B'
    asyncio.run(b['teaching_action']('record'))
    robot.update(x=20.,offline=True)
    asyncio.run(b['teaching_action']('save'))
    assert not session.active and not session.busy
    path=session.loaded_path;route=store.load(path)
    assert [p['x']for p in route['points']]==[0.,1.]
    assert [p['wait_s']for p in route['points']]==[1.,1.]
    assert [p['state']['pose']['position'][0]for p in route['points']]==[0.,1.]
    assert route['return_to_start'] is False
    b['UI']['teach_saved'].value=path;session.loaded=None
    asyncio.run(b['teaching_action']('load'))
    assert session.loaded['points'][0]['x']==0.


def test_actual_page_import_shares_draft_and_edit_ownership():
    node=next(n for n in ast.parse(SOURCE.read_text()).body if isinstance(n,ast.ImportFrom)and n.module=='teach_routes')
    namespaces=[{},{}]
    for scope in namespaces:exec(compile(ast.Module(body=[node],type_ignores=[]),'page_import','exec'),scope)
    assert namespaces[0]['TEACH_SESSION'] is namespaces[1]['TEACH_SESSION']


@pytest.mark.parametrize('defect',['missing','moving','pose_changed','yaw_changed'])
def test_planned_or_inconsistent_stations_cannot_finish_recording(defect):
    point=dict(name='A',x=0.,y=0.,z=0.,yaw=0.,wait_s=1.,state=measured())
    points=[point,json.loads(json.dumps(point))]
    if defect=='missing':points[1].pop('state')
    elif defect=='moving':points[1]['state']['odometry']['linear'][0]=.03
    elif defect=='pose_changed':points[1]['x']=1.
    else:points[1]['yaw']=.5
    with pytest.raises(ValueError):validate_recorded_stops(points)
