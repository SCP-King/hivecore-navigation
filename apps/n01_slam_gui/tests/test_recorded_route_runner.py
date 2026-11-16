"""Real route coroutine with an in-memory typed-controller feedback contract."""
import ast,asyncio,math,time,threading,uuid,sys
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teach_routes import map_identity,waypoint,SCHEMA
from n01_live_navigation.task_status import NavigationTaskView
SOURCE=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()


def harness(tmp_path):
    root=tmp_path/'repo';(root/'config').mkdir(parents=True)
    geometry=root/'config/n01_sensor_geometry.json';geometry.write_text('{}')
    pcd=tmp_path/'map.pcd';pcd.write_bytes(b'map')
    identity=map_identity(pcd,geometry)
    route={'schema':SCHEMA,'name':'A B','map':identity,'return_to_start':False,
      'points':[waypoint('A',0.,0.,0.,0.,1.),waypoint('B',1.,0.,0.,math.pi/2,1.)]}
    state=S(lock=threading.Lock(),loc={'map_path':str(pcd)},navigation_task=NavigationTaskView(),pose=(5.,3.,0.),
      plan_goal_ns=0,goal_seq=0)
    task=S(pause=False,cancel=False,feedback={},binding=None)
    clock=[100.];goals=[];spawns=[];events=[];live=Mock(return_value=(str(pcd),state.pose))
    active=[False];enabled=[False];task_id=[''];delayed=[0]
    def arrived():
        task.feedback=dict(task_id=task_id[0],goal_id=state.plan_goal_ns,state='ARRIVED',received_at=clock[0])
    def request(x,y,yaw,frame):
        goals.append((x,y,yaw));state.plan_goal_ns+=1;state.goal_seq+=1
        if active[0]:arrived()
    def start(speed,cbf,id,target,disabled,route_mode):
        spawns.append((id,target,disabled,route_mode));task_id[0]=id;active[0]=True
        task.feedback=dict(task_id=id,goal_id=state.plan_goal_ns,state='DISABLED',start_ready=True,received_at=clock[0]);return True
    def enable(value):enabled[0]=value;events.append(('enable',value));arrived()
    async def sleep(delay):
        clock[0]+=delay;events.append(('wait',delay))
        if enabled[0] and delayed[0]:
            delayed[0]-=1
            if not delayed[0]:arrived()
        task.feedback['received_at']=clock[0]
    async def inline(fn,*args,**kw):return fn(*args,**kw)
    async def spawn(method,*args):return method(*args)
    class Paused(RuntimeError):pass
    scope=dict(asyncio=S(sleep=sleep,to_thread=inline,CancelledError=asyncio.CancelledError,current_task=asyncio.current_task),time=S(monotonic=lambda:clock[0]),uuid=uuid,
      STATE=state,TASK_STATE=task,REPO_ROOT=root,Path=Path,map_identity=map_identity,
      PROCS=S(running=lambda key:active[0],start_navigation_execution=start),
      WORKER=S(request_goal=request,set_route_enabled=enable),RoutePaused=Paused,RouteLocalizationWait=type("LocWait",(RuntimeError,),{}),
      _route_live=live,_teach_view=Mock(),push_log=Mock(),MotionFeedback=Mock(),
      _start_owned_motion=spawn)
    tree=ast.parse(SOURCE)
    names={'_run_route_mission','_execute_recorded_route','_wait_route_motion','_start_route_motion_when_ready','_check_route_binding'}
    selected=[n for n in tree.body if isinstance(n,(ast.AsyncFunctionDef,ast.FunctionDef))and n.name in names]
    exec(compile(ast.Module(body=selected,type_ignores=[]),'route_coroutines','exec'),scope)
    context=dict(index=0,remaining_wait=None)
    return S(scope=scope,state=state,task=task,route=route,context=context,goals=goals,spawns=spawns,
      clock=clock,events=events,live=live,delayed=delayed,arrived=arrived,pcd=pcd,Paused=Paused)


def test_original_a_then_b_one_controller_and_station_dwell(tmp_path):
    h=harness(tmp_path)
    assert asyncio.run(h.scope['_execute_recorded_route'](h.route,h.context,.03))==2
    assert h.goals==[(0.,0.,0.),(1.,0.,math.pi/2)]
    assert len(h.spawns)==1 and h.spawns[0][1:]==(None,True,True)
    assert h.context['index']==2 and h.clock[0]>=102.
    h.live.assert_called_once() # GUI never gates the moving loop on its pose cache.
    assert h.state.pose==(5.,3.,0.) # Never rebase the saved A to current position.


def test_pause_during_dwell_preserves_station_and_remaining_time(tmp_path):
    h=harness(tmp_path);original=h.scope['asyncio'].sleep
    async def pause(delay):
        await original(delay)
        if h.scope['TASK_STATE'].feedback.get('state')=='ARRIVED':h.task.pause=True
    h.scope['asyncio'].sleep=pause
    with pytest.raises(h.Paused):asyncio.run(h.scope['_execute_recorded_route'](h.route,h.context,.03))
    assert h.context['index']==0 and h.context['remaining_wait']is not None
    assert len(h.spawns)==1


def test_old_arrived_feedback_cannot_advance_next_goal(tmp_path):
    h=harness(tmp_path);original=h.scope['WORKER'].request_goal
    def request(*args):
        original(*args)
        if len(h.goals)==2:
            h.task.feedback['goal_id']=h.state.plan_goal_ns-1;h.delayed[0]=4
    h.scope['WORKER'].request_goal=request
    asyncio.run(h.scope['_execute_recorded_route'](h.route,h.context,.03))
    assert h.delayed[0]==0 and h.context['index']==2


def test_map_changed_during_route_cannot_start_next_station(tmp_path):
    h=harness(tmp_path);original=h.scope['asyncio'].sleep
    async def change(delay):await original(delay);h.pcd.write_bytes(b'changed')
    h.scope['asyncio'].sleep=change
    with pytest.raises((ValueError,RuntimeError),match='地图'):asyncio.run(h.scope['_execute_recorded_route'](h.route,h.context,.03))
    assert len(h.goals)==1


def test_task_cancel_is_not_arrival(tmp_path):
    h=harness(tmp_path);h.task.cancel=True
    with pytest.raises(RuntimeError,match='取消'):asyncio.run(h.scope['_execute_recorded_route'](h.route,h.context,.03))
    assert h.context['index']==0


def test_map_switch_during_travel_stops_without_using_gui_pose_age(tmp_path):
    h=harness(tmp_path);h.task.binding=(str(h.pcd),[])
    h.state.loc['map_path']=str(tmp_path/'other.pcd')
    with pytest.raises(RuntimeError,match='地图已切换'):h.scope['_check_route_binding']()


def test_realtime_route_rejects_new_slam_coordinate_session(tmp_path):
    h=harness(tmp_path);h.task.binding=('session://'+'a'*32,[])
    h.scope['PROCS'].coordinate_session_id='a'*32
    h.scope['_check_route_binding']()
    h.scope['PROCS'].coordinate_session_id='b'*32
    with pytest.raises(RuntimeError,match='坐标会话'):h.scope['_check_route_binding']()


def test_cancelled_mission_reports_cancelled_instead_of_failed(tmp_path):
    h=harness(tmp_path)
    async def noop(*args):pass
    async def cancelled(*args):raise asyncio.CancelledError()
    h.task.active=True;h.task.context=h.context;h.task.task=None;h.task.cancel=True
    h.state.motion_feedback=Mock()
    h.scope.update(_await_route_stable=noop,_execute_recorded_route=cancelled,
        _stop_owned_motion=noop,publish_motion_events=Mock())
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(h.scope['_run_route_mission'](h.route,h.context,.03))
    assert h.state.navigation_task.phase=='CANCELLED'
    assert not h.task.active and h.task.context is None
