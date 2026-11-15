"""Exercise the real GUI route coroutine with non-actuating fake processes."""
import ast,asyncio,json,sys,time,uuid,math
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import threading
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from navigation_session import NavigationSession
from teach_routes import map_identity,waypoint,SCHEMA
from localization_recovery import RecoveryGate
SOURCE=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()


def harness(tmp_path,outcome='ARRIVED'):
 root=tmp_path/'repo';(root/'config').mkdir(parents=True);geo=root/'config/n01_sensor_geometry.json';geo.write_text('{}')
 pcd=tmp_path/'map.pcd';pcd.write_bytes(b'map');identity=map_identity(pcd,geo)
 route={'schema':SCHEMA,'name':'route','map':identity,'points':[waypoint(str(i),i,0,0,0)for i in (1,2,3)],'return_to_start':False}
 state=S(lock=threading.Lock(),loc={'session':'tracking','reloc_ok':True,'map_path':str(pcd)},pose=(10,0,0),
         nav_speed_mps=.05,age_of=lambda k:.1,goal_seq=0,plan_goal_ns=0,plan_success_ns=0,plan_active=False,goal_failed=False,
         motion_feedback=S(terminal=False),pose_stamp_age_at_rx=.2,pose_revision=1)
 goals=[];started=[];active=[False]
 clock=[100.]
 async def tick(delay):
  clock[0]+=delay;state.pose_revision+=1
  if active[0] and scope['TASK_STATE'].feedback:
   scope['TASK_STATE'].feedback['received_at']=clock[0]
 async def confirm(*args):return True
 async def inline(fn,*args,**kwargs):return fn(*args,**kwargs)
 def request(x,y,yaw,frame):
  goals.append((x,y));state.goal_seq+=1;state.plan_goal_ns=time.time_ns();state.plan_success_ns=time.time_ns();state.plan_active=True
 def start(speed,cbf,task_id,target,start_disabled=False):
  started.append(target);active[0]=True;state.pose=(*target,state.pose[2])
  scope['TASK_STATE'].feedback={'task_id':task_id,'state':'DISABLED' if start_disabled else outcome,'received_at':clock[0]}
  return True
 rotations=[]
 def rotate(task_id,yaw):
  rotations.append(yaw);active[0]=True;state.pose=(*state.pose[:2],yaw)
  scope['TASK_STATE'].feedback={'task_id':task_id,'state':'DISABLED','received_at':clock[0]}
  return True
 def stop(*args):active[0]=False
 def hold():
  scope['TASK_STATE'].feedback.update(state='DISABLED',received_at=clock[0])
 def enable(value):
  scope['TASK_STATE'].feedback.update(state=scope['_outcome'] if value else 'DISABLED',received_at=clock[0])
 scope={'TASK_STATE':NavigationSession(), 'ROUTE_ACTIVE':False,'TEACH_SESSION':S(active=False,busy=False,loaded=route),'ROUTE_CANCEL':False,
        '_rotations':rotations,'ROUTE_CONTEXT':None,'ROUTE_PAUSE':False,'ROUTE_FEEDBACK':{},'STATE':state,'UI':{'slow_speed':S(value=.05),'slow_cbf':S(value=False)},
        'PROCS':S(running=lambda *args:active[0],start_slow_motion=start,start_rotation=rotate,stop=stop),
        'WORKER':S(request_goal=request,hold_route_motion=hold,set_route_enabled=enable),'asyncio':S(to_thread=inline,sleep=tick,CancelledError=asyncio.CancelledError,create_task=asyncio.create_task,shield=asyncio.shield,current_task=asyncio.current_task),'json':json,'time':S(monotonic=lambda:clock[0],time_ns=time.time_ns),'uuid':uuid,'math':__import__('math'),
        'map_identity':map_identity,'Path':Path,'REPO_ROOT':root,'_confirm':confirm,'_teach_view':Mock(),'push_log':Mock(),
        '_tick':tick,'_clock':clock,'_outcome':outcome,'RecoveryGate':RecoveryGate,
        '_slow_notice':Mock(),'MotionFeedback':lambda:S(last_rx=None),'ui':S(notify=Mock())}
 funcs=[n for n in ast.parse(SOURCE).body if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef,ast.ClassDef))and n.name in ['RoutePaused','RouteLocalizationWait','_route_uses_prediction','_route_live','_align_route_station','_wait_route_motion','_start_route_motion_when_ready','_start_owned_motion','_stop_owned_motion','change_navigation_mode','_await_route_stable','_hold_route_for_localization','_execute_recorded_route','run_recorded_route','stop_slow_motion','pause_recorded_route']]
 scope['notify_user']=scope['ui'].notify
 exec(compile(ast.Module(body=funcs,type_ignores=[]),'route_runner','exec'),scope)
 return scope,goals,started


def test_return_to_first_point_then_visit_in_order(tmp_path):
 scope,goals,started=harness(tmp_path);asyncio.run(scope['run_recorded_route']())
 assert goals==started==[(1.,0.),(2.,0.),(3.,0.)]
 assert not scope['TASK_STATE'].active;assert scope['ui'].notify.call_count==1
 assert scope['ui'].notify.call_args.kwargs['type']=='positive'


def test_pause_during_second_station_dwell_keeps_station_and_remaining_wait(tmp_path):
 scope,goals,started=harness(tmp_path)
 scope['TEACH_SESSION'].loaded['points'][1]['wait_s']=.2
 async def pause_on_sleep(delay):
  await scope['_tick'](delay)
  if len(goals)>=2 and scope['TASK_STATE'].feedback.get('state')=='ARRIVED':await scope['pause_recorded_route']()
 scope['asyncio'].sleep=pause_on_sleep
 asyncio.run(scope['run_recorded_route']())
 assert goals==[(1.,0.),(2.,0.)]
 assert scope['TASK_STATE'].context['index']==1
 assert 0<scope['TASK_STATE'].context['remaining_wait']<=.2
 assert scope['ui'].notify.call_count==0
 scope['asyncio'].sleep=scope['_tick']
 asyncio.run(scope['run_recorded_route'](True))
 assert goals==[(1.,0.),(2.,0.),(3.,0.)]
 assert scope['TASK_STATE'].context is None
 assert scope['ui'].notify.call_args.kwargs['type']=='positive'


def test_cancel_paused_route_clears_resume_context(tmp_path):
 scope,_,_=harness(tmp_path,'MANUAL_OVERRIDE')
 asyncio.run(scope['run_recorded_route']())
 assert scope['TASK_STATE'].context is not None
 asyncio.run(scope['stop_slow_motion']())
 assert scope['TASK_STATE'].context is None
 assert scope['STATE'].motion_result=='已取消暂停路线'


def test_two_stops_from_distant_position_goes_to_original_a_then_b_without_rebasing(tmp_path):
 scope,goals,started=harness(tmp_path)
 route=scope['TEACH_SESSION'].loaded
 route['points']=[waypoint('A',2.,3.,0.,0.,1.),waypoint('B',3.,3.,0.,0.,1.)]
 route['orientation_timing']='after_arrival'
 original=json.loads(json.dumps(route))
 scope['STATE'].pose=(20.,20.,0.)
 asyncio.run(scope['run_recorded_route']())
 assert goals==started==[(2.,3.),(3.,3.)]
 assert scope['STATE'].pose==(3.,3.,0.)
 assert route==original
 assert scope['ui'].notify.call_args.kwargs['type']=='positive'


def test_localization_gap_parks_and_preserves_route_without_relaxing_limit(tmp_path):
 scope,goals,started=harness(tmp_path)
 original=scope['PROCS'].start_slow_motion
 def gap(*args):
  result=original(*args)
  scope['STATE'].age_of=lambda key:.577 if key=='pose' else .1
  return result
 scope['PROCS'].start_slow_motion=gap
 asyncio.run(scope['run_recorded_route']())
 assert len(goals)==len(started)==1
 assert scope['TASK_STATE'].context['index']==0
 assert '30秒' in scope['STATE'].motion_result
 assert scope['STATE'].motion_feedback.terminal
 assert not scope['PROCS'].running('slow_motion')
 assert scope['ui'].notify.call_count==0


def test_zero_receive_age_is_fresh(tmp_path):
 scope,goals,_=harness(tmp_path)
 scope['STATE'].age_of=lambda key:0.
 # Test the live gate directly: the planner's independent path gate is outside this check.
 assert scope['_route_live']()[1]==(10,0,0)


def test_short_gap_recovers_automatically_and_replans_same_station(tmp_path):
 scope,goals,started=harness(tmp_path)
 original=scope['PROCS'].start_slow_motion
 recovery_at=[None]
 original_enable=scope['WORKER'].set_route_enabled
 def enable(value):
  original_enable(value)
  if len(started)==1:
   scope['STATE'].pose=(10,0,0)
   scope['STATE'].age_of=lambda key:.65 if key=='pose' else .1
   recovery_at[0]=scope['_clock'][0]+.7
 async def recover(delay):
  await scope['_tick'](delay)
  if recovery_at[0] is not None and scope['_clock'][0]>=recovery_at[0]:
   scope['STATE'].age_of=lambda key:.1
 scope['WORKER'].set_route_enabled=enable;scope['asyncio'].sleep=recover
 asyncio.run(scope['run_recorded_route']())
 assert goals==started==[(1.,0.),(1.,0.),(2.,0.),(3.,0.)]
 assert scope['TASK_STATE'].context is None
 assert scope['ui'].notify.call_args.kwargs['type']=='positive'


def test_manual_takeover_while_waiting_prevents_automatic_restart(tmp_path):
 scope,goals,started=harness(tmp_path)
 original=scope['PROCS'].start_slow_motion
 def move(*args):
  result=original(*args)
  scope['STATE'].age_of=lambda key:.65 if key=='pose' else .1
  return result
 async def takeover(delay):
  await scope['_tick'](delay)
  if started:scope['TASK_STATE'].feedback['state']='MANUAL_OVERRIDE'
 scope['PROCS'].start_slow_motion=move;scope['asyncio'].sleep=takeover
 asyncio.run(scope['run_recorded_route']())
 assert len(started)==1 and scope['TASK_STATE'].context['index']==0
 assert 'MANUAL_OVERRIDE' in scope['STATE'].motion_result
 assert scope['ui'].notify.call_count==0


def test_delayed_timestamp_cannot_pass_with_fresh_receipt(tmp_path):
 import pytest
 scope,_,_=harness(tmp_path)
 scope['STATE'].pose_stamp_age_at_rx=1.2
 with pytest.raises(scope['RouteLocalizationWait']):scope['_route_live']()


def test_no_stop_ack_disallows_automatic_recovery(tmp_path):
 scope,_,started=harness(tmp_path)
 original=scope['WORKER'].set_route_enabled
 def enable(value):
  original(value)
  scope['STATE'].age_of=lambda key:.65 if key=='pose' else .1
  scope['TASK_STATE'].feedback['state']='MOVING'
 scope['WORKER'].set_route_enabled=enable
 scope['WORKER'].hold_route_motion=lambda:None
 asyncio.run(scope['run_recorded_route']())
 assert len(started)==1 and '未确认执行器停驶' in scope['STATE'].motion_result
 assert scope['TASK_STATE'].context['index']==0


def test_cancel_during_initial_stability_wait_starts_no_motion(tmp_path):
 scope,goals,started=harness(tmp_path)
 async def cancel(delay):
  await scope['_tick'](delay)
  scope['TASK_STATE'].cancel=True
 scope['asyncio'].sleep=cancel
 asyncio.run(scope['run_recorded_route']())
 assert not started and not goals and scope['TASK_STATE'].context is None


def test_external_goal_change_while_waiting_is_not_overwritten(tmp_path):
 scope,goals,started=harness(tmp_path)
 async def new_goal(delay):
  await scope['_tick'](delay)
  scope['STATE'].goal_seq+=1
 scope['asyncio'].sleep=new_goal
 asyncio.run(scope['run_recorded_route']())
 assert not started and not goals
 assert '目标被更改' in scope['STATE'].motion_result


def test_executor_startup_longer_than_two_seconds_stays_disabled_until_ready(tmp_path):
 scope,_,started=harness(tmp_path)
 original_start=scope['PROCS'].start_slow_motion
 original_enable=scope['WORKER'].set_route_enabled
 pending={};enabled_at=[]
 def start(*args):
  assert args[-1] is True
  result=original_start(*args)
  pending.update(task=args[2],ready_at=scope['_clock'][0]+4.)
  scope['TASK_STATE'].feedback={}
  return result
 async def delayed(delay):
  await scope['_tick'](delay)
  if pending and scope['_clock'][0]>=pending['ready_at'] and not scope['TASK_STATE'].feedback:
   scope['TASK_STATE'].feedback={'task_id':pending['task'],'state':'DISABLED','received_at':scope['_clock'][0]}
 def enable(value):
  assert scope['_clock'][0]>=pending['ready_at']+1.2
  enabled_at.append(scope['_clock'][0]);original_enable(value)
 scope['PROCS'].start_slow_motion=start
 scope['WORKER'].set_route_enabled=enable;scope['asyncio'].sleep=delayed
 asyncio.run(scope['run_recorded_route']())
 assert len(started)==len(enabled_at)==3
 assert scope['ui'].notify.call_args.kwargs['type']=='positive'


def test_executor_startup_timeout_never_enables_motion(tmp_path):
 scope,_,started=harness(tmp_path)
 original=scope['PROCS'].start_slow_motion
 def missing(*args):
  result=original(*args);scope['TASK_STATE'].feedback={};return result
 scope['PROCS'].start_slow_motion=missing
 scope['WORKER'].set_route_enabled=Mock()
 asyncio.run(scope['run_recorded_route']())
 scope['WORKER'].set_route_enabled.assert_not_called()
 assert len(started)==1 and '初始化20秒未就绪' in scope['STATE'].motion_result


def test_mode_switch_cancels_paused_route_and_clears_shared_context(tmp_path):
 scope,_,_=harness(tmp_path,'MANUAL_OVERRIDE')
 asyncio.run(scope['run_recorded_route']())
 shared=scope['TASK_STATE'];assert shared.context is not None
 shared.mode='手动记录路线'
 scope['UI']['navigation_mode']=S(value='地图选点移动')
 asyncio.run(scope['change_navigation_mode']('地图选点移动'))
 assert shared.context is None and not shared.active and not shared.switching
 assert shared.mode=='地图选点移动'


def test_another_page_can_cancel_owner_coroutine_and_then_switch_mode(tmp_path):
 first,_,_=harness(tmp_path/'first');second,_,_=harness(tmp_path/'second')
 second['TASK_STATE']=first['TASK_STATE'];second['PROCS']=first['PROCS']
 shared=first['TASK_STATE'];shared.mode='手动记录路线'
 second['UI']['navigation_mode']=S(value='地图选点移动')
 async def exercise():
  began=asyncio.Event();cleanup=[]
  async def owner():
   shared.active=True;shared.context={'index':1}
   began.set()
   try:await asyncio.Event().wait()
   finally:
    await first['_stop_owned_motion']();cleanup.append(True)
    shared.active=False;shared.context=None
  task=asyncio.create_task(owner());shared.task=task
  await began.wait()
  await second['change_navigation_mode']('地图选点移动')
  assert task.cancelled() and cleanup
  assert shared.task is None and shared.context is None and not shared.active
 asyncio.run(exercise())


def test_cancel_waits_for_inflight_spawn_before_stopping_process(tmp_path):
 scope,_,_=harness(tmp_path)
 async def exercise():
  shared=scope['TASK_STATE'];began=asyncio.Event();finish=asyncio.Event();calls=[]
  async def pending_spawn():
   began.set();await finish.wait();calls.append('spawned')
  shared.pending_spawn=asyncio.create_task(pending_spawn())
  await began.wait()
  original=scope['PROCS'].stop
  def stopped(*args):calls.append('stopped');original(*args)
  scope['PROCS'].stop=stopped
  stopper=asyncio.create_task(scope['stop_slow_motion']())
  await asyncio.sleep(0)
  assert calls==[]
  finish.set();await stopper
  assert calls==['spawned','stopped'] and shared.pending_spawn is None
 asyncio.run(exercise())


def test_stop_invalidates_pending_plan_and_releases_next_goal(tmp_path):
 scope,_,_=harness(tmp_path)
 state=scope['STATE'];state.plan_pending=True;state.global_pending=True;state.pending_goal={'x':1}
 state.plan_active=True;state.global_active=True;seq=state.goal_seq
 asyncio.run(scope['stop_slow_motion']())
 assert state.goal_seq==seq+1 and not state.plan_pending and not state.global_pending
 assert state.pending_goal is None and not state.plan_active and not state.global_active


def test_invalid_localization_session_remains_terminal_failure(tmp_path):
 scope,goals,_=harness(tmp_path)
 scope['STATE'].loc['reloc_ok']=False
 asyncio.run(scope['run_recorded_route']())
 assert not goals and scope['TASK_STATE'].context is None
 assert scope['ui'].notify.call_args.kwargs['type']=='negative'


def test_manual_takeover_stops_without_skipping_to_next_station(tmp_path):
 scope,goals,started=harness(tmp_path,'MANUAL_OVERRIDE');asyncio.run(scope['run_recorded_route']())
 assert len(goals)==len(started)==1
 assert scope['TASK_STATE'].context['index']==0
 assert scope['ui'].notify.call_count==0
 assert not scope['TASK_STATE'].active


def test_speed_change_requires_matching_navigation_profile(tmp_path):
 scope,goals,started=harness(tmp_path);scope['UI']['slow_speed'].value=.8
 asyncio.run(scope['run_recorded_route']())
 assert not goals and not started and not scope['TASK_STATE'].active


def test_map_switch_during_travel_stops_current_route(tmp_path):
 scope,goals,started=harness(tmp_path)
 original=scope['PROCS'].start_slow_motion
 def changed(*args):
  result=original(*args);scope['STATE'].loc['map_path']='different.pcd';return result
 scope['PROCS'].start_slow_motion=changed
 asyncio.run(scope['run_recorded_route']())
 assert len(goals)==1 and scope['ui'].notify.call_args.kwargs['type']=='negative'


def test_alignment_and_manual_pause_resume_current_station(tmp_path):
 scope,goals,started=harness(tmp_path,'MANUAL_OVERRIDE')
 scope['TEACH_SESSION'].loaded['points'][0]['yaw']=1.2
 asyncio.run(scope['run_recorded_route']())
 assert scope['_rotations']==[1.2]
 assert scope['TASK_STATE'].context['index']==0
 assert not goals  # Manual takeover during alignment cannot advance into planning.
 # Resume with restored control at the same station; never start again at an earlier index.
 scope['_outcome']='ARRIVED'
 asyncio.run(scope['run_recorded_route'](True))
 assert goals==[(1.,0.),(2.,0.),(3.,0.)]
 assert scope['TASK_STATE'].context is None
 assert scope['ui'].notify.call_args.kwargs['type']=='positive'


def test_route_uses_bounded_control_pose_but_keeps_raw_pose_unchanged(tmp_path):
 scope,_,_=harness(tmp_path);s=scope['STATE'];raw=s.pose
 s.control_pose=(.1,.2,.3);s.control_pose_stamp_age=.05
 s.readiness={'pose_prediction':{'horizon_s':2.},'status':'LIVE_SCAN_INPUTS_READY','body_pose_live_fresh':True}
 s.age_of=lambda key:1.5 if key=='pose' else .1
 assert scope['_route_live']()[1]==s.control_pose
 assert s.pose==raw
 s.readiness['body_pose_live_fresh']=False
 import pytest
 with pytest.raises(scope['RouteLocalizationWait']):scope['_route_live']()
 s.readiness['body_pose_live_fresh']=True
 s.age_of=lambda key:1. if key=='readiness' else .1
 with pytest.raises(scope['RouteLocalizationWait']):scope['_route_live']()


def test_prediction_does_not_bypass_route_map_file_binding(tmp_path):
 import pytest
 scope,_,_=harness(tmp_path);s=scope['STATE'];path=Path(s.loc['map_path'])
 scope['TASK_STATE'].binding=(str(path),[(str(path),path.stat().st_size,path.stat().st_mtime_ns)])
 path.write_bytes(b'changed map content')
 with pytest.raises(RuntimeError,match='地图或安装模型文件已改变'):
  scope['_route_live']()


def test_four_stations_translate_then_rotate_and_wait_one_second(tmp_path):
 scope,goals,started=harness(tmp_path);r=scope['TEACH_SESSION'].loaded
 r['orientation_timing']='after_arrival';r['return_to_start']=False
 r['points']=[waypoint(name,x,0,0,yaw,1.) for name,x,yaw in [('起点',0,0),('中间点1',1,math.pi/2),('中间点2',2,0),('终点',3,0)]]
 scope['STATE'].pose=(3,0,0);actions=[]
 old_start=scope['PROCS'].start_slow_motion;old_rotate=scope['PROCS'].start_rotation
 def start(*args):actions.append(('move',args[3]));return old_start(*args)
 def rotate(task,yaw):actions.append(('rotate',scope['STATE'].pose[:2],yaw));return old_rotate(task,yaw)
 scope['PROCS'].start_slow_motion=start;scope['PROCS'].start_rotation=rotate
 before=scope['_clock'][0]
 asyncio.run(scope['run_recorded_route']())
 assert goals==[(0.,0.),(1.,0.),(2.,0.),(3.,0.)]
 assert actions==[('move',(0.,0.)),('move',(1.,0.)),('rotate',(1.,0.),math.pi/2),('move',(2.,0.)),('rotate',(2.,0.),0.),('move',(3.,0.))]
 assert scope['_clock'][0]-before>=4
 assert scope['ui'].notify.call_args.kwargs['type']=='positive'


def test_route_keeps_tracking_during_three_second_global_pose_gap(tmp_path):
 scope,_,_=harness(tmp_path);s=scope['STATE']
 s.control_pose=(.09,0.,0.);s.control_pose_stamp_age=.02
 s.readiness={'pose_prediction':{'fault':'','horizon_s':5.},
              'robot_pose_stamp_age_s':3.,'robot_pose_raw_receive_fresh':False,
              'status':'LIVE_SCAN_INPUTS_READY','body_pose_live_fresh':True}
 s.age_of=lambda key:3. if key in ('pose','loc_status') else .05
 assert scope['_route_live']()[1]==s.control_pose
 assert s.pose==(10,0,0)  # Raw held observation remains untouched.
 s.readiness['status']='WAITING_OR_BLOCKED'
 import pytest
 with pytest.raises(scope['RouteLocalizationWait']):scope['_route_live']()
