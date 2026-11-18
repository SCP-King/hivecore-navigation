"""A freshly republished previous goal must not fail the next GUI request."""
import ast,asyncio,math,threading,time
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock

def test_waits_for_matching_endpoint_after_newly_stamped_previous_path():
 source=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()
 function=next(n for n in ast.parse(source).body if isinstance(n,ast.AsyncFunctionDef)and n.name=='send_goal')
 state=S(lock=threading.Lock(),ros_ready=True,plan_pending=False,global_pending=False,
         goal_seq=1,plan_goal_ns=1000,global_success_ns=0,global_pts=502,
         global_goal_distance=None,goal_status={},plan_success_ns=0,plan_pts=501,plan_active=True)
 calls=[]
 async def sleep(_):
  calls.append(1)
  state.global_success_ns=2000
  state.global_goal_distance=.52 if len(calls)==1 else 0.
  state.plan_success_ns=3000
 async def confirm(*_):return True
 ui={k:S(value=v)for k,v in [('goal_x',.17),('goal_y',-.1),('goal_yaw',0)]}
 ui['goal_result']=S(set_text=Mock())
 scope={'TASK_STATE':S(active=False),'STATE':state,'UI':ui,'WORKER':S(request_goal=Mock()),
        'ui':S(notify=Mock()),'math':math,'time':time,'os':__import__('os'),
        'asyncio':S(sleep=sleep),'_confirm':confirm,'push_log':Mock(),
        'draw_grid':Mock()}
 exec(compile(ast.Module(body=[function],type_ignores=[]),'gui_goal','exec'),scope)
 asyncio.run(scope['send_goal']())
 texts=[c.args[0]for c in ui['goal_result'].set_text.call_args_list]
 assert any(t.startswith('规划成功：')for t in texts)
 assert not any(t.startswith('规划失败')for t in texts)
 assert len(calls)>=3


def test_backend_failure_is_reported_without_waiting_for_three_minute_timeout():
 source=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()
 function=next(n for n in ast.parse(source).body if isinstance(n,ast.AsyncFunctionDef)and n.name=='send_goal')
 state=S(lock=threading.Lock(),ros_ready=True,plan_pending=False,global_pending=False,
         goal_seq=1,plan_goal_ns=time.time_ns(),global_success_ns=0,global_pts=0,
         global_goal_distance=None,goal_status={},plan_success_ns=0,plan_pts=0,plan_active=False)
 calls=[]
 async def sleep(_):
  calls.append(1)
  state.planning_status={'goal_id':state.plan_goal_ns,'phase':'GLOBAL_FAILED','reason':'goal_occupied'}
 async def confirm(*_):return True
 ui={k:S(value=v)for k,v in [('goal_x',1.),('goal_y',0.),('goal_yaw',0)]}
 ui['goal_result']=S(set_text=Mock())
 scope={'TASK_STATE':S(active=False),'STATE':state,'UI':ui,'WORKER':S(request_goal=Mock()),
        'math':math,'time':time,'os':__import__('os'),'asyncio':S(sleep=sleep),
        '_confirm':confirm,'push_log':Mock(),'notify_user':Mock(),'draw_grid':Mock()}
 exec(compile(ast.Module(body=[function],type_ignores=[]),'gui_goal','exec'),scope)
 asyncio.run(scope['send_goal']())
 assert len(calls)==1 and state.goal_failed
 assert not state.plan_pending and not state.global_pending
 assert '终点落在已知障碍内'in ui['goal_result'].set_text.call_args.args[0]
