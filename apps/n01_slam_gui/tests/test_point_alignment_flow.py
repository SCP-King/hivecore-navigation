import ast,asyncio,math,sys,time,threading,uuid
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from navigation_ui import alignment_before_replan


def test_right_front_52_degree_target_can_request_checked_rotation_without_local_path():
    pose=(-.295,0,0);target=(.61,-1.18);route=[pose[:2],target]
    h=alignment_before_replan(pose,target,route,.1,False,'heading_search_time_budget')
    assert h is not None and -55<math.degrees(h)<-45
    assert alignment_before_replan(pose,target,route,.1,True,'heading_search_time_budget') is None
    for reason in ('start_footprint_occupied_or_outside_window','heading_search_waiting_for_fresh_body_pose'):
        assert alignment_before_replan(pose,target,route,.1,False,reason) is None
    assert alignment_before_replan(pose,target,route,3.,False,'heading_search_time_budget') is None
    assert alignment_before_replan(pose,target,[(0,0),(9,9)],.1,False,'heading_search_time_budget') is None


def test_start_orders_checked_turn_replan_then_disabled_translation_and_enable():
    actions=[];state=S(lock=threading.Lock(),loc={'session':'tracking'},plan_active=False,
        control_pose=(-.295,0,0),goal_target=(.61,-1.18),goal_seq=1,global_xy=[(-.295,0),(.61,-1.18)],
        feasibility={'reason':'heading_search_time_budget'},age_of=lambda k:.1,nav_speed_mps=.05,
        motion_feedback=S(terminal=False),goal_failed=False,plan_goal_ns=0,plan_success_ns=0)
    proc=S(desk_mode='mapping',running=lambda name:name=='desk',start_slow_motion=Mock())
    task=S(switching=False,context=None,active=False,task=None,cancel=False,pause=False,feedback={})
    async def turn(target,label,token=None):
        actions.append('checked_turn');state.control_pose=(*state.control_pose[:2],target['yaw'])
    def goal(*args):
        actions.append('replan');state.goal_seq+=1;state.plan_active=True;state.plan_goal_ns=1;state.plan_success_ns=2
    async def owned(*args):
        assert args[-1] is True
        actions.append('translation_disabled');return True
    async def ready(*args):actions.append('enable_after_ready')
    async def stop():actions.append('stop')
    scope=dict(STATE=state,PROCS=proc,TASK_STATE=task,TEACH_SESSION=S(active=False,busy=False),
        UI={},asyncio=asyncio,time=time,math=math,uuid=uuid,_nav_ready_state=lambda:True,
        _mapping_navigation_ready=lambda:True,_route_live=lambda:('',state.control_pose),
        _align_route_station=turn,_start_owned_motion=owned,_start_route_motion_when_ready=ready,
        _stop_owned_motion=stop,WORKER=S(request_goal=goal),MotionFeedback=lambda:S(last_rx=None),
        _slow_notice=Mock(),notify_user=Mock(),push_log=Mock())
    source=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()
    functions=[n for n in ast.parse(source).body if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef))
               and n.name in ('_point_alignment_heading','start_slow_motion')]
    exec(compile(ast.Module(body=functions,type_ignores=[]),'alignment_flow','exec'),scope)
    asyncio.run(scope['start_slow_motion']())
    assert actions==['checked_turn','replan','translation_disabled','enable_after_ready']
    assert not scope['_slow_notice'].call_args.args[0].startswith('本次行驶未启动')
