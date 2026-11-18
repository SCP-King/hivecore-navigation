import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from navigation_flow import build_flow,render_flow


def test_missing_model_does_not_claim_a_measured_obstacle_at_start():
    from navigation_flow import describe
    message=describe('start_envelope_footprint_missing@(-0.30,0.00,-0.15)')
    assert '模型缺失或过期' in message and '实测障碍' not in message


def test_waiting_execution_exposes_planner_root_and_recovery_in_whole_flow():
    planning=dict(goal_id=7,global_phase='GLOBAL_READY',global_reason='reference_available',
        local_phase='WAITING_FOR_OBSERVATION',local_reason='no_reachable_observed_frontier',
        recovery='new_observation_or_alternative_frontier',frontier_expanded=12,local_attempts=3,global_replans=1)
    f=build_flow(7,planning,dict(goal_id=7,reason='no_bspline_received'),
        dict(goal_id=7,state='WAITING_FOR_FEEDBACK',reason='waiting_for_native_scan_trajectory'))
    assert len(f['stages'])==8
    assert f['stages'][2]['state']=='done' and f['stages'][3]['state']=='waiting'
    assert '机身可通过'in f['causes'][0] and '自动重选'in f['recovery']
    assert '3 次'in f['loops'] and '<details'in render_flow(f)


def test_previous_goal_arrival_or_failure_cannot_color_current_flow():
    f=build_flow(8,dict(goal_id=7,phase='GLOBAL_FAILED',reason='goal_occupied'),
        dict(goal_id=7,candidate_review_ready=True),dict(goal_id=7,state='ARRIVED'))
    assert f['stages'][2]['state']=='waiting'
    assert f['stages'][5]['state']!='done' and f['stages'][6]['state']!='done'
    assert not any('终点机身'in c for c in f['causes'])


def test_flow_renders_data_as_text_and_never_interprets_failure_html():
    f=build_flow(7,dict(goal_id=7,phase='GLOBAL_FAILED',reason='<script>alert(1)</script>'))
    output=render_flow(f)
    assert '<script>'not in output and '&lt;script&gt;'in output


def test_stale_computing_status_is_not_claimed_as_live_computation():
    f=build_flow(7,dict(goal_id=7,local_phase='LOCAL_PLANNING'),ages={'planning':9.})
    assert f['stages'][4]['state']=='waiting' and '不能确认'in f['stages'][4]['detail']


def test_gui_flow_snapshot_never_calls_locking_age_functions_under_state_lock():
    import ast,threading
    from types import SimpleNamespace as S
    source=Path(__file__).resolve().parents[1]/'slam_lite_gui.py'
    fn=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='_run_flow_html')
    lock=threading.Lock()
    def age(_):
        assert lock.acquire(blocking=False), 'flow snapshot reentered STATE.lock'
        lock.release();return .1
    state=S(lock=lock,plan_goal_ns=7,planning_status={},feasibility={},scene_status={},readiness={},
        navigation_task=S(phase='PLANNING',reason=''),age_of=age)
    from map_presentation import scan_freshness
    import time
    scope=dict(STATE=state,_nav_ready_state=lambda:True,scan_freshness=scan_freshness,time=time)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),scope)
    assert scope['_run_flow_html']().count('<details')==8


def test_static_ready_and_fresh_heartbeat_do_not_hide_stale_scan_without_goal():
    f=build_flow(0,scene=dict(ready=True,scan_ready=False,scan_age_s=75.,scan_reason='occupancy_stale'),
                 running=True,ages={'scene':.1})
    scene=f['stages'][1]
    assert scene['state']=='waiting'
    assert '75.0s' in scene['detail'] and '/cloud' in scene['detail']
    assert any('原始扫描过期' in cause for cause in f['causes'])
