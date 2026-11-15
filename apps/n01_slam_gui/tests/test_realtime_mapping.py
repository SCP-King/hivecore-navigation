import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation_ui import is_realtime_backend, mapping_ready, localization_help


def test_mapping_task_uses_realtime_workflow_without_offline_map_requirement():
    assert is_realtime_backend('mapping')
    assert is_realtime_backend('fast_lio_realtime')
    assert not is_realtime_backend('loc')
    assert not is_realtime_backend('mapping', False)
    text = localization_help(False, True)
    assert '甲方' in text['summary'] and '无需加载旧 PCD 或重定位' in text['workflow']


@pytest.mark.parametrize('changes,age', [({},.1), ({'session':'idle'},.1),
    ({'mapping_pose_fresh':'false'},.1), ({'source_status_age_s':'3'},.1), ({},2)])
def test_mapping_gui_rejects_missing_pose_and_old_source_even_when_recently_received(changes, age):
    data = dict(readiness_semantics='mapping_session_and_fresh_pose', session='mapping',
                mapping_pose_fresh='true', status_mapping_valid='true', source_status_age_s='0.1')
    data.update(changes)
    assert mapping_ready(data, age) is (not changes and age == .1)


@pytest.mark.parametrize('pose_fresh', ['true', 'false'])
def test_gui_navigation_uses_current_mapping_session_and_never_requires_old_pcd(pose_fresh):
    import ast, asyncio, math, os, threading
    from types import SimpleNamespace as S
    from unittest.mock import Mock
    source = (Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()
    functions = [n for n in ast.parse(source).body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
                 and n.name in ('_slam_uses_live_map', '_mapping_navigation_ready', 'navigation_action')]
    lifecycle = S(navigation=Mock(return_value=(True, 'ready')))
    # An old offline tracking status must not bypass invalid mapping readiness.
    state = S(lock=threading.Lock(), loc=dict(session='tracking', map_loaded=True, reloc_ok=True),
              mapping_nav=dict(readiness_semantics='mapping_session_and_fresh_pose', session='mapping',
                mapping_pose_fresh=pose_fresh, status_mapping_valid='true', source_status_age_s='0.1'),
              body_pose_xyz=(0.,0.,0.), age_of=lambda _: .1)
    async def ground(realtime):
        assert realtime
        return S(c=-.8,a=0.,b=0.,residual=.01,height=lambda *_:-.8), (0.,0.)
    scope = dict(PROCS=S(desk_mode='mapping',running=lambda _:True), STATE=state,
                 UI=dict(nav_result=S(set_text=Mock()),nav_ground_z=S(value=-.8),slow_speed=S(value=.03)),
                 is_realtime_backend=is_realtime_backend,mapping_ready=mapping_ready,
                 navigation_ground_model=ground,LIFECYCLE=lifecycle,
                 asyncio=asyncio,math=math,os=os,push_log=Mock(),notify_user=Mock())
    exec(compile(ast.Module(body=functions,type_ignores=[]),'gui_mapping_navigation','exec'), scope)
    asyncio.run(scope['navigation_action']('start'))
    if pose_fresh == 'true':
        env = lifecycle.navigation.call_args.kwargs['env_extra']
        assert env['N01_NAV_MAP_MODE'] == 'realtime'
        assert env['N01_NAV_REALTIME_REFERENCE_TOPIC'] == '/cloud_map'
        assert 'N01_NAV_PCD' not in env
    else:
        lifecycle.navigation.assert_not_called()


@pytest.mark.parametrize('ready', [True, False])
def test_motion_gate_uses_realtime_mapping_status_without_offline_relocation(ready):
    import ast, threading
    from types import SimpleNamespace as S
    source=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()
    fn=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='_route_live')
    state=S(lock=threading.Lock(),loc={'session':'—','reloc_ok':None},pose=(1,2,0),
            pose_stamp_age_at_rx=.1,age_of=lambda key: .1 if key!='loc_status' else None)
    scope=dict(STATE=state,TASK_STATE=S(pause=False,cancel=False,binding=None),
        PROCS=S(desk_mode='mapping'),_slam_uses_live_map=lambda:True,
        _mapping_navigation_ready=lambda:ready,_route_uses_prediction=lambda:False,
        RouteLocalizationWait=ValueError)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),'motion_mapping_gate','exec'),scope)
    if ready:assert scope['_route_live']()[1]==(1,2,0)
    else:
        with pytest.raises(ValueError,match='实时建图'):scope['_route_live']()


def test_rear_point_goal_faces_target_without_overriding_front_or_near_goal():
    import math
    from navigation_ui import rear_goal_heading
    assert abs(rear_goal_heading((0,0,0),(-1,0)))==pytest.approx(math.pi)
    assert rear_goal_heading((0,0,0),(1,0)) is None
    assert rear_goal_heading((0,0,0),(-.01,0)) is None
    assert rear_goal_heading((0,0,math.pi),(-1,0)) is None
