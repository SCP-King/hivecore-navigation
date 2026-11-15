"""Two mode workflows; realtime SLAM does not expose a separate localization task."""
import ast
import asyncio
import itertools
import sys
import threading
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation_ui import slam_selection, workflow_help, is_realtime_backend

CASES = [(fast, task, realtime) for fast in (False, True)
         for task,realtime in [('建图',False),('定位',False),('建图',True)]]
SOURCE = (Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()


def function(name):
    return next(n for n in ast.parse(SOURCE).body
                if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == name)


@pytest.mark.parametrize('fast,task,realtime', CASES)
def test_offline_tasks_and_unified_realtime_slam_have_explicit_semantics(fast, task, realtime):
    selection = slam_selection(fast, task, realtime)
    assert selection['task'] == task
    assert selection['record_bag'] == (task == '建图')
    assert selection['needs_pcd'] == (task == '定位' and not realtime)
    assert selection['navigation_allowed'] == (realtime or task == '定位')
    assert is_realtime_backend('mapping', True, selection['map_mode']) == realtime
    help = workflow_help(fast, realtime)
    assert '实时SLAM' in help['summary'] if realtime else '离线地图' in help['summary']
    if realtime:
        assert '实时' in help['summary']


@pytest.mark.parametrize('fast,realtime', itertools.product((False,True),(False,True)))
def test_mode_switch_changes_visible_actions_and_hides_offline_localization(fast,realtime):
    controls = {key:S(value=value, set_enabled=Mock(), set_text=Mock(), set_visibility=Mock())
                for key,value in [('slam_backend','fast' if fast else 'vendor'),
                 ('slam_map_mode','realtime' if realtime else 'offline'),
                 ('backend_help',''),('loc_workflow',''),('loc_map_action',''),('map_path',''),
                 ('offline_slam_actions',''),('realtime_slam_actions',''),
                 ('offline_localization_controls',''),('offline_loc_advanced',''),
                 ('offline_map_start',''),('offline_loc_start',''),('realtime_slam_start','')]}
    scope = dict(UI=controls, PROCS=S(running=lambda _:False), BACKEND_FAST_LIO='fast',
                 FAST_MODE_REALTIME='realtime', workflow_help=workflow_help)
    exec(compile(ast.Module(body=[function('refresh_backend_descriptions')],type_ignores=[]),'selectors','exec'),scope)
    scope['refresh_backend_descriptions']()
    assert 'slam_task' not in controls and 'UI["slam_task"]' not in SOURCE
    controls['realtime_slam_actions'].set_visibility.assert_called_with(realtime)
    for key in ('offline_slam_actions','offline_localization_controls','offline_loc_advanced'):
        controls[key].set_visibility.assert_called_with(not realtime)
    controls['map_path'].set_enabled.assert_called_with(not realtime)


@pytest.mark.parametrize('fast', [False,True])
def test_realtime_has_no_independent_localization_profile(fast):
    with pytest.raises(ValueError, match='统一启动SLAM'):
        slam_selection(fast,'定位',True)


@pytest.mark.parametrize('fast,task,realtime', CASES)
def test_each_gui_combination_starts_selected_backend_without_starting_navigation(fast, task, realtime):
    state = S(lock=threading.Lock(),loc={},mapping={},recv_t={})
    procs = S(running=lambda _:False,slam_selection={})
    started=[]
    async def vendor_start():
        started.append(('vendor',dict(procs.slam_selection)));return True
    def fast_start(manager, mode, saved, output, record_bag):
        assert manager is procs
        started.append(('fast',dict(procs.slam_selection),mode,record_bag));return True
    ui = {key:S(value=value) for key,value in [('slam_backend','fast' if fast else 'vendor'),
          ('slam_map_mode','realtime' if realtime else 'offline'),('mg_enable',False),
          ('mg_mode',''),('map_path','/tmp/reference.pcd'),('fast_lio_map','/tmp/output.pcd')]}
    scope = dict(PROCS=procs,STATE=state,UI=ui,ProcManager=S(start_fast_lio=fast_start),
                 BACKEND_VENDOR='vendor',BACKEND_FAST_LIO='fast',FAST_MODE_REALTIME='realtime',
                 DEFAULT_FAST_LIO_MAP='/tmp/output.pcd',slam_selection=slam_selection,
                 start_desk=vendor_start,asyncio=asyncio,ui=S(notify=Mock()),push_log=Mock())
    exec(compile(ast.Module(body=[function('lifecycle_start_slam')],type_ignores=[]),'startup','exec'),scope)
    asyncio.run(scope['lifecycle_start_slam']('realtime' if realtime else ('mapping' if task=='建图' else 'loc')))
    assert len(started)==1 and started[0][0]==('fast' if fast else 'vendor')
    assert started[0][1]['task']==task
    engines = {(False,False,'定位'):'loc',(False,False,'建图'):'mapping',
               (False,True,'定位'):'mapping',(False,True,'建图'):'mapping',
               (True,False,'定位'):'loc',(True,False,'建图'):'mapping',
               (True,True,'定位'):'realtime',(True,True,'建图'):'realtime'}
    assert procs.slam_selection['engine_mode']==engines[fast,realtime,task]
    assert state.recv_t['pose']==0 and state.recv_t['n01_status']==0
