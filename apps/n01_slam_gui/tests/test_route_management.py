"""Real GUI route management function, using local files and no ROS worker."""
import ast
import asyncio
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from navigation_session import NavigationSession
from teach_routes import RouteStore,map_identity,waypoint


def harness(tmp_path,confirm=True):
    p=tmp_path/'map.pcd';p.write_bytes(b'map')
    store=RouteStore(tmp_path/'routes')
    path=store.save('A-B',map_identity(p),[waypoint('A',0,0,0,0)])
    selected=S(value=path,options=store.list(),update=Mock())
    async def inline(fn,*args):return fn(*args)
    async def ask(*args):return confirm
    scope=dict(TASK_STATE=NavigationSession(),TEACH_SESSION=S(busy=False,loaded=store.load(path)),TEACH_STORE=store,
        ROUTE_ACTIVE=False,ROUTE_CONTEXT=None,Path=Path,
        UI={'teach_saved':selected,'teach_saved_info':S(set_text=Mock())},
        asyncio=S(to_thread=inline),_confirm=ask,_teach_view=Mock(),push_log=Mock(),ui=S(notify=Mock()))
    tree=ast.parse((Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text())
    fn=next(n for n in tree.body if isinstance(n,ast.AsyncFunctionDef) and n.name=='manage_saved_route')
    exec(compile(ast.Module(body=[fn],type_ignores=[]),'route_management','exec'),scope)
    return scope,Path(path)


def test_delete_clears_loaded_route_and_selector(tmp_path):
    scope,path=harness(tmp_path)
    asyncio.run(scope['manage_saved_route']('delete'))
    assert not path.exists() and scope['TEACH_SESSION'].loaded is None
    assert scope['UI']['teach_saved'].value is None
    assert scope['UI']['teach_saved'].options=={}
    assert not scope['TEACH_SESSION'].busy


@pytest.mark.parametrize('state',['active','paused','declined'])
def test_delete_requires_inactive_task_and_confirmation(tmp_path,state):
    scope,path=harness(tmp_path,confirm=state!='declined')
    scope['TASK_STATE'].active=state=='active'
    scope['TASK_STATE'].context={} if state=='paused' else None
    asyncio.run(scope['manage_saved_route']('delete'))
    assert path.exists() and scope['TEACH_SESSION'].loaded is not None


def test_management_does_not_require_localization(tmp_path):
    scope,path=harness(tmp_path)
    asyncio.run(scope['manage_saved_route']('info'))
    assert 'map.pcd' in scope['UI']['teach_saved_info'].set_text.call_args.args[0]
    asyncio.run(scope['manage_saved_route']('refresh'))
    assert str(path) in scope['UI']['teach_saved'].options


def test_delete_rechecks_task_after_confirmation(tmp_path):
    scope,path=harness(tmp_path)
    async def changed(*args):
        scope['TASK_STATE'].active=True
        return True
    scope['_confirm']=changed
    asyncio.run(scope['manage_saved_route']('delete'))
    assert path.exists()
