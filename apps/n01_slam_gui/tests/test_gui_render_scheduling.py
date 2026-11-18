"""Exercise the production map scheduler without starting ROS or a web server."""
import ast
import asyncio
from pathlib import Path
from types import SimpleNamespace


def test_slow_frames_coalesce_and_do_not_block_browser_heartbeat():
    source = Path(__file__).resolve().parents[1] / 'slam_lite_gui.py'
    tree = ast.parse(source.read_text())
    selected = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
                and n.name in ('draw_grid', '_draw_grid_pending')]

    async def run():
        view = SimpleNamespace(caches={}, tasks=set())
        owner = SimpleNamespace(available=True, view=lambda: view)
        started = []
        async def frame(force):
            started.append(force)
            # Same scheduling boundary used by the real renderer.
            import time
            await asyncio.to_thread(time.sleep, .08)
        scope = dict(asyncio=asyncio, UI=owner, _draw_grid_async=frame,
                     push_log=lambda *args: None)
        exec(compile(ast.Module(body=selected, type_ignores=[]), str(source), 'exec'), scope)
        scope['draw_grid'](True)
        await asyncio.sleep(.01)
        for _ in range(20):
            scope['draw_grid']()
        await asyncio.sleep(.01)
        assert started == [True]
        assert not view.caches['grid_task'].done()
        await view.caches['grid_task']
        scope['draw_grid']()
        await view.caches['grid_task']
        assert len(started) == 2
    asyncio.run(run())


def test_background_frame_encodes_real_png_without_widgets():
    import base64
    import io
    import numpy as np
    from PIL import Image
    source = Path(__file__).resolve().parents[1] / 'slam_lite_gui.py'
    functions = [n for n in ast.parse(source.read_text()).body
                 if isinstance(n, ast.FunctionDef) and n.name in ('_render_grid_frame','_finish_frame')]
    scope = dict(np=np, Image=Image, io=io, base64=base64, GRID_IMAGE_PX=64,
                 _rasterize=lambda *args: (np.zeros((10, 10, 3), dtype=np.uint8), .03, 1.6),
                 paint_robot=lambda img, *args, **kwargs: img,
                 _grid_overlay=lambda img, *args: img)
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), scope)
    data, low, high = scope['_render_grid_frame'](
        np.array([[0., 0., .1]]), {}, 0., None, (0., 0., 0.), 0.,
        None, None, [], None, [], None, [], False, False, [], False, True)
    image = Image.open(io.BytesIO(base64.b64decode(data.split(',', 1)[1])))
    assert image.size == (64, 64) and low == .03 and high == 1.6


def test_grid_only_frame_reaches_png_encoder_with_and_without_robot_pose():
    """Exercise the complete async route which previously returned a black pixel."""
    import base64,io,math,threading,time
    import numpy as np
    from PIL import Image
    from unittest.mock import Mock
    S=SimpleNamespace
    source=Path(__file__).resolve().parents[1]/'slam_lite_gui.py'
    functions=[n for n in ast.parse(source.read_text()).body
               if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) and n.name in
               ('_draw_grid_async','_render_occupancy_frame','_project_occupancy','_finish_frame')]
    class Widgets(dict):available=True
    for pose in (None,(0.,0.,0.)):
        data=np.zeros((40,40),np.int8);data[10:20,10:20]=100
        layer=(data,-1.,-1.,.05,time.monotonic())
        state=S(lock=threading.Lock(),plan_xy=[],plan_seq=0,plan_active=False,goal_failed=False,
                global_xy=[],global_seq=0,global_active=False,robot_projection=None,
                robot_projection_error='',projection_joints={},projection_joint_times={},body_tilt=(0.,0.),
                nav_inflated_pts=None,nav_inflated_seq=0,nav_dyn_pts=None,nav_risk_pts=None,
                nav_risk_seq=0,feasibility={},plan_goal_ns=0,ref_grid=layer,live_grid=None,
                age_of=lambda _:None)
        ui=Widgets(map_image=Mock(),grid_info=Mock(),src=S(value='导航'),range=S(value='20m'))
        scope=dict(UI=ui,STATE=state,_GRID_CACHE={},_GRID_VIEW={},_BLANK_PNG='black-pixel',
                   GRID_RES=.05,GRID_MAX_N=100,GRID_IMAGE_PX=100,_GRID_UNKNOWN=(8,12,20),
                   _GRID_FREE=(26,40,62),_GRID_REF=(116,151,182),_GRID_LIVE=(79,239,219),
                   asyncio=asyncio,np=np,Image=Image,io=io,base64=base64,math=math,time=time,
                   model_visual_key=lambda *a:'model',_nav_paths_live=lambda:(False,False),
                   _grid_ground=lambda:0.,paint_robot=lambda img,*a,**k:img,
                   _grid_overlay=lambda img,*a:img,
                   _grid_source=lambda:(None,'导航栅格',(),None,pose,0.,[],(layer,None),None))
        exec(compile(ast.Module(body=functions,type_ignores=[]),str(source),'exec'),scope)
        asyncio.run(scope['_draw_grid_async'](True))
        encoded=ui['map_image'].set_source.call_args.args[0]
        assert encoded.startswith('data:image/png;base64,')
        rgb=np.asarray(Image.open(io.BytesIO(base64.b64decode(encoded.split(',',1)[1]))))
        assert np.any(np.all(rgb==[116,151,182],axis=2)), 'occupied cells lost in grid-only frame'
