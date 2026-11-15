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
    fn = next(n for n in ast.parse(source.read_text()).body
              if isinstance(n, ast.FunctionDef) and n.name == '_render_grid_frame')
    scope = dict(np=np, Image=Image, io=io, base64=base64, GRID_IMAGE_PX=64,
                 _rasterize=lambda *args: (np.zeros((10, 10, 3), dtype=np.uint8), .03, 1.6),
                 paint_robot=lambda img, *args, **kwargs: img,
                 _grid_overlay=lambda img, *args: img)
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(source), 'exec'), scope)
    data, low, high = scope['_render_grid_frame'](
        np.array([[0., 0., .1]]), {}, 0., None, (0., 0., 0.), 0.,
        None, None, [], None, [], None, [], False, False, [], False, True)
    image = Image.open(io.BytesIO(base64.b64decode(data.split(',', 1)[1])))
    assert image.size == (64, 64) and low == .03 and high == 1.6
