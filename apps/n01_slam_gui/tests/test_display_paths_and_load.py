"""GUI display regressions; no ROS processes or physical motion."""
import ast
import math
import queue
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from render_policy import model_visual_key, path_visual_key

TREE = ast.parse((ROOT / 'slam_lite_gui.py').read_text())


def extract(name, scope, cls=None):
    body = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == cls).body if cls else TREE.body
    fn = next(n for n in body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == name)
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(ROOT / 'slam_lite_gui.py'), 'exec'), scope)
    return scope[name]


def test_full_reference_remains_visible_beyond_local_prefix():
    colors = dict(grid=(10, 10, 10), global_plan=(55, 125, 255), plan=(35, 220, 220))
    scope = dict(ImageDraw=ImageDraw, math=math, GRID_IMAGE_PX=100, GRID_COLORS=colors,
                 _map_xy_to_pixel=lambda x, y, view: (x, y))
    overlay = extract('_grid_overlay', scope)
    image = overlay(Image.new('RGB', (100, 100)), dict(x0=0, y0=0, size_m=1),
                    None, [], None, [(10, 50), (30, 50)], True, False,
                    [(10, 50), (90, 50)], True, False)
    assert image.getpixel((20, 50)) == colors['plan']
    assert image.getpixel((80, 50)) == colors['global_plan']


def test_global_and_execution_topics_are_distinct_and_correct():
    constants = {n.targets[0].id: ast.literal_eval(n.value) for n in TREE.body
                 if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
                 and n.targets[0].id in ('GLOBAL_PATH_TOPIC', 'LOCAL_PATH_TOPIC')}
    assert constants == dict(GLOBAL_PATH_TOPIC='/planning/global_path', LOCAL_PATH_TOPIC='/n01/navigation/path')


def test_path_heartbeats_refresh_freshness_without_invalidating_geometry():
    for name, prefix, field in [('_on_global_path', 'global', 'global_path'), ('_on_plan_path', 'plan', 'plan_path')]:
        state = SimpleNamespace(lock=threading.Lock(), plan_goal_ns=0, goal_target=None, recv=Mock(),
                                **{prefix+'_pending': False, prefix+'_seq': 0})
        scope = dict(STATE=state, time=time, math=math, RosPath=object, path_visual_key=path_visual_key)
        callback = extract(name, scope, 'RosWorker')
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0), frame_id='map'),
                              poses=[SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0))) for x in (0, 1)])
        callback(None, msg)
        msg.header.stamp.sec = 2
        callback(None, msg)
        assert getattr(state, prefix+'_seq') == 1
        assert state.recv.call_count == 2
        state.recv.assert_called_with(field)
        msg.poses[-1].pose.position.x = 1.1
        callback(None, msg)
        assert getattr(state, prefix+'_seq') == 2


def test_log_bound_counts_lines_instead_of_eighty_line_chunks():
    log = Mock()
    class UI(dict):
        available = True
    state = SimpleNamespace(log_q=queue.Queue())
    for i in range(80):
        state.log_q.put(str(i))
    drain = extract('drain_logs', dict(UI=UI(log=log), STATE=state, queue=queue, drain_ui_events=Mock()))
    drain()
    assert [c.args[0] for c in log.push.call_args_list] == [str(i) for i in range(80)]


def test_model_cache_tracks_geometry_not_heartbeat_time():
    model = object()
    key = model_visual_key(model, {'wheel': .1}, (0, 0))
    assert key == model_visual_key(model, {'wheel': .1}, (0, 0))
    assert key != model_visual_key(model, {'wheel': .2}, (0, 0))
    assert key != model_visual_key(model, {}, (0, 0))  # stale joints invalidate projection
    assert key != model_visual_key(model, {'wheel': .1}, (.1, 0))
