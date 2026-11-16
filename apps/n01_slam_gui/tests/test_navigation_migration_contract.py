"""Production ownership and session binding, without launching GUI or hardware."""
import ast,sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teach_routes import map_identity,waypoint,validate,SCHEMA
ROOT=Path(__file__).resolve().parents[3]


def test_live_routes_are_bound_to_slam_session_not_current_pose(tmp_path):
    geometry=tmp_path/'g.json';geometry.write_text('{}')
    identity=map_identity('session://'+'a'*32,geometry)
    route=dict(schema=SCHEMA,name='live',map=identity,points=[waypoint('A',0,0,0,0),waypoint('B',1,0,0,0)],return_to_start=False)
    validate(route,identity)
    with pytest.raises(ValueError,match='地图'):validate(route,map_identity('session://'+'b'*32,geometry))
    geometry.write_text('{"version":2}')
    with pytest.raises(ValueError,match='地图'):validate(route,map_identity('session://'+'a'*32,geometry))


def test_production_launch_has_no_second_planner_or_shadow_follower():
    source=(ROOT/'src/n01_live_navigation/launch/live_scan.launch.py').read_text()
    assert 'executable="n01_shadow_path_follower"'not in source
    assert 'executable="n01_navigation_path"'not in source
    assert 'executable="n01_trajectory_display"'in source
    monitor=(ROOT/'src/n01_live_navigation/n01_live_navigation/trajectory_monitor.py').read_text()
    for forbidden in ('heading_search','maneuver_review','heading_candidate'):
        assert forbidden not in monitor
    assert 'NavigationTrajectory'in monitor


def test_gui_has_no_separate_rotation_process_or_motion_publisher():
    source=(ROOT/'apps/n01_slam_gui/slam_lite_gui.py').read_text();tree=ast.parse(source)
    assert '_align_route_station'not in source and 'def start_rotation'not in source
    assert 'n01/navigation/execution_status'in source
    assert 'payload["goal_id"]'in source # command identity preserved across GUI queue
    for node in ast.walk(tree):
        if isinstance(node,ast.Call)and isinstance(node.func,ast.Attribute)and node.func.attr=='create_publisher':
            assert not(node.args and isinstance(node.args[0],ast.Name)and node.args[0].id=='Twist')


def test_fast_lio_live_route_uses_its_actual_tracking_status():
    from types import SimpleNamespace as S
    from threading import Lock
    source=(ROOT/'apps/n01_slam_gui/slam_lite_gui.py').read_text()
    function=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef)and n.name=='_mapping_navigation_ready')
    state=S(lock=Lock(),loc={'session':'tracking','reloc_ok':True},age_of=lambda k:0.)
    scope={'STATE':state,'PROCS':S(desk_mode='fast_lio_realtime',running=lambda k:True)}
    exec(compile(ast.Module(body=[function],type_ignores=[]),'live_route_status','exec'),scope)
    assert scope['_mapping_navigation_ready']()
    state.loc['reloc_ok']=False
    assert not scope['_mapping_navigation_ready']()
