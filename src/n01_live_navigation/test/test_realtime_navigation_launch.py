"""Evaluate realtime/offline node wiring without spawning ROS processes."""
import importlib.util
from pathlib import Path
import pytest

pytest.importorskip('launch')


@pytest.mark.parametrize('mode,topic', [('offline', ''), ('realtime', '/cloud_map'), ('realtime', '')])
def test_realtime_accumulated_map_is_separate_from_live_scan_input(monkeypatch, mode, topic):
    from launch import LaunchContext
    path = Path(__file__).resolve().parents[1]/'launch/live_scan.launch.py'
    spec = importlib.util.spec_from_file_location('test_live_launch', path)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module, '_guard_single_live_stack', lambda *_:None)
    monkeypatch.setattr(module, 'Node', lambda **kwargs:kwargs)
    context = LaunchContext()
    context.launch_configurations.update(dict(navigation_speed_mps='.03', pcd_path='/tmp/test.pcd',
        map_mode=mode, realtime_reference_topic=topic, pcd_map_frame_asserted='true',
        ground_z_map_m='-.8',ground_slope_x='0',ground_slope_y='0',body_height_m='.9',
        body_to_ground_m='.68',rviz_config='/tmp/test.rviz',start_localization_adapter='false',rviz='false'))
    nodes = module._nodes(context)
    live_reference = mode=='realtime' and bool(topic)
    relay = [n for n in nodes if n['name']=='n01_realtime_reference']
    assert bool(relay) == live_reference
    if relay:
        assert relay[0]['parameters'][0]['input_topic']=='/cloud_map'
    planner = next(n for n in nodes if n['name']=='scan_planner_node')['parameters'][-1]
    assert planner['grid_map.reference_updates_enabled'] == live_reference
    assert planner['grid_map.static_cloud_enabled'] == (mode=='offline' or live_reference)
    provider = next(n for n in nodes if n['name']=='n01_static_map')
    assert provider['condition'].evaluate(context) == (mode=='offline')
