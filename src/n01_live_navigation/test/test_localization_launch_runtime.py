"""Exercise the real delayed launch actions, with process creation stubbed out."""
import importlib.util
from pathlib import Path
import pytest

pytest.importorskip('launch')
pytest.importorskip('ament_index_python')


@pytest.mark.parametrize('mode,map_path', [('localization', ''),
    ('localization', '/tmp/explicit_localization_map.pcd'), ('mapping', '')])
def test_delayed_localization_keeps_arguments_and_isolates_only_its_tf(monkeypatch, mode, map_path, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    from launch import LaunchDescription, LaunchService
    from launch.actions import TimerAction, OpaqueFunction, EmitEvent, SetLaunchConfiguration
    from launch.events import Shutdown
    import launch_ros.actions
    from ament_index_python.packages import get_package_share_directory
    try:
        get_package_share_directory('hivecore_slam3d_lite_console')
        get_package_share_directory('n01_live_navigation')
    except Exception:
        pytest.skip('vendor ROS underlay not sourced')
    recorded=[]

    def node_stub(**kwargs):
        def visit(context):
            recorded.append((kwargs, dict(context.launch_configurations)))
            return []
        return OpaqueFunction(function=visit)

    monkeypatch.setattr(launch_ros.actions,'Node',node_stub)
    path=Path(__file__).resolve().parents[1]/'launch/n01_localization_stack.launch.py'
    spec=importlib.util.spec_from_file_location('localization_launch_under_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    warmup=[]
    monkeypatch.setattr(module,'wait_for_mapping_inputs',lambda:warmup.append(True))
    service=LaunchService()
    service.include_launch_description(LaunchDescription([
        SetLaunchConfiguration('map_path',map_path),
        SetLaunchConfiguration('slam_mode',mode),
        module.generate_launch_description(),
        TimerAction(period=2.6,actions=[EmitEvent(event=Shutdown(reason='test complete'))]),
    ]))
    assert service.run()==0
    assert bool(warmup)==(mode=='mapping')
    loc=next(kwargs for kwargs,_ in recorded if kwargs.get('name')==mode+'_lite_node')
    overrides=loc['parameters'][1]
    assert overrides['auto_activate_'+mode] is True
    assert overrides['use_sim_time'] is False
    assert overrides.get('map_path','')==map_path
    import yaml
    pipeline=yaml.safe_load(Path(overrides['mola_pipeline_yaml']).read_text())
    assert pipeline['params']['max_time_to_wait_for_imu']==.05
    assert pipeline['params']['max_lidar_queue_before_drop']==2
    assert loc['remappings']==[('/tf','/n01/vendor_'+mode+'/tf_raw')]
    adapter=next((kwargs,context) for kwargs,context in recorded if kwargs.get('name')=='n01_localization_adapter')
    assert not adapter[0].get('remappings')
    config = yaml.safe_load(Path(adapter[0]['parameters'][0]).read_text())['n01_localization_adapter']['ros__parameters']
    if mode == 'mapping':
        assert overrides['input_scan_topic'] == '/n01/slam/mapping_scan'
        sampler = next(kwargs for kwargs, _ in recorded
                       if kwargs.get('name') == 'n01_mapping_scan_sampler')
        assert sampler['parameters'][0]['max_rate_hz'] == 5.0
        mapping_config = yaml.safe_load(Path(loc['parameters'][0]).read_text())['/**']['ros__parameters']
        assert mapping_config['enable_imu_fusion'] is False
        assert config['status_mode'] == 'mapping'
        assert config['status_input_topic'] == '/mapping_status'
    assert sum(kwargs.get('name')=='n01_live_scan_bridge' for kwargs,_ in recorded)==1
    assert ('/tf','/n01/vendor_localization/tf_raw') not in adapter[1].get('ros_remaps',[])
    assert not any(kwargs.get('package') in ('livox_ros_driver2','robot_state_publisher') for kwargs,_ in recorded)
