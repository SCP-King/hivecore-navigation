from pathlib import Path
import pytest
import yaml
from n01_live_navigation.mola_pipeline import prepare_pipeline


def test_overlay_preserves_geometry_and_imu_and_does_not_modify_vendor(tmp_path):
    original='params:\n  imu_sensor_label: imu\n  other: "${KEEP|true}"\ninitial_localization:\n  fixed_initial_pose: [0, 0, 0, 0, 0, 0]\n'
    base=tmp_path/'vendor.yaml';base.write_text(original)
    overlay=tmp_path/'overlay.yaml';overlay.write_text('params:\n  max_time_to_wait_for_imu: 0.05\n  max_lidar_queue_before_drop: 2\n')
    target=prepare_pipeline(base,overlay,tmp_path/'cache')
    result=yaml.safe_load(Path(target).read_text())
    assert result['params']=={'imu_sensor_label':'imu','other':'${KEEP|true}',
        'max_time_to_wait_for_imu':.05,'max_lidar_queue_before_drop':2}
    assert result['initial_localization']['fixed_initial_pose']==[0]*6
    assert base.read_text()==original
    assert prepare_pipeline(base,overlay,tmp_path/'cache')==target


@pytest.mark.parametrize('wait,queue',[(0,2),(.5,2),(.05,15),(.05,True)])
def test_rejects_unbounded_or_host_incompatible_wait(tmp_path,wait,queue):
    base=tmp_path/'vendor.yaml';base.write_text('params: {}')
    overlay=tmp_path/'overlay.yaml'
    overlay.write_text(yaml.safe_dump({'params':{'max_time_to_wait_for_imu':wait,'max_lidar_queue_before_drop':queue}}))
    with pytest.raises(ValueError):prepare_pipeline(base,overlay,tmp_path/'cache')
