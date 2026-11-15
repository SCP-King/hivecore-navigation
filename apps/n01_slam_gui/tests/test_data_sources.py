from pathlib import Path
import sys
import pytest
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation_ui import inspect_bag, route_summary


def bag(tmp_path, raw_imu=20, filtered_imu=100):
    directory=tmp_path/'recording';directory.mkdir()
    (directory/'recording.db3').write_bytes(b'fixture')
    info={'relative_file_paths':['recording.db3'],'message_count':130,
          'duration':{'nanoseconds':10_000_000_000},
          'topics_with_message_count':[{'topic_metadata':{'name':name},'message_count':count}
              for name,count in [('/livox/lidar',10),('/livox/imu',raw_imu),('/imu/data',filtered_imu)]]}
    (directory/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':info}))
    return directory


def test_readonly_metadata_inspection_accepts_directory_or_metadata_and_flags_sparse_imu(tmp_path):
    directory=bag(tmp_path)
    before={p.name:p.read_bytes() for p in directory.iterdir()}
    result=inspect_bag(directory)
    assert result==inspect_bag(directory/'metadata.yaml')
    assert result['duration_s']==10 and result['message_count']==130
    assert '原始 IMU 记录明显少于滤波 IMU' in result['warnings'][0]
    assert before=={p.name:p.read_bytes() for p in directory.iterdir()}


def test_missing_recording_file_is_not_presented_as_available(tmp_path):
    directory=bag(tmp_path);(directory/'recording.db3').unlink()
    with pytest.raises(ValueError,match='数据文件缺失'):inspect_bag(directory)


def test_missing_raw_input_is_reported_without_claiming_fast_lio_replay_ready(tmp_path):
    result=inspect_bag(bag(tmp_path,raw_imu=0))
    assert '原始雷达或 IMU 缺失' in result['warnings'][0]


def test_malformed_yaml_has_a_user_readable_error(tmp_path):
    directory=bag(tmp_path);(directory/'metadata.yaml').write_text('bad: [')
    with pytest.raises(ValueError,match='元数据格式无效'):inspect_bag(directory)


def test_route_summary_distinguishes_starting_at_a_from_returning_after_b():
    route={'points':[{'name':'A','x':0.,'y':0.,'wait_s':1.},
                     {'name':'B','x':1.,'y':0.,'wait_s':1.}], 'return_to_start':False}
    text=route_summary(route)
    assert '先到 A' in text and 'A → B' in text and '1.00m' in text
    assert '1s / 1s' in text and '终点结束，不再返回' in text
    route['return_to_start']=True
    assert '结束后再回首点' in route_summary(route)
