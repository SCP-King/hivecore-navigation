"""Mapping session readiness, source time, and map reference boundaries."""
from types import SimpleNamespace as S
from unittest.mock import Mock

import pytest
from n01_live_navigation.core import mapping_navigation_status, localization_is_tracking


def status(session='mapping', stamp=10_000_000_000, now=10_100_000_000,
           pose=10_000_000_000, rx=.1):
    return mapping_navigation_status(session, stamp, now, pose, rx)


def test_mapping_ready_without_fabricating_offline_localization_fields():
    values = status()
    assert localization_is_tracking(values) == (True, 'mapping_ready')
    assert not {'map_loaded', 'reloc_ok', 'reloc_fitness'} & values.keys()


@pytest.mark.parametrize('changes', [dict(session='idle'), dict(session='tracking'),
    dict(stamp=0), dict(stamp=1_000_000_000), dict(pose=0), dict(pose=4_000_000_000),
    dict(pose=11_000_000_000), dict(rx=6.)])
def test_mapping_session_or_stale_pose_cannot_authorize_navigation(changes):
    assert not localization_is_tracking(status(**changes))[0]


def test_delayed_mapping_pose_keeps_session_ready_for_independently_bounded_bridge():
    values=status(pose=8_600_000_000,rx=1.2)
    assert values['mapping_pose_fresh']=='false'
    assert values['mapping_pose_available']=='true'
    assert values['mapping_pose_stamp_ns']=='8600000000'
    assert localization_is_tracking(values)==(True,'mapping_ready')


def test_mapping_branch_does_not_relax_offline_status_contract():
    assert not localization_is_tracking({'session': 'mapping'})[0]
    assert not localization_is_tracking({'session': 'tracking', 'map_loaded': 'true'})[0]


def cloud(stamp=10, frame='map'):
    return S(header=S(stamp=S(sec=stamp, nanosec=0), frame_id=frame),
             width=2, height=1, point_step=12, row_step=24,
             fields=[S(name=name) for name in ('x', 'y', 'z')], data=bytes(24))


def test_reference_preserves_message_time_and_rejects_wrong_frame_old_or_truncated_map():
    pytest.importorskip('rclpy')
    from n01_live_navigation.realtime_reference import reference_rejection, RealtimeReference
    msg = cloud()
    fake = S(last_stamp_ns=0, get_clock=lambda: S(now=lambda:S(nanoseconds=11_000_000_000)),
             pub=S(publish=Mock()), get_logger=lambda:S(warning=Mock()))
    RealtimeReference.on_map(fake, msg)
    fake.pub.publish.assert_called_once_with(msg)
    assert fake.last_stamp_ns == 10_000_000_000
    assert msg.header.stamp.sec == 10
    RealtimeReference.on_map(fake, msg)
    assert fake.pub.publish.call_count == 1
    assert reference_rejection(cloud(frame='lidar_link'), 11_000_000_000, 0)
    assert reference_rejection(cloud(), 20_000_000_000, 0)
    msg.data = b''
    assert reference_rejection(msg, 11_000_000_000, 0) == 'reference_data_truncated'


def test_mapping_adapter_emits_separate_status_and_keeps_original_source_stamp():
    pytest.importorskip('rclpy')
    from n01_live_navigation.localization_adapter import N01LocalizationAdapter
    from hivecore_slam3d_lite_mapping.msg import MappingStatus
    msg = MappingStatus(session='mapping')
    msg.header.stamp.sec = 10
    import time
    fake = S(status_counts={'received':0, 'published':0}, last_output_stamp_ns=10_000_000_000,
             last_output_receive_time=time.monotonic(), stamp_ns=N01LocalizationAdapter.stamp_ns,
             get_clock=lambda:S(now=lambda:S(nanoseconds=10_100_000_000)),
             status_pub=S(publish=Mock()))
    N01LocalizationAdapter.on_mapping_status(fake, msg)
    out = fake.status_pub.publish.call_args.args[0]
    assert out.header.stamp.sec == 10
    values = {v.key:v.value for v in out.status[0].values}
    assert localization_is_tracking(values)[0]
    assert 'reloc_ok' not in values
