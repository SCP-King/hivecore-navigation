from types import SimpleNamespace as S
from unittest.mock import Mock
from n01_live_navigation.mapping_scan_sampler import ScanSchedule, MappingScanSampler


def test_schedule_tolerates_capture_jitter_without_reducing_five_hz_to_three():
    schedule=ScanSchedule(.2)
    accepted=[]
    # Alternating 1ms timestamp jitter around 10Hz should preserve every other scan.
    for i in range(100):
        stamp=1_000_000_000+i*100_000_000+(1_000_000 if i%4==0 else 0)
        if schedule.accept(stamp,stamp+110_000_000):accepted.append(stamp)
    assert len(accepted)==50
    assert max(b-a for a,b in zip(accepted,accepted[1:]))<=201_000_000


def test_schedule_rejects_old_future_and_duplicate_scans_and_skips_missed_slots():
    schedule=ScanSchedule(.2)
    assert schedule.accept(1_000_000_000,1_100_000_000)
    assert not schedule.accept(1_100_000_000,1_200_000_000)
    assert not schedule.accept(1_200_000_000,2_000_000_000)
    assert not schedule.accept(2_000_000_000,1_300_000_000)
    assert schedule.accept(2_900_000_000,3_000_000_000)
    assert not schedule.accept(2_900_000_000,3_000_000_000)
    assert not schedule.accept(2_910_000_000,3_000_000_000)
    assert schedule.accept(3_000_000_000,3_100_000_000)


def test_forwards_original_cloud_without_retiming():
    fake=S(schedule=ScanSchedule(.2),count=0,received=0,pub=S(get_subscription_count=lambda:1,publish=Mock()),
           get_clock=lambda:S(now=lambda:S(nanoseconds=1_100_000_000)))
    msg=S(header=S(frame_id='lidar_link',stamp=S(sec=1,nanosec=0)),width=100,height=1)
    MappingScanSampler.on_cloud(fake,msg)
    fake.pub.publish.assert_called_once_with(msg)
    assert fake.schedule.last_ns==1_000_000_000 and msg.header.stamp.sec==1
