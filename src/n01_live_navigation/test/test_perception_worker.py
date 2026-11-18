"""Exercise production scheduling with a blocked CPU job, no ROS graph or motion."""
from collections import deque,defaultdict
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace as S,MethodType
from threading import Event,get_ident
import time
from sensor_msgs.msg import PointCloud2
from n01_live_navigation.live_scan_bridge import LiveScanBridge


def message(ns):
    m=PointCloud2();m.header.frame_id='lidar';m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9
    return m


def test_cpu_work_does_not_block_ros_callbacks_or_build_a_fifo_backlog():
    release=Event();started=Event();calls=[];committed=[];main=get_ident()
    def convert(msg,*args):
        calls.append(msg.header.stamp.nanosec);started.set();assert release.wait(2.);return msg
    transform=S(transform=S(translation=S(x=0.,y=0.,z=0.),rotation=S(x=0.,y=0.,z=0.,w=1.)))
    worker=ThreadPoolExecutor(max_workers=1)
    n=S(cloud_executor=worker,cloud_future=None,cloud_context=None,cloud_work_stamp_ns=0,
        pending_clouds=deque([(message(10**9),time.monotonic(),False)]),
        get_clock=lambda:S(now=lambda:S(nanoseconds=1200000000)),_status_is_fresh=lambda:True,
        max_cloud_age=.9,max_tf_wait=.55,body_frame='base_link',cloud_frame='lidar',
        _pose_at=lambda _:([0.,0.,0.],[0.,0.,0.,1.]),tf_buffer=S(lookup_transform=lambda *args,**kwargs:transform),
        _transform_values=LiveScanBridge._transform_values,scene_perception=S(snapshot=lambda _:('model',{})),
        last_motion_compensation={},_xyz_map_cloud=convert,
        _finish_cloud=lambda ctx,output:committed.append((ctx.stamp_ns,get_ident())),
        _stamp_ns=lambda s:s.sec*10**9+s.nanosec,last_processed_scan_stamp_ns=0,counts=defaultdict(int))
    try:
        before=time.monotonic();LiveScanBridge.process_pending_clouds(n)
        assert time.monotonic()-before<.1 and started.wait(.5)
        # These incoming callbacks run while the converter is still blocked.
        LiveScanBridge.on_cloud(n,message(1100000000))
        LiveScanBridge.on_cloud(n,message(1150000000))
        LiveScanBridge.process_pending_clouds(n)
        assert len(n.pending_clouds)==1 and n.pending_clouds[0][0].header.stamp.nanosec==150000000
        assert len(calls)==1 and not committed
        release.set();deadline=time.monotonic()+1.
        while len(committed)<2 and time.monotonic()<deadline:
            LiveScanBridge.process_pending_clouds(n);time.sleep(.002)
        assert [stamp for stamp,_ in committed]==[10**9,1150000000]
        assert all(thread==main for _,thread in committed)
        assert calls==[0,150000000] # superseded frame never gets CPU work
    finally:release.set();worker.shutdown(wait=True)


def test_completed_scan_keeps_original_pose_stamp_and_compensation_context():
    from unittest.mock import Mock
    body=object();counts=defaultdict(int)
    n=S(get_clock=lambda:S(now=lambda:S(nanoseconds=1200000000)),last_cloud_input_stamp_ns=1100000000,
        max_cloud_age=.9,max_latest_scan_lag=.7,_status_is_fresh=lambda:True,scene_perception=S(body=body),
        counts=counts,last_body=None,_odom=LiveScanBridge._odom,map_frame='map',body_frame='base_link',cloud_frame='lidar',
        sensor_pose_pub=Mock(),scan_body_pose_pub=Mock(),cloud_pub=Mock(),max_tf_wait_observed_s=0.,
        last_motion_compensation={'scan_stamp_ns':1100000000})
    ctx=S(msg=message(10**9),stamp_ns=10**9,snapshot=(body,{}),body=([0.,0.,.7],[0.,0.,0.,1.]),
        sensor=([.295,0.,.85],[0.,0.,0.,1.]),waited_s=.01,wait_counted=True,
        compensation={'scan_stamp_ns':10**9,'anchor_stamp_ns':900000000})
    result=message(10**9);LiveScanBridge._finish_cloud(n,ctx,result)
    assert counts['cloud_out']==1 and n.last_processed_scan_stamp_ns==10**9
    assert n.scan_body_pose_pub.publish.call_args.args[0].header.stamp==ctx.msg.header.stamp
    assert n.sensor_pose_pub.publish.call_args.args[0].pose.pose.position.x==.295
    assert n.last_output_compensation['scan_stamp_ns']==10**9
    assert n.cloud_pub.publish.call_args.args[0]is result


def test_model_changed_during_cpu_job_cannot_publish_old_filtered_scene():
    n=S(get_clock=lambda:S(now=lambda:S(nanoseconds=1200000000)),last_cloud_input_stamp_ns=1100000000,
        max_cloud_age=.9,max_latest_scan_lag=.7,_status_is_fresh=lambda:True,scene_perception=S(body=object()),counts=defaultdict(int))
    context=S(msg=message(10**9),stamp_ns=10**9,snapshot=(object(),{}))
    LiveScanBridge._finish_cloud(n,context,message(10**9))
    assert n.counts['drop_stale_after_wait']==1
