import threading
from types import SimpleNamespace
from n01_live_navigation.executor_scheduling import FeedbackScheduling


def test_busy_perception_does_not_hold_feedback_group_or_state_lock():
    s=FeedbackScheduling();entered=threading.Event();release=threading.Event();received=[]
    PointCloud2=type('PointCloud2',(),{})
    def slow(message):entered.set();release.wait(2.)
    perception,pg=s.subscription('/grid_map/occupancy',PointCloud2,slow)
    feedback,fg=s.subscription('/odom',object,received.append)
    t=threading.Thread(target=perception,args=(None,));t.start()
    try:
        assert entered.wait(1.)
        assert fg is s.feedback and pg is s.perception and fg is not pg
        feedback('fresh')
        assert received==['fresh'] and t.is_alive()
    finally:release.set();t.join()


def test_critical_feedback_commit_is_serialized_with_control():
    s=FeedbackScheduling();called=threading.Event();started=threading.Event()
    callback,_=s.subscription('/estop',object,lambda _:called.set())
    def invoke():started.set();callback(True)
    with s.lock:
        t=threading.Thread(target=invoke);t.start()
        assert started.wait(1.) and not called.is_set()
    t.join(1.)
    assert called.is_set()
