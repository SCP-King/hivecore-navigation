import threading,time
from types import SimpleNamespace as S
import numpy as np
from n01_live_navigation import rotation_sweep_job as j

P=np.array([[-.4,-.15],[.4,-.15],[.4,.15],[-.4,.15]])
def node():
    return S(pose=(0.,0.,0.),pose_z=.6,target_yaw=1.,rotation_footprint=S(get=lambda:P),
             rotation_plane=np.zeros(3),rotation_static=np.empty((0,3)),rotation_static_rx=time.monotonic(),
             rotation_raw=(),rotation_pose_cache=(),rotation_tf=None,
             get_clock=lambda:S(now=lambda:S(nanoseconds=10**10)))

def test_slow_sweep_does_not_block_control_and_proof_expires():
    n=node();entered=threading.Event();release=threading.Event()
    def decode(snapshot,with_stamp=False):
        entered.set();release.wait(2.)
        return np.tile([3.,0.,.3],(40,1)),10**10
    try:
        assert j.poll(n,time.monotonic(),decode) is None
        assert entered.wait(1.)
        assert j.poll(n,time.monotonic(),decode) is None
        release.set();n.rotation_sweep_future.result(timeout=2.)
        assert j.poll(n,time.monotonic(),decode) is True
        result=n.rotation_sweep_result
        assert not j.applicable(result,time.monotonic()+.4,10**10,(0.,0.,.6),0.,1.,n.rotation_plane,P)
        assert not j.applicable(result,time.monotonic(),10**10,(.1,0.,.6),0.,1.,n.rotation_plane,P)
        assert not j.applicable(result,time.monotonic(),10**10,(0.,0.,.6),0.,2.,n.rotation_plane,P)
    finally:
        release.set();n.rotation_sweep_pool.shutdown(wait=True)


def test_small_yaw_noise_and_equivalent_hull_vertices_use_checked_margin():
    now=time.monotonic()
    s=S(position=(0.,0.,.6),start_yaw=0.,target_yaw=1.,plane=np.zeros(3),footprint=P,static_rx=now)
    proof=dict(snapshot=s,clear=True,raw_stamp=10**10,finished=now)
    equivalent=np.insert(P,1,[-.2,-.15],axis=0)
    assert j.applicable(proof,now,10**10,(0.,0.,.6),-.001,1.,np.zeros(3),equivalent)
    assert not j.applicable(proof,now,10**10,(0.,0.,.6),-.2,1.,np.zeros(3),P)
