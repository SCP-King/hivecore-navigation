import numpy as np
from n01_live_navigation.dynamic_obstacles import ObstacleTracks


def cluster(x):
    xx,yy=np.meshgrid(np.linspace(x,x+.3,4),np.linspace(0,.3,4))
    return np.column_stack((xx.ravel(),yy.ravel(),np.full(xx.size,.5)))


def test_stationary_clusters_do_not_get_motion_predictions():
    t=ObstacleTracks()
    for i in range(10):assert not len(t.update(cluster(0),1.+i*.1))
    assert t.stats['dynamic_tracks']==0


def test_moving_cluster_predicts_forward_and_expires_on_gap():
    t=ObstacleTracks();out=None
    for i in range(9):out=t.update(cluster(.5*i*.1),1.+i*.1)
    assert len(out)>0 and t.stats['dynamic_tracks']==1
    assert out[:,0].max()>cluster(.4)[:,0].max()+.2
    np.testing.assert_allclose(t.update(cluster(.4),1.8),out) # duplicate doesn't erase predictions
    assert not len(t.update(cluster(2),4.))


def test_reordered_scan_cannot_replace_recent_motion_state():
    t=ObstacleTracks();t.update(cluster(0),2.)
    assert not len(t.update(cluster(1),1.)) and t.stamp==2.
