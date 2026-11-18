from types import SimpleNamespace as S
import numpy as np
from n01_live_navigation.trajectory_monitor import Hive2TrajectoryMonitor as Monitor


def test_different_scene_versions_never_mix_obstacles_and_coverage():
    old=np.array([[0.,0.,0.]])
    n=S(require_observed_space=True,obstacles=old,observed_free=old,obstacles_stamp_ns=10,observed_free_stamp=10)
    obstacle=np.array([[1.,0.,0.]]);coverage=np.array([[2.,0.,0.]])
    Monitor.stage_scene(n,'occupied',11,obstacle)
    assert n.obstacles_stamp_ns==10 and n.observed_free_stamp==10
    Monitor.stage_scene(n,'free',12,coverage)
    assert n.obstacles_stamp_ns==10
    Monitor.stage_scene(n,'free',11,coverage)
    assert n.obstacles_stamp_ns==n.observed_free_stamp==11
    assert n.obstacles is obstacle and n.observed_free is coverage
    Monitor.stage_scene(n,'occupied',10,old)
    assert n.obstacles_stamp_ns==11
    for stamp in range(20,30):Monitor.stage_scene(n,'free',stamp,coverage)
    assert len(n.pending_scenes)==3


def test_offline_scene_does_not_wait_for_unused_coverage_topic():
    n=S(require_observed_space=False,obstacles_stamp_ns=0)
    p=np.zeros((0,3));Monitor.stage_scene(n,'occupied',5,p)
    assert n.obstacles_stamp_ns==5 and n.obstacles is p
