import threading,time
from types import SimpleNamespace as S
from concurrent.futures import Future
from n01_live_navigation import maneuver_search_job as j
from n01_live_navigation.maneuver_planner import Maneuver
from n01_live_navigation.heading_search import SearchResult

class Scene:
    radius=.06
    def free(self,states):return True

def result():
    return Maneuver(SearchResult([(i*.02,0.,0.,0.)for i in range(51)],'forward_heading_path',10,100.),[], 'forward_heading_path')

def args():
    return ((0.,0.,0.,0.),(1.,0.),Scene(),None,S(segment_allowed=lambda s:True))

def test_search_job_is_nonblocking_and_rechecked_on_return(monkeypatch):
    entered=threading.Event();release=threading.Event()
    def search(*a,**kw):entered.set();release.wait(2.);return result()
    monkeypatch.setattr(j,'plan_maneuver',search)
    n=S()
    try:
        assert j.poll_maneuver(n,1,*args()) is None
        assert entered.wait(1.)
        assert not n.maneuver_future.done()
        assert j.poll_maneuver(n,1,*args()) is None
        release.set();n.maneuver_future.result(timeout=2.)
        assert j.poll_maneuver(n,1,*args()).forward.states
    finally:
        release.set();n.maneuver_pool.shutdown(wait=True)

def test_old_goal_and_changed_obstacle_cannot_publish_snapshot_result():
    for changed_goal in (True,False):
        f=Future();f.set_result(result());n=S(maneuver_future=f,maneuver_future_goal=1)
        a=list(args())
        if not changed_goal:a[2].free=lambda s:False
        assert j.poll_maneuver(n,2 if changed_goal else 1,*a) is None


def test_retry_budget_grows_on_same_goal_and_resets_for_new_goal(monkeypatch):
    budgets=[]
    def search(*a,**kw):
        budgets.append(kw['budget_s'])
        return Maneuver(SearchResult([],'heading_search_time_budget',1,1.),[],'heading_search_time_budget')
    monkeypatch.setattr(j,'plan_maneuver',search)
    n=S()
    try:
        for goal in (1,1,1,1,2):
            assert j.poll_maneuver(n,goal,*args()) is None
            n.maneuver_future.result(timeout=1.)
            assert not j.poll_maneuver(n,goal,*args()).forward.states
        assert budgets==[2.,4.,8.,8.,2.]
    finally:n.maneuver_pool.shutdown(wait=True)
