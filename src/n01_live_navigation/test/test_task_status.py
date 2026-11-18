from n01_live_navigation.task_status import NavigationTaskView,execution_task_state


def test_prepared_path_survives_health_flap_but_not_actual_path_loss():
    t=NavigationTaskView();t.begin_goal(1)
    t.observe_plan(True,True);assert t.phase=='READY'
    t.observe_plan(False,True,reason='runtime check');assert t.phase=='READY'
    t.observe_plan(False,False);assert t.phase=='PLANNING'


def test_execution_owns_task_state_instead_of_start_readiness():
    t=NavigationTaskView();t.begin_goal(1);t.begin_execution('a',0.)
    t.observe_execution(dict(task_id='a',state='MOVING'),1.)
    t.observe_plan(False,False);assert t.phase=='RUNNING'
    t.observe_execution(dict(task_id='a',state='MOVING',navigation_degraded=True),2.)
    assert t.phase=='DEGRADED'
    t.observe_execution(dict(task_id='a',state='NO_PATH'),3.)
    assert t.phase=='PAUSED'
    t.observe_execution(dict(task_id='a',state='MOVING'),4.)
    assert t.phase=='RUNNING'


def test_arrival_is_terminal_and_old_tasks_cannot_advance_new_goal():
    t=NavigationTaskView();t.begin_goal(1);t.begin_execution('a',0.)
    t.observe_execution(dict(task_id='a',state='ARRIVED'),1.)
    assert not t.observe_execution(dict(task_id='a',state='NO_PATH'),2.)
    t.observe_plan(False,False,failed=True);t.cancel();assert t.phase=='ARRIVED'
    t.begin_goal(2)
    assert not t.observe_execution(dict(task_id='a',state='ARRIVED'),3.)
    t.begin_execution('b',4.)
    assert not t.observe_execution(dict(task_id='a',state='MOVING'),5.)


def test_missing_feedback_is_unknown_not_assumed_moving_or_arrived():
    t=NavigationTaskView();t.begin_execution('a',0.)
    t.observe_execution(dict(task_id='a',state='MOVING'),1.)
    t.check_feedback(5.)
    assert t.phase=='PAUSED' and '待确认'in t.reason
    assert execution_task_state('NOT_READY')=='PAUSED'
    assert execution_task_state('ROTATING')=='RUNNING'


def test_waiting_for_native_path_exposes_only_matching_goal_planning_reason():
    from n01_live_navigation.task_status import planning_wait_description
    status=dict(goal_id=2,phase='WAITING_FOR_OBSERVATION')
    assert '雷达覆盖'in planning_wait_description(status,2,'waiting_for_native_scan_trajectory')
    assert '本次目标'in planning_wait_description(status,3,'waiting_for_native_scan_trajectory')
    assert planning_wait_description(status,2,'estop')=='estop'
