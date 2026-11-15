"""Shared operator task vocabulary; raw safety/permission checks stay separate."""
from dataclasses import dataclass

TERMINAL={'ARRIVED','FAILED','CANCELLED'}
LABELS={'IDLE':'未开始','PLANNING':'规划中','READY':'路径已就绪','STARTING':'启动中',
        'RUNNING':'行驶中','DEGRADED':'降速行驶','PAUSED':'暂停／等待恢复',
        'VERIFYING':'到达停车确认中','ARRIVED':'已到达','FAILED':'执行失败',
        'CANCELLED':'已取消','MANUAL':'手动已接管'}

def execution_task_state(status,degraded=False):
    if status=='ARRIVED':return 'ARRIVED'
    if status in {'RECOVERY_FAILED','ROTATION_TIMEOUT','ROTATION_DRIFT','SESSION_COMPLETE','GOAL_CHANGED'}:return 'FAILED'
    if status=='MANUAL_OVERRIDE':return 'MANUAL'
    if status=='MOVING':return 'DEGRADED' if degraded else 'RUNNING'
    if status in {'ROTATING','ROTATION_CHECKING','STOPPING_FOR_TURN','ALIGN_PATH'}:return 'RUNNING'
    if status=='VERIFYING_ARRIVAL':return 'VERIFYING'
    if status in {'DISABLED','WAITING_COMMAND_OBSERVER'}:return 'STARTING'
    return 'PAUSED'

@dataclass
class NavigationTaskView:
    phase: str='IDLE'
    goal: object=None
    task_id: str=''
    reason: str=''
    last_rx: float=0.
    executing: bool=False

    def begin_goal(self,goal):
        self.goal=goal;self.task_id='';self.phase='PLANNING'
        self.reason='等待本次目标路径';self.executing=False;self.last_rx=0.

    def begin_execution(self,task_id,now):
        self.task_id=task_id;self.phase='STARTING';self.executing=True
        self.reason='等待执行器反馈';self.last_rx=now

    def observe_execution(self,data,now):
        if not self.task_id or data.get('task_id')!=self.task_id:return False
        if self.phase in TERMINAL or self.phase=='MANUAL':return False
        self.last_rx=now
        self.phase=execution_task_state(data.get('state'),bool(data.get('navigation_degraded') or data.get('steering_degraded')))
        self.reason=str(data.get('reason') or '')
        return True

    def observe_plan(self,can_start,path_valid,failed=False,running=True,reason=''):
        if self.executing or self.phase in TERMINAL or self.phase=='MANUAL':return
        if self.goal is None:self.phase='IDLE';return
        if failed:self.phase='FAILED'
        elif can_start:self.phase='READY'
        elif self.phase=='READY' and path_valid and running:pass
        else:self.phase='PLANNING'
        self.reason=reason

    def finish(self,phase,reason):
        self.phase=phase;self.reason=reason

    def cancel(self):
        if self.phase not in TERMINAL:self.phase='CANCELLED';self.reason='用户停止'

    def check_feedback(self,now):
        if self.executing and self.phase not in TERMINAL|{'MANUAL'} and now-self.last_rx>3.:
            self.phase='PAUSED';self.reason='执行反馈中断，运动状态待确认'

    @property
    def label(self):return LABELS[self.phase]
