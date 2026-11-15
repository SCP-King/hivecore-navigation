"""Explicit task phases and bounded blocked-turn recovery, independent of ROS."""
from enum import Enum


class Phase(str,Enum):
    WAITING='WAITING_READY'
    TRACKING='TRACKING'
    TURNING='STOP_AND_TURN'
    REPLANNING='REPLANNING'
    VERIFYING='VERIFYING_ARRIVAL'
    ARRIVED='ARRIVED'
    FAILED='FAILED'
    MANUAL='MANUAL_OVERRIDE'


class MotionStateMachine:
    def __init__(self):
        self.phase=Phase.WAITING;self.entered=None;self.transitions=0

    def observe(self, status, now):
        if self.phase in (Phase.ARRIVED,Phase.FAILED,Phase.MANUAL):return self.phase
        if status=='ARRIVED':target=Phase.ARRIVED
        elif status in ('RECOVERY_FAILED','ROTATION_TIMEOUT','ROTATION_DRIFT','SESSION_COMPLETE','GOAL_CHANGED'):target=Phase.FAILED
        elif status=='MANUAL_OVERRIDE':target=Phase.MANUAL
        elif status=='MOVING':target=Phase.TRACKING
        elif status in ('ROTATING','ALIGN','ALIGN_PATH','STOPPING_FOR_TURN'):target=Phase.TURNING
        elif status in ('REPLANNING_AFTER_ROTATION_BLOCKED','NO_PATH','WAITING_FOR_REPLAN','WAITING_FOR_PATH_EXTENSION'):target=Phase.REPLANNING
        elif status=='VERIFYING_ARRIVAL':target=Phase.VERIFYING
        else:target=Phase.WAITING
        if self.entered is None or target!=self.phase:
            self.phase=target;self.entered=now;self.transitions+=1
        return self.phase
