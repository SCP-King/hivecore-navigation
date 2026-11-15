"""Keep DDS feedback schedulable while perception callbacks are busy."""
import threading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

FAST_TOPICS=frozenset(('/odom','/body_pose','/joint_states','/estop',
    '/chassis_dec_stop/state','/n01/live_navigation/system_health',
    '/n01/robot_footprint','/n01/slow_motion/enable','/n01/live_navigation/scan_body_pose'))

class FeedbackScheduling:
    def __init__(self):
        self.lock=threading.RLock()
        self.feedback=MutuallyExclusiveCallbackGroup()
        self.perception=MutuallyExclusiveCallbackGroup()

    def subscription(self,topic,msg_type,callback):
        if topic in FAST_TOPICS:
            def synchronized(message):
                with self.lock:callback(message)
            return synchronized,self.feedback
        if getattr(msg_type,'__name__','')=='PointCloud2':
            return callback,self.perception
        return callback,None
