"""Preserve stamped integration samples across short compute bursts.

Age and gap checks remain at consumers; queue capacity is not a freshness permit.
"""
from rclpy.qos import QoSProfile, ReliabilityPolicy

def odometry_integration_qos():
    return QoSProfile(depth=64,reliability=ReliabilityPolicy.BEST_EFFORT)
