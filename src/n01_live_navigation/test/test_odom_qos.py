from collections import deque
from rclpy.qos import ReliabilityPolicy
from n01_live_navigation.odom_qos import odometry_integration_qos
from n01_live_navigation.angular_travel import AngularTravel

def test_short_compute_burst_does_not_discard_angular_integration_samples():
    q=odometry_integration_qos()
    assert q.reliability==ReliabilityPolicy.BEST_EFFORT
    def segments(depth):
        queue=deque(maxlen=depth);travel=AngularTravel()
        travel.update(1_000_000_000,.08)
        for i in range(1,11):queue.append((1_000_000_000+i*20_000_000,.08))
        for stamp,rate in queue:travel.update(stamp,rate)
        return travel.segment,travel.angle
    assert segments(5)[0]==1
    assert segments(q.depth)[0]==0
    assert abs(segments(q.depth)[1]-.016)<1e-9
