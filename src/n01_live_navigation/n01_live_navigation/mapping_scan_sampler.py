"""Bound SLAM registration input; navigation continues using full-rate scans."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class ScanSchedule:
    """Capture-time schedule: tolerate jitter, skip missed slots, never queue scans."""
    def __init__(self, period_s, max_age_s=.25):
        self.period_ns=int(period_s*1e9)
        self.jitter_ns=min(2_000_000,self.period_ns//20)
        self.max_age_s=max_age_s
        self.next_ns=0
        self.last_ns=0

    def accept(self, stamp_ns, now_ns):
        if (stamp_ns<=self.last_ns or not -.1<=(now_ns-stamp_ns)/1e9<=self.max_age_s
                or (self.next_ns and stamp_ns+self.jitter_ns<self.next_ns)):
            return False
        if not self.next_ns:
            self.next_ns=stamp_ns+self.period_ns
        else:
            missed=max(1,(stamp_ns+self.jitter_ns-self.next_ns)//self.period_ns+1)
            self.next_ns+=missed*self.period_ns
        self.last_ns=stamp_ns
        return True


class MappingScanSampler(Node):
    def __init__(self):
        super().__init__('n01_mapping_scan_sampler')
        rate=float(self.declare_parameter('max_rate_hz',5.).value)
        if not 1<=rate<=10:raise ValueError('mapping scan rate outside [1,10] Hz')
        self.schedule=ScanSchedule(1./rate);self.count=0;self.received=0
        self.pub=self.create_publisher(PointCloud2,'/n01/slam/mapping_scan',
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(10.,self.report_rate)
        self.sub=self.create_subscription(PointCloud2,'/livox/lidar/geo_filtered',self.on_cloud,qos_profile_sensor_data)

    def report_rate(self):
        self.get_logger().info(f"mapping_scan_counts received={self.received} forwarded={self.count}")

    def on_cloud(self,message):
        self.received+=1
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if (message.header.frame_id!='lidar_link' or message.width*message.height<=0
            or self.pub.get_subscription_count()==0
            or not self.schedule.accept(stamp,self.get_clock().now().nanoseconds)):
            return
        self.pub.publish(message)  # Original acquisition time/fields/extrinsics.
        self.count+=1


def main(args=None):
    rclpy.init(args=args);node=MappingScanSampler()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == "__main__":
    main()
