"""Display sampled SCAN trajectory only; never authorizes motion or splices paths."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scan_planner_msgs.msg import NavigationTrajectory,ExecutionProgress

class TrajectoryDisplay(Node):
    def __init__(self):
        super().__init__('n01_trajectory_display')
        self.controller_rx=0.
        self.pub=self.create_publisher(Path,'/n01/navigation/path',QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(ExecutionProgress,'/n01/navigation/progress',self.progress,10)
        self.create_subscription(NavigationTrajectory,'/n01/navigation/trajectory',self.trajectory,10)
    def progress(self,msg):self.controller_rx=time.monotonic()
    def trajectory(self,msg):
        if time.monotonic()-self.controller_rx<.6:return
        path=Path();path.header=msg.header
        if msg.valid:
            for p in msg.samples:
                pose=PoseStamped();pose.header=msg.header;pose.pose=p;path.poses.append(pose)
        self.pub.publish(path)

def main(args=None):
    rclpy.init(args=args);node=TrajectoryDisplay()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
