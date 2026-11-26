"""Display sampled SCAN trajectory only; never authorizes motion or splices paths."""
import copy
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PolygonStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from scan_planner_msgs.msg import NavigationTrajectory,ExecutionProgress

class TrajectoryDisplay(Node):
    def __init__(self):
        super().__init__('n01_trajectory_display')
        self.controller_rx=0.
        self.pub=self.create_publisher(Path,'/n01/navigation/path',QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.ground = tuple(self.declare_parameter(name, 0.).value for name in
                            ('ground_slope_x', 'ground_slope_y', 'ground_z_map_m'))
        self.goal = None
        self.footprint = None
        for name, topic, qos in (
            ('global', '/planning/global_path', retained),
            ('local', '/n01/live_navigation/hive2_candidate_path', 10),
            ('execution', '/n01/navigation/path', retained),
        ):
            publisher = self.create_publisher(Path, f'/n01/navigation/rviz/{name}_path', retained)
            self.create_subscription(Path, topic,
                                     lambda msg, pub=publisher: pub.publish(self.project_path(msg)), qos)
        self.goal_pub = self.create_publisher(MarkerArray, '/n01/navigation/rviz/goal', retained)
        self.create_subscription(PoseStamped, '/n01/live_navigation/goal_gated', self.on_goal, 10)
        self.create_subscription(PolygonStamped, '/n01/robot_footprint', self.on_footprint, retained)
        self.create_subscription(ExecutionProgress,'/n01/navigation/progress',self.progress,10)
        self.create_subscription(NavigationTrajectory,'/n01/navigation/trajectory',self.trajectory,10)
    def height(self, x, y):
        a, b, c = self.ground
        return a*x+b*y+c+.035

    def project_path(self, message):
        path = copy.deepcopy(message)
        for item in path.poses:
            p = item.pose.position
            p.z = self.height(p.x, p.y)
        return path

    def on_goal(self, message):
        if message.header.frame_id == 'map':
            self.goal = message
            self.publish_goal()

    def on_footprint(self, message):
        if message.header.frame_id == 'base_link':
            self.footprint = message.polygon.points
            self.publish_goal()

    def publish_goal(self):
        if self.goal is None:
            return
        pose = self.goal.pose
        p, q = pose.position, pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        heading = Marker(header=copy.deepcopy(self.goal.header), ns='goal', id=0, type=Marker.ARROW)
        heading.header.stamp.sec = heading.header.stamp.nanosec = 0
        heading.pose.orientation.w = 1.
        heading.scale.x, heading.scale.y, heading.scale.z = .045, .12, .16
        heading.color.r, heading.color.g, heading.color.b, heading.color.a = .9, .4, 1., .95
        dx, dy = math.cos(yaw), math.sin(yaw)
        heading.points = [Point(x=x, y=y, z=self.height(x,y)) for x,y in
                          [(p.x,p.y),(p.x+.65*dx,p.y+.65*dy)]]
        hull = Marker(header=heading.header, ns='goal', id=1, type=Marker.LINE_STRIP)
        hull.pose.orientation.w = 1.
        hull.scale.x = .025
        hull.color = heading.color
        vertices = list(self.footprint or ())
        if len(vertices) >= 3:
            for v in vertices + vertices[:1]:
                x, y = p.x+dx*v.x-dy*v.y, p.y+dy*v.x+dx*v.y
                hull.points.append(Point(x=x,y=y,z=self.height(x,y)))
        else:
            hull.action = Marker.DELETE
        self.goal_pub.publish(MarkerArray(markers=[heading,hull]))

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
