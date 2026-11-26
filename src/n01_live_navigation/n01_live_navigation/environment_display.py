"""RViz surfaces only. No map, path, TF, or actuator authority."""
import copy
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from .core import GROUND_CLEARANCE_M, decode_pointcloud_xyz
from .environment_geometry import contour_geometry, ground_grid


def obstacle_cloud(xyz, header, ground, ground_clearance=GROUND_CLEARANCE_M):
    """Keep measured points between the floor margin and ceiling cutoff."""
    a, b, c = ground
    height = xyz[:, 2] - (a * xyz[:, 0] + b * xyz[:, 1] + c)
    points = np.asarray(xyz[np.isfinite(xyz).all(axis=1) & (height > ground_clearance) & (height <= 2.2)],
                        dtype='<f4')
    return PointCloud2(header=header, height=1, width=len(points), is_dense=True,
                       point_step=12, row_step=12 * len(points), data=points.tobytes(),
                       fields=[PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                               for i, name in enumerate(('x','y','z'))])


def display_markers(geometry, header, reference=False):
    markers=[]
    for identifier,kind,key in [(0,Marker.TRIANGLE_LIST,'triangles'),(1,Marker.LINE_LIST,'outlines')]:
        marker=Marker(header=copy.deepcopy(header), ns='reference' if reference else 'live', id=identifier,
                      type=kind, action=Marker.ADD)
        marker.pose.orientation.w=1.
        marker.scale.x=marker.scale.y=marker.scale.z=1. if identifier==0 else .018
        if reference:
            marker.color.r,marker.color.g,marker.color.b,marker.color.a=(.30,.46,.63,.60) if identifier==0 else (.48,.65,.80,.85)
        else:
            marker.color.r,marker.color.g,marker.color.b,marker.color.a=(.15,.79,.69,1.) if identifier==0 else (.68,1.,.89,1.)
            marker.lifetime.sec=1;marker.lifetime.nanosec=500_000_000
        marker.points=[Point(x=x,y=y,z=z) for x,y,z in geometry[key].tolist()]
        if not marker.points:
            marker.action=Marker.DELETE
        markers.append(marker)
    return MarkerArray(markers=markers)


class EnvironmentDisplay(Node):
    def __init__(self):
        super().__init__('n01_environment_display')
        self.ground=tuple(float(self.declare_parameter(name,default).value) for name,default in
                          [('ground_slope_x',0.),('ground_slope_y',0.),('ground_z_map_m',0.)])
        self.ground_clearance = float(self.declare_parameter('display_ground_clearance_m', GROUND_CLEARANCE_M).value)
        if not .03 <= self.ground_clearance <= .15:
            raise ValueError('display_ground_clearance_m outside [0.03, 0.15]')
        self.resolution=float(self.declare_parameter('resolution_m',.06).value)
        self.radius=float(self.declare_parameter('radius_m',12.).value)
        if not (.04<=self.resolution<=.3 and 2<=self.radius<=20 and np.isfinite(self.ground).all()):
            raise ValueError('invalid_environment_display_config')
        retained=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference_pub=self.create_publisher(MarkerArray,'/n01/navigation/reference_surfaces',retained)
        self.live_pub=self.create_publisher(MarkerArray,'/n01/navigation/live_surfaces',qos_profile_sensor_data)
        self.reference_points_pub = self.create_publisher(
            PointCloud2, '/n01/navigation/reference_obstacle_points', retained)
        self.live_points_pub = self.create_publisher(
            PointCloud2, '/n01/navigation/live_obstacle_points', qos_profile_sensor_data)
        self.grid_pub=self.create_publisher(Marker,'/n01/navigation/environment_grid',retained)
        self.status_pub=self.create_publisher(String,'/n01/navigation/environment_display_status',10)
        self.reference=None;self.live=None;self.center=(0.,0.)
        self.reference_key=None;self.live_key=None;self.grid_center=None
        self.live_sub=None;self.scan_sub=None;self.last_scan_stamp_ns=0
        self.live_visible=False;self.last_reference_time=0.
        self.reference_sub=self.create_subscription(PointCloud2,'/grid_map/static_occupancy',lambda m:setattr(self,'reference',m),retained)
        self.pose_sub=self.create_subscription(Odometry,'/body_pose',self.on_pose,qos_profile_sensor_data)
        self.create_subscription(String, '/n01/ground_reference/status', self.on_ground_reference, 10)
        self.create_timer(.5,self.update)

    def on_ground_reference(self, message):
        try:
            status = json.loads(message.data)
            if status['state'] != 'ground_projection_ready':
                return
            plane = status['plane']
            ground = tuple(float(plane[k]) for k in ('a', 'b', 'c'))
            residual = float(plane['residual'])
        except (KeyError, ValueError, TypeError):
            return
        if (np.isfinite(ground).all() and 0 <= residual <= .04
                and np.hypot(*ground[:2]) <= np.tan(np.deg2rad(2.))):
            if ground != self.ground:
                self.ground = ground
                self.reference_key = self.live_key = self.grid_center = None

    def on_pose(self, message):
        p=message.pose.pose.position
        if message.header.frame_id=='map' and np.isfinite([p.x,p.y]).all():
            self.center=(p.x,p.y)

    def clear_live(self):
        if self.live_visible:
            self.live_pub.publish(MarkerArray(markers=[Marker(ns='live',id=i,action=Marker.DELETE) for i in (0,1)]))
            self.live_points_pub.publish(obstacle_cloud(np.empty((0, 3)), self.live.header, self.ground))
            self.live_visible=False

    def on_live_cloud(self, message):
        self.live=message
        self.on_scan(message)

    def on_scan(self, message):
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if message.header.frame_id=='map' and message.width*message.height>0:
            self.last_scan_stamp_ns=max(self.last_scan_stamp_ns,stamp)

    def update(self):
        now_ns=self.get_clock().now().nanoseconds
        # Do not cause the planner to export a large occupancy cloud when no
        # viewer is displaying it. Only one newest cloud is retained.
        if self.live_pub.get_subscription_count() or self.live_points_pub.get_subscription_count():
            if self.live_sub is None:
                self.live_sub=self.create_subscription(PointCloud2,'/cloud',self.on_live_cloud,qos_profile_sensor_data)
        elif self.live_sub is not None:
            self.destroy_subscription(self.live_sub);self.live_sub=None
            if self.scan_sub is not None:
                self.destroy_subscription(self.scan_sub);self.scan_sub=None
            self.last_scan_stamp_ns=0
            self.live=None;self.live_key=None;self.live_visible=False
        grid_center=tuple(np.floor(self.center).astype(int))
        if self.grid_pub.get_subscription_count()>0 and self.grid_center!=grid_center:
            grid=Marker(ns='ground',id=0,type=Marker.LINE_LIST,action=Marker.ADD)
            grid.header.frame_id='map';grid.header.stamp=self.get_clock().now().to_msg()
            grid.pose.orientation.w=1.;grid.scale.x=.008
            grid.color.r,grid.color.g,grid.color.b,grid.color.a=.36,.43,.53,.42
            grid.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in ground_grid(self.ground,self.center,self.radius)]
            self.grid_pub.publish(grid);self.grid_center=grid_center
        for reference in (True,False):
            pub=self.reference_pub if reference else self.live_pub
            message=self.reference if reference else self.live
            points_pub = self.reference_points_pub if reference else self.live_points_pub
            show_surfaces = pub.get_subscription_count() > 0
            show_points = points_pub.get_subscription_count() > 0
            if not (show_surfaces or show_points) or message is None:
                continue
            stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
            age=(now_ns-stamp)/1e9
            scan_age=(now_ns-self.last_scan_stamp_ns)/1e9
            if (message.header.frame_id!='map' or stamp<=0 or
                (not reference and (not -.2<=age<=1.0 or not -.2<=scan_age<=1.0))):
                if not reference:self.clear_live()
                continue
            key=(stamp, grid_center, show_surfaces, show_points)
            if key==(self.reference_key if reference else self.live_key):
                continue
            if reference and time.monotonic()-self.last_reference_time<1.:
                continue
            started=time.perf_counter()
            try:
                xyz = (decode_pointcloud_xyz(message, 1000000, exclude_ground=not reference)
                       if message.width * message.height else np.empty((0, 3)))
                geometry = (contour_geometry(xyz,self.ground,self.center,self.radius,self.resolution)
                            if show_surfaces else None)
                if not reference:
                    publish_now=self.get_clock().now().nanoseconds
                    if (publish_now-stamp)/1e9>1.0 or (publish_now-self.last_scan_stamp_ns)/1e9>1.0:
                        self.clear_live()
                        continue
                if show_points:
                    points_pub.publish(obstacle_cloud(xyz, message.header, self.ground, self.ground_clearance))
                if show_surfaces:
                    pub.publish(display_markers(geometry, message.header, reference))
                if reference:
                    self.reference_key=key;self.last_reference_time=time.monotonic()
                else:
                    self.live_key=key;self.live_visible=True
                elapsed=(time.perf_counter()-started)*1000
                self.status_pub.publish(String(data=json.dumps(dict(layer='reference' if reference else 'live',
                    faces=geometry['faces'] if geometry else 0,
                    voxels=geometry['voxels'] if geometry else 0, resolution_m=self.resolution,
                    source_scan_age_s=None if reference else round(scan_age,3),
                    build_and_submit_ms=round(elapsed,2),display_only=True))))
            except (ValueError,OverflowError) as error:
                if not reference:self.clear_live()
                self.get_logger().warning('Environment display: '+str(error),throttle_duration_sec=5.)


def main(args=None):
    rclpy.init(args=args)
    node=EnvironmentDisplay()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
