#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N-01 地图融合节点:融合静态占用(全3D,含地面)与实时占用(地面已过滤),
合成 map 系 2D OccupancyGrid(-1/0/100),按安全半径膨胀。

无 TF 依赖;不发布 TF;不发布 /cmd_vel*。输出话题:
  /map/fusion/static_grid   静态层 OccupancyGrid(诊断)
  /map/fusion/dynamic_grid  动态层 OccupancyGrid(诊断)
  /map/fusion/fused_grid    融合+膨胀 OccupancyGrid(A* 输入)
  /map/fusion/stats         统计 String(来源区分定量)
"""
import array
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, String

from .fusion_core import FusionGrid, FREE, OCCUPIED, UNKNOWN
from .cloud_xyz import pc2_to_xyz

STATIC_TOPIC = '/grid_map/static_occupancy'
DYNAMIC_TOPIC = '/grid_map/occupancy'
OUT_STATIC = '/map/fusion/static_grid'
OUT_DYNAMIC = '/map/fusion/dynamic_grid'
OUT_FUSED = '/map/fusion/fused_grid'
OUT_STATS = '/map/fusion/stats'


class FusionNode(Node):
    def __init__(self, **kwargs):
        super().__init__('n01_map_fusion', **kwargs)
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('origin_x', -20.0)
        self.declare_parameter('origin_y', -18.0)
        self.declare_parameter('size_x', 65.0)
        self.declare_parameter('size_y', 56.0)
        self.declare_parameter('robot_h', 1.5)
        self.declare_parameter('floor_max', 0.5)
        self.declare_parameter('obs_bottom', 0.15)
        self.declare_parameter('safety_radius', 0.35)
        self.declare_parameter('min_band_points', 1)

        self.fg = FusionGrid(
            origin_x=self.get_parameter('origin_x').value,
            origin_y=self.get_parameter('origin_y').value,
            size_x=self.get_parameter('size_x').value,
            size_y=self.get_parameter('size_y').value,
            resolution=self.get_parameter('resolution').value,
            robot_h=self.get_parameter('robot_h').value,
            floor_max=self.get_parameter('floor_max').value,
            obs_bottom=self.get_parameter('obs_bottom').value,
            safety_radius=self.get_parameter('safety_radius').value,
            min_band_points=self.get_parameter('min_band_points').value,
        )
        self._lock = threading.Lock()
        self._header = Header()
        self._header.frame_id = 'map'
        self._static_seen = False

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._stat_sub = self.create_subscription(
            PointCloud2, STATIC_TOPIC, self._on_static, qos)
        self._dyn_sub = self.create_subscription(
            PointCloud2, DYNAMIC_TOPIC, self._on_dynamic, qos)

        self._pub_static = self.create_publisher(OccupancyGrid, OUT_STATIC, 10)
        self._pub_dynamic = self.create_publisher(OccupancyGrid, OUT_DYNAMIC, 10)
        self._pub_fused = self.create_publisher(OccupancyGrid, OUT_FUSED, 10)
        self._pub_stats = self.create_publisher(String, OUT_STATS, 10)

        self.get_logger().info(
            f'融合节点就绪: grid={self.fg.sx}x{self.fg.sy} res={self.fg.resolution} '
            f'robot_h={self.fg.robot_h} floor_max={self.fg.floor_max} '
            f'safety={self.fg.safety_radius}m min_band={self.fg.min_band_points} '
            f'订阅 {STATIC_TOPIC} {DYNAMIC_TOPIC}')

    def _occupancy_msg(self, grid1d, label):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._header.stamp
        msg.info.resolution = self.fg.resolution
        msg.info.width = self.fg.sx
        msg.info.height = self.fg.sy
        msg.info.origin.position.x = self.fg.ox
        msg.info.origin.position.y = self.fg.oy
        msg.info.origin.position.z = 0.0
        msg.data = array.array('b', grid1d.astype(np.int8).ravel().tolist())
        return msg

    def _stamp(self, msg):
        self._header.stamp = msg.header.stamp

    def _on_static(self, msg):
        self._stamp(msg)
        pts = pc2_to_xyz(msg)
        with self._lock:
            self.fg.classify_static(pts)
            self._static_seen = True
            self.fg.fuse()
            self._publish_locked()

    def _on_dynamic(self, msg):
        self._stamp(msg)
        pts = pc2_to_xyz(msg)
        with self._lock:
            self.fg.set_dynamic(pts)
            if not self._static_seen:
                return  # 静态层未到前不发(避免把纯动态图当融合)
            self.fg.fuse()
            self._publish_locked()

    def _publish_locked(self):
        self._pub_static.publish(self._occupancy_msg(self.fg.static, 'static'))
        self._pub_dynamic.publish(
            self._occupancy_msg(np.where(self.fg.dynamic_occ, OCCUPIED, UNKNOWN),
                                'dynamic'))
        self._pub_fused.publish(self._occupancy_msg(self.fg.fused, 'fused'))
        s = self.fg.log
        stat = String()
        stat.data = ('stamp=%d.%d static_occ=%d dynamic_occ=%d free_floor=%d '
                     'unknown=%d ground=%d inflated=%d inflate_cells=%d' % (
                         self._header.stamp.sec, self._header.stamp.nanosec,
                         s.get('static_occ', 0), int(self.fg.dynamic_occ.sum()),
                         s.get('free', 0), s.get('unknown', 0),
                         s.get('ground', 0), s.get('inflated', 0),
                         s.get('inflate_cells', 0)))
        self._pub_stats.publish(stat)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()