#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""隔离域模拟动态障碍(Phase 5 ⑦:注入动态障碍验证重规划/拒绝)。

只向 /grid_map/occupancy 发布合成的 2D 圆盘障碍(地面已过滤语义,
map 系 PointCloud2 xyz)。融合节点收到后更新 dynamic 层→ fused_grid,
规划节点收到新栅格自动重规划。

安全:只运行在 ROS_DOMAIN_ID=88 + ROS_LOCALHOST_ONLY=1 的隔离域;
不发布 /cmd_vel*;不发布 TF;不启动真机。
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

TOPIC = '/grid_map/occupancy'


class MockObstacle(Node):
    def __init__(self, **kwargs):
        super().__init__('n01_mock_obstacle', **kwargs)
        self.declare_parameter('center_x', 11.05)
        self.declare_parameter('center_y', 7.20)   # 路径中点,垂直路径段
        self.declare_parameter('radius', 0.4)      # m
        self.declare_parameter('sampling', 0.05)   # m 栅格化密度
        self.declare_parameter('rate', 2.0)        # Hz
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_after_s', 3.0)  # 延迟注入(等路径先出)

        self._cx = float(self.get_parameter('center_x').value)
        self._cy = float(self.get_parameter('center_y').value)
        self._r = float(self.get_parameter('radius').value)
        self._s = float(self.get_parameter('sampling').value)
        self._frame = str(self.get_parameter('frame_id').value)
        self._delay = float(self.get_parameter('publish_after_s').value)

        n = math.ceil(self._r / self._s)
        xs, ys = np.meshgrid(np.arange(-n, n + 1) * self._s,
                             np.arange(-n, n + 1) * self._s)
        d2 = xs ** 2 + ys ** 2
        inside = d2 <= (self._r + 1e-9) ** 2
        self._cloud = np.column_stack(
            [self._cx + xs[inside], self._cy + ys[inside],
             np.full(int(inside.sum()), 0.2, dtype=np.float64)]).astype('<f4')

        self._pub = self.create_publisher(PointCloud2, TOPIC, 10)
        self._clock0 = self.get_clock().now()
        self._started = False
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.get_logger().info(
            f'模拟动态障碍: 中心({self._cx},{self._cy}) r={self._r}m '
            f'sampling={self._s}m 点={self._cloud.shape[0]} 延迟 {self._delay}s '
            f'→ {TOPIC} (隔离测试话题环境)')

    def _tick(self):
        if (self.get_clock().now() - self._clock0).nanoseconds / 1e9 < self._delay:
            return
        if not self._started:
            self._started = True
            self.get_logger().info('动态障碍注入开始 (持续发布)')
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.height = 1
        msg.width = self._cloud.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        msg.data = self._cloud.tobytes()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockObstacle()
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