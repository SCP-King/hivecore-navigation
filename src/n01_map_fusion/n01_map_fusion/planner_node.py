#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N-01 最小 A* 路径规划节点(离线验证)。

订阅融合栅格 /map/fusion/fused_grid;start/goal 由参数或 /planning/goal
(PoseStamped)给定;8 邻域 A*,仅 FREE=0 可通行,UNKNOWN/OCCUPIED 一律封闭。
输出 /plan/path(nav_msgs/Path,frame=map)与 /plan/result(std_msgs/String)。
没有控制器连接;不发布 /cmd_vel*、不发布 TF。
"""
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from .a_star_core import astar, nearest_free

FREE = np.int8(0)


class PlannerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('n01_path_planner', **kwargs)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 2.8)
        self.declare_parameter('goal_y', 0.2)
        self.declare_parameter('snap_radius', 1.5)
        # 隔离测试专用输入(默认关闭,不影响真机接口):
        #   start_topic 非空时,从该 PoseStamped 话题取模拟位姿作为规划起点。
        #   供阶段 C(运行中注入障碍)使用;不发布任何命令。
        self.declare_parameter('start_topic', '')
        self._latched_start = None      # (x, y) | None

        self._lock = threading.Lock()
        self._grid = None
        self._sx = 0
        self._sy = 0
        self._ox = 0.0
        self._oy = 0.0
        self._res = 0.10
        self._len = 0

        self._sub = self.create_subscription(
            OccupancyGrid, '/map/fusion/fused_grid', self._on_grid, 10)
        self._goal_sub = self.create_subscription(
            PoseStamped, '/planning/goal', self._on_goal, 10)
        self._pub_path = self.create_publisher(Path, '/plan/path', 10)
        self._pub_result = self.create_publisher(String, '/plan/result', 10)

        start_topic = self.get_parameter('start_topic').value
        if start_topic:
            self.create_subscription(PoseStamped, start_topic, self._on_start, 10)
            self.get_logger().info(f'start_topic={start_topic}: 规划起点将跟随模拟位姿')
        else:
            self.get_logger().info('start_topic 未设置: 规划起点固定用 start_x/start_y')

        self._publish(self.get_parameter('start_x').value, self.get_parameter('start_y').value,
                      self.get_parameter('goal_x').value, self.get_parameter('goal_y').value,
                      reason='param')

    def _on_start(self, msg):
        with self._lock:
            self._latched_start = (msg.pose.position.x, msg.pose.position.y)

    def _pub_from(self):
        """有效规划起点:模拟位姿(若配置了 start_topic 且已收到)→ 参数默认值。"""
        if self._latched_start is not None:
            return self._latched_start
        return (self.get_parameter('start_x').value,
                self.get_parameter('start_y').value)

    def _on_grid(self, msg):
        with self._lock:
            self._grid = np.frombuffer(msg.data, dtype=np.int8).copy()
            self._sx = msg.info.width
            self._sy = msg.info.height
            self._len = self._sx * self._sy
            self._ox = msg.info.origin.position.x
            self._oy = msg.info.origin.position.y
            self._res = msg.info.resolution
        self.get_logger().info(
            f'栅格已刷新 {self._sx}x{self._sy} ox={self._ox} oy={self._oy} res={self._res}')
        # 融合栅格每帧刷新即重规划(保持路径与动态层更新一致)
        sx, sy = self._pub_from()
        self._publish(sx, sy,
                      self.get_parameter('goal_x').value,
                      self.get_parameter('goal_y').value,
                      reason='grid_refresh')

    def _on_goal(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f'收到 /planning/goal ({x:.2f},{y:.2f})'
                               f'(group_id={msg.header.frame_id})')
        sx, sy = self._pub_from()
        self._publish(sx, sy, x, y, reason='topic')

    def _xy_to_col(self, x, y):
        ix = int(np.floor((x - self._ox) / self._res))
        iy = int(np.floor((y - self._oy) / self._res))
        if not (0 <= ix < self._sx and 0 <= iy < self._sy):
            return None
        return iy * self._sx + ix

    def _col_to_center(self, c):
        iy, ix = divmod(c, self._sx)
        return self._ox + (ix + 0.5) * self._res, self._oy + (iy + 0.5) * self._res

    def _publish(self, sx, sy, gx, gy, reason):
        with self._lock:
            if self._grid is None or self._len == 0:
                self._emit_result('no_grid_yet', [])
                return
            grid = self._grid
            sx_ = self._sx
            sy_ = self._sy
        sc = self._xy_to_col(sx, sy)
        gc = self._xy_to_col(gx, gy)
        snap_r = max(1, int(round(self.get_parameter('snap_radius').value / self._res)))
        s_free, sd = nearest_free(grid, sx_, sy_, sc, snap_r) if (sc is not None) else (None, None)
        g_free, gd = nearest_free(grid, sx_, sy_, gc, snap_r) if (gc is not None) else (None, None)
        if s_free is None or g_free is None:
            self._emit_result(f'start_or_goal_unreachable '
                              f'(start_free={s_free is not None}, goal_free={g_free is not None})', [])
            return
        path, stats = astar(grid, sx_, sy_, s_free, g_free)
        if path is None:
            self._emit_result(f'plan_failed:{stats["reason"]}', [])
            return
        msg = Path()
        msg.header.frame_id = 'map'
        for c in path:
            px, py = self._col_to_center(c)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            msg.poses.append(ps)
        self._pub_path.publish(msg)
        self._emit_result(
            f'ok start_snap={sd:.2f}m goal_snap={gd:.2f}m n={len(path)} '
            f'cost={stats["cost"]:.2f} expanded={stats["expanded"]} '
            f'partition={reason}', [])
        self.get_logger().info(
            f'路径规划成功: start_snap={sd:.2f}m goal_snap={gd:.2f}m n={len(path)} '
            f'cost={stats["cost"]:.2f}')

    def _emit_result(self, text, _):
        m = String()
        m.data = text
        self._pub_result.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
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