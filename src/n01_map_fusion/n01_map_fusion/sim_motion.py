#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立模拟位姿 + 运动模型局部运动闭环(Phase 5 ⑦)。

不靠 bag 回放,不连真机:本节点在隔离域内
  - 消费 /plan/path(nav_msgs/Path,map 系)—— A* 规划输出;
  - 维护模拟位姿 (x, y, theta),用单轮运动学模型按路径逐点跟踪;
  - "控制"仅计算并发布到隔离测试话题 /nav/test/cmd_vel;
  - 发布模拟位姿 /nav/test/pose(PoseStamped,map 系);
  - 订阅 /map/fusion/fused_grid 做碰撞检查(路径点落在 FREE=0 之外即判碰撞);
  - 输出 /nav/test/status(String):S0 路径就绪/S1 跟踪中/S2 已到达/S3 碰撞或不可达。

铁律:不发布 /cmd_vel*;不发布 TF;不启动真机。仅隔离域(ROS_DOMAIN_ID=88)。
"""
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header, String

FREE = np.int8(0)

CMD_TOPIC = '/nav/test/cmd_vel'
POSE_TOPIC = '/nav/test/pose'
STATUS_TOPIC = '/nav/test/status'


class SimMotion(Node):
    def __init__(self, **kwargs):
        super().__init__('n01_sim_motion', **kwargs)
        self.declare_parameter('start_x', 11.05)
        self.declare_parameter('start_y', 7.95)
        self.declare_parameter('start_yaw', math.pi / 2)   # 朝 +Y(路径主要沿 y 下降)
        self.declare_parameter('max_speed', 0.4)           # m/s
        self.declare_parameter('max_yaw_rate', 1.2)        # rad/s
        self.declare_parameter('waypoint_tol', 0.12)       # m 视为到达路径点
        self.declare_parameter('goal_tol', 0.15)           # m 视为到达终点
        self.declare_parameter('rate', 20.0)               # Hz 控制/位姿积分

        self._x = float(self.get_parameter('start_x').value)
        self._y = float(self.get_parameter('start_y').value)
        self._yaw = float(self.get_parameter('start_yaw').value)
        self._vmax = float(self.get_parameter('max_speed').value)
        self._wmax = float(self.get_parameter('max_yaw_rate').value)
        self._wtol = float(self.get_parameter('waypoint_tol').value)
        self._gtol = float(self.get_parameter('goal_tol').value)
        self._dt = 1.0 / float(self.get_parameter('rate').value)

        self._lock = threading.Lock()
        self._path = []          # [(x,y)] map 系路径点序列
        self._wp_idx = 0
        self._goal = None
        self._grid = None        # fused 栅格引用
        self._grid_meta = None
        self._arrived = False
        self._collided = False
        self._started = False

        self._sub_path = self.create_subscription(Path, '/plan/path', self._on_path, 10)
        self._sub_grid = self.create_subscription(
            OccupancyGrid, '/map/fusion/fused_grid', self._on_grid, 10)
        self._pub_cmd = self.create_publisher(Twist, CMD_TOPIC, 10)
        self._pub_pose = self.create_publisher(PoseStamped, POSE_TOPIC, 10)
        self._pub_status = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_timer(self._dt, self._tick)
        self._emit_status('S0-init')

    # ---------- 输入 ----------
    def _on_path(self, msg):
        with self._lock:
            new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            old_goal = self._goal
            self._path = new_path
            # S3 碰撞锁存必须优先于 S2:碰撞时 _arrived 也为真,
            # 若先检查 S2 会把锁存失败误报为 S2-arrived。
            if self._collided:
                self._emit_status('S3-collision') # 重申失败态
                return
            # S2 终态锁存:已到达且新路径目标不变 → 保持 S2,不因 2Hz 刷新重新进入 S1
            if (self._arrived and old_goal is not None and new_path and
                    abs(new_path[-1][0] - old_goal[0]) < 1e-6 and
                    abs(new_path[-1][1] - old_goal[1]) < 1e-6):
                self._emit_status('S2-arrived')   # 重申终态,不下转 S1
                return
            # 重规划(path 刷新)不打断跟踪:goal 未变时从离当前位置最近的 waypoint 续走。
            # (static_map 2Hz 周期重发 → fusion/planner 每 0.5s 都会重规划一次。
            #  阶段 C 注入障碍后,新路径起点=当前模拟位姿,同样从当前位置续走。)
            if (self._started and old_goal is not None and new_path and
                    abs(new_path[-1][0] - old_goal[0]) < 1e-6 and
                    abs(new_path[-1][1] - old_goal[1]) < 1e-6):
                best_i, best_d = 0, float('inf')
                for i, (wx, wy) in enumerate(new_path):
                    d = math.hypot(wx - self._x, wy - self._y)
                    if d < best_d:
                        best_d, best_i = d, i
                self._wp_idx = best_i
            else:
                self._wp_idx = 0
            self._arrived = False
            if new_path:
                self._goal = new_path[-1]
                self._started = True
            else:
                self._goal = None
        n = len(self._path)
        g = self._goal
        self.get_logger().info(f'收到路径 n={n}{f" goal=({g[0]:.2f},{g[1]:.2f})" if g else ""}')
        if n:
            self._emit_status('S1-tracking')

    def _on_grid(self, msg):
        with self._lock:
            self._grid = np.frombuffer(msg.data, dtype=np.int8).copy()
            self._grid_meta = dict(
                ox=msg.info.origin.position.x, oy=msg.info.origin.position.y,
                res=msg.info.resolution, sx=msg.info.width, sy=msg.info.height)

    # ---------- 碰撞检查 ----------
    def _point_ok(self, x, y):
        if self._grid is None or self._grid_meta is None:
            return True  # 无栅格时不误判
        gm = self._grid_meta
        ix = int(math.floor((x - gm['ox']) / gm['res']))
        iy = int(math.floor((y - gm['oy']) / gm['res']))
        if not (0 <= ix < gm['sx'] and 0 <= iy < gm['sy']):
            return False  # 出界=不可行
        return self._grid[iy * gm['sx'] + ix] == FREE

    # ---------- 控制:单轮运动学,路径点跟踪 ----------
    def _tick(self):
        with self._lock:
            if not self._started or self._arrived or self._collided:
                if not self._started:
                    pass
                return
            path = self._path
            if not path:
                # 规划器发了空路径(start/goal 被堵至不可达):不误报 S2
                self._emit_status('S3-unreachable')
                self._arrived = True
                return
            if self._wp_idx >= len(path):
                self._arrived = True
                self._emit_status('S2-arrived')
                return
            tx, ty = path[self._wp_idx]
            # 越过当前路径点(距离足够近)
            dx, dy = tx - self._x, ty - self._y
            dist = math.hypot(dx, dy)
            if dist <= self._wtol:
                self._wp_idx += 1
                if self._wp_idx >= len(path):
                    self._arrived = True
                    self._emit_status('S2-arrived')
                    return
                tx, ty = path[self._wp_idx]
                dx, dy = tx - self._x, ty - self._y
                dist = math.hypot(dx, dy)
            # 速度:按剩余距离收尾,防超调
            v = min(self._vmax, dist * 4.0)
            if dist > 1e-9:
                vx, vy = v * dx / dist, v * dy / dist
            else:
                vx, vy = 0.0, 0.0
            # 航向:跟踪目标方向(仅供位姿可视化;速度已按 (vx,vy) 直接给出)
            if dist > 1e-9:
                self._yaw = math.atan2(dy, dx)
            # 运动学积分(全向平移动力学:模拟底盘可沿任意方向移动)
            self._x += vx * self._dt
            self._y += vy * self._dt
            # 碰撞检查:新位姿必须落在 FREE
            if not self._point_ok(self._x, self._y):
                self._arrived = True
                self._collided = True
                self._emit_status('S3-collision')
                return
            # 到达终点判定
            gx, gy = self._goal
            if math.hypot(gx - self._x, gy - self._y) <= self._gtol:
                self._arrived = True
                self._emit_status('S2-arrived')
        # 发布隔离控制话题(不碰 /cmd_vel*)
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = 0.0
        self._pub_cmd.publish(msg)

        pose = PoseStamped()
        pose.header = Header(frame_id='map')
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self._x)
        pose.pose.position.y = float(self._y)
        pose.pose.orientation.z = float(math.sin(self._yaw / 2.0))
        pose.pose.orientation.w = float(math.cos(self._yaw / 2.0))
        self._pub_pose.publish(pose)

    def _emit_status(self, s):
        m = String()
        m.data = s
        self._pub_status.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = SimMotion()
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