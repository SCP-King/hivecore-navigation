#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""隔离域最小闭环采集器(只订阅,不发布)。

记录 /plan/result、/nav/test/status、/plan/path 点数与头部点、
/map/fusion/stats,带 wall-clock 时间戳,写 JSON 报告。
只读订阅;不参与任何控制。
"""
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# 与 fusion_core 同语义:FREE=0, OCCUPIED=100, UNKNOWN=-1
FREE = np.int8(0)
OCC = np.int8(100)
UNK = np.int8(-1)

BEST_EFFORT = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)


class Collector(Node):
    def __init__(self, out, quit_after):
        super().__init__('n01_loop_collector')
        self._out = out
        self._quit_after = float(quit_after)
        self._start_wall = time.time()
        self._events = []
        self._fused_n = 0
        self._poses = []            # (t, x, y, yaw) 模拟位姿序列
        self._dyn_state = 0         # 最近一次 fusion_stats 的 dynamic_occ
        self._grid_before = None    # 注入前最后一帧 fused grid 摘要
        self._grid_after = None     # 注入后第一帧 fused grid 摘要
        self.create_subscription(String, '/plan/result', self._on_result, 10)
        self.create_subscription(String, '/nav/test/status', self._on_status, 10)
        self.create_subscription(Path, '/plan/path', self._on_path, 10)
        self.create_subscription(String, '/map/fusion/stats', self._on_stats, 10)
        self.create_subscription(OccupancyGrid, '/map/fusion/fused_grid',
                                 self._on_fused, BEST_EFFORT)
        self.create_subscription(PoseStamped, '/nav/test/pose', self._on_pose, BEST_EFFORT)
        self.create_timer(1.0, self._tick)

    def _wall(self):
        return round(time.time() - self._start_wall, 2)

    def _push(self, kind, data):
        self._events.append({'t': self._wall(), 'kind': kind, 'data': data})

    def _on_result(self, msg):
        self._push('plan_result', msg.data)
        self.get_logger().info(f'plan_result: {msg.data}')

    def _on_status(self, msg):
        self._push('sim_status', msg.data)
        self.get_logger().info(f'sim_status: {msg.data}')

    def _on_path(self, msg):
        n = len(msg.poses)
        head = [(round(p.pose.position.x, 2), round(p.pose.position.y, 2))
                for p in msg.poses[:4]]
        tail = [(round(p.pose.position.x, 2), round(p.pose.position.y, 2))
                for p in msg.poses[-2:]]
        self._push('path', {'n': n, 'head': head, 'tail': tail})

    def _on_pose(self, msg):
        self._poses.append((
            self._wall(), msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.z))

    def _on_stats(self, msg):
        self._push('fusion_stats', msg.data)
        import re
        m = re.search(r'dynamic_occ=(\d+)', msg.data)
        if m:
            self._dyn_state = int(m.group(1))

    def _grid_summary(self, msg, tag):
        """提取 fused grid 摘要(占用格数、free 数)+ 全量栅格 base64,供注入前后对比/轨迹检查。"""
        import base64
        vals = np.frombuffer(msg.data, dtype=np.int8)
        return {
            'tag': tag,
            't': self._wall(),
            'sx': msg.info.width, 'sy': msg.info.height,
            'ox': msg.info.origin.position.x, 'oy': msg.info.origin.position.y,
            'res': float(msg.info.resolution),
            'occ': int((vals == OCC).sum()), 'unknown': int((vals == UNK).sum()),
            'free': int((vals == FREE).sum()),
            'grid_b64': base64.b64encode(vals.tobytes()).decode('ascii'),
        }

    def _on_fused(self, msg):
        self._fused_n += 1
        # 注意: fusion 先发 fused_grid 后发 stats,_dyn_state 滞后一帧。
        # before=首帧(阶段C注入前静态图恒定);after=观测到 dynamic_occ>0 后的帧。
        if self._grid_before is None:
            self._grid_before = self._grid_summary(msg, 'before')
        elif self._grid_after is None and self._dyn_state > 0:
            self._grid_after = self._grid_summary(msg, 'after')

    def _tick(self):
        if time.time() - self._start_wall > self._quit_after:
            self._write_report()
            rclpy.shutdown()

    def _write_report(self):
        report = {
            'wall_start_epoch': self._start_wall,
            'wall_duration_s': round(time.time() - self._start_wall, 1),
            'fused_grid_messages': self._fused_n,
            'events': self._events,
            'poses': self._poses,
            'grid_before': self._grid_before,
            'grid_after': self._grid_after,
        }
        with open(self._out, 'w') as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        self.get_logger().info(f'collector report -> {self._out} '
                               f'(walls={self._fused_n} events={len(self._events)})')


def main(args=None):
    rclpy.init(args=args)
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', default='/tmp/loop_report.json')
    ap.add_argument('--quit-after', default='40')
    a = ap.parse_args()
    node = Collector(a.output, a.quit_after)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        # 外部 SIGTERM/_tick shutdown 后 spin 抛 ExternalShutdownException。
        # 报告可能尚未写出(竞态),兜底写一次;幂等,覆盖安全。
        import traceback
        traceback.print_exc()
        try:
            node._write_report()
        except Exception as e2:
            print(f'[collector] 兜底写盘失败: {e2}', file=sys.stderr)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()