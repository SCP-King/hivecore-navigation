"""Record navigation /cmd_vel commands per task under n01_runtime/run_cmd.

Only commands issued while the navigation controller reports enabled=true are
recorded.  The controller and manual teleop share /cmd_vel, so the execution
status is the ownership filter; manual/preprocessor traffic is ignored.
"""
import csv
import json
import os
import re
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class RunCmdLogger(Node):
    def __init__(self):
        super().__init__('n01_run_cmd_logger')
        default_dir = Path.home() / 'lyy' / 'n01_runtime' / 'run_cmd'
        self.log_dir = Path(self.declare_parameter('log_dir', str(default_dir)).value)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.status = None
        self.task_id = ''
        self.file = None
        self.writer = None
        self.path = None
        self.rows = 0
        self.last_status_dump = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(String, '/n01/navigation/execution_status', self.on_status, 10)
        self.get_logger().info(f'run_cmd log dir: {self.log_dir}')

    @staticmethod
    def _safe(text):
        return re.sub(r'[^A-Za-z0-9_.-]+', '_', text or 'unknown')[:80]

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        task = str(self.status.get('task_id') or '')
        if task != self.task_id:
            self.close()
            self.task_id = task
            if task:
                stamp = time.strftime('%Y%m%d_%H%M%S')
                self.path = self.log_dir / f'run_{stamp}_{self._safe(task)}.csv'
                self.file = self.path.open('w', newline='', encoding='utf-8')
                self.writer = csv.writer(self.file)
                self.writer.writerow([
                    'wall_time', 'ros_stamp_ns', 'monotonic_s', 'task_id',
                    'trajectory_id', 'state', 'phase', 'reason',
                    'enabled', 'paused', 'manual_override',
                    'cmd_linear_x', 'cmd_linear_y', 'cmd_angular_z',
                    'admitted_speed_mps', 'max_speed_mps', 'navigation_degraded',
                    'speed_limit_sources',
                    'latest_packet_valid', 'steering_scale', 'v_before_scale',
                    'safety_reductions', 'physical_reason', 'legs_home', 'leg_home_gate',
                ])
                self.file.flush()
                self.get_logger().info(f'run log: {self.path}')
        # Status-only event line at a low rate, even with no command yet.
        now = time.monotonic()
        if self.status and now - self.last_status_dump > 2.0 and self.writer:
            self.last_status_dump = now

    def on_cmd(self, msg):
        if not self.status or not self.status.get('enabled') or self.status.get('manual_override'):
            return
        if self.writer is None:
            return
        s = self.status
        stamp_ns = self.get_clock().now().nanoseconds
        # Twist has no header; use the controller status timestamp if present,
        # otherwise leave unchanged and keep the local wall/monotonic columns.
        self.writer.writerow([
            time.time(), stamp_ns, time.monotonic(), self.task_id,
            s.get('trajectory_id', ''), s.get('state', ''), s.get('phase', ''),
            s.get('reason', ''), s.get('enabled'), s.get('paused'),
            s.get('manual_override'), f'{msg.linear.x:.9f}', f'{msg.linear.y:.9f}',
            f'{msg.angular.z:.9f}', s.get('admitted_speed_mps', ''),
            s.get('max_speed_mps', ''), s.get('navigation_degraded', ''),
            s.get('speed_limit_sources', ''),
            s.get('latest_packet_valid', ''), s.get('steering_scale', ''),
            s.get('v_before_scale', ''), s.get('safety_reductions', ''),
            s.get('physical_reason', ''), s.get('legs_home', ''),
            s.get('leg_home_gate', ''),
        ])
        self.rows += 1
        if self.rows % 50 == 0:
            self.file.flush()

    def close(self):
        if self.file is not None:
            try:
                self.file.flush()
                self.file.close()
            except Exception:
                pass
            self.get_logger().info(f'closed {self.path} rows={self.rows}')
        self.file = None
        self.writer = None
        self.path = None
        self.rows = 0


def main(args=None):
    rclpy.init(args=args)
    node = RunCmdLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
