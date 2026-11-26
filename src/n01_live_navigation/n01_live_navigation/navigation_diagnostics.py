"""Navigation diagnostics recorder.

Writes one JSONL event stream per navigation session under
/home/demo/lyy/n01_runtime/run_logs. It never prints routine status to the
console; logs are flushed to disk during the run and finalized on arrival,
failure, cancellation or process shutdown.
"""
from __future__ import annotations

import json
import math
import os
import signal
import time
from datetime import datetime
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


TERMINAL = {
    "ARRIVED", "FAILED", "CANCELLED", "SESSION_COMPLETE", "GOAL_CHANGED",
    "RECOVERY_FAILED", "MANUAL_OVERRIDE", "ROTATION_TIMEOUT", "ROTATION_DRIFT",
}


def _path_summary(msg: NavPath) -> dict:
    pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
           for p in msg.poses]
    length = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    return {
        "frame_id": msg.header.frame_id,
        "poses": len(pts),
        "length_m": round(length, 3),
        "start": [round(v, 3) for v in pts[0]] if pts else None,
        "end": [round(v, 3) for v in pts[-1]] if pts else None,
    }


def _grid_summary(msg: OccupancyGrid) -> dict:
    free = sum(1 for v in msg.data if v == 0)
    occupied = sum(1 for v in msg.data if v == 100)
    return {
        "frame_id": msg.header.frame_id,
        "width": msg.info.width,
        "height": msg.info.height,
        "resolution": msg.info.resolution,
        "origin": [round(msg.info.origin.position.x, 3),
                   round(msg.info.origin.position.y, 3)],
        "free": free,
        "occupied": occupied,
        "unknown": len(msg.data) - free - occupied,
        "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
    }


class NavigationDiagnostics(Node):
    def __init__(self):
        super().__init__("n01_navigation_diagnostics")
        self.declare_parameter("run_log_root", "/home/demo/lyy/n01_runtime/run_logs")
        self.declare_parameter("map_path", "")
        self.declare_parameter("record_period_s", 0.5)
        self.root = Path(str(self.get_parameter("run_log_root").value))
        self.map_path = str(self.get_parameter("map_path").value or "")
        self.period = max(0.1, float(self.get_parameter("record_period_s").value))
        self.root.mkdir(parents=True, exist_ok=True)
        self.session_dir = None
        self.stream = None
        self.session_started = 0.0
        self.last_body = 0.0
        self.last_path = 0.0
        self.last_grid = 0.0
        self.counts = {}
        self.last = {}
        self.goal_id = None
        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        small = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/planning/status",
                                 lambda m: self.on_json("planning_status", m), small)
        self.create_subscription(String, "/n01/navigation/execution_status",
                                 lambda m: self.on_execution(m), small)
        self.create_subscription(String, "/n01/ground_reference/status",
                                 lambda m: self.on_json("ground_status", m), small)
        self.create_subscription(Odometry, "/body_pose", self.on_body, best)
        self.create_subscription(NavPath, "/planning/bspline",
                                 lambda m: self.on_path("local_path", m), small)
        self.create_subscription(NavPath, "/planning/global_path",
                                 lambda m: self.on_path("global_path", m), small)
        self.create_subscription(OccupancyGrid, "/grid_map/reference_grid",
                                 lambda m: self.on_grid("reference_grid", m), latched)
        self.create_subscription(OccupancyGrid, "/grid_map/global_grid",
                                 lambda m: self.on_grid("global_grid", m), latched)
        self._start_session("navigation_start")

    def _start_session(self, reason):
        self._close_session(reason)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.root / f"nav_{stamp}_{os.getpid()}"
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.stream = (self.session_dir / "events.jsonl").open("a", encoding="utf-8")
        self.session_started = time.monotonic()
        self.counts = {}
        self.last = {}
        self.goal_id = None
        self._write("session_start", {"reason": reason, "map_path": self.map_path})

    def _write(self, kind, data):
        if self.stream is None:
            self._start_session("auto_restart")
        rec = {"wall_time": time.time(), "ros_time": self.get_clock().now().nanoseconds * 1e-9,
               "kind": kind, "data": data}
        self.stream.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")
        self.stream.flush()
        self.counts[kind] = self.counts.get(kind, 0) + 1
        self.last[kind] = rec

    def _close_session(self, reason):
        if self.stream is None:
            return
        try:
            self._write("session_end", {"reason": reason,
                                        "duration_s": round(time.monotonic() - self.session_started, 3)})
            summary = {
                "map_path": self.map_path,
                "started_wall": self.session_started,
                "duration_s": round(time.monotonic() - self.session_started, 3),
                "event_counts": self.counts,
                "last": {k: v for k, v in self.last.items()},
            }
            (self.session_dir / "summary.json").write_text(
                json.dumps(summary, ensure_ascii=False, indent=2, default=str),
                encoding="utf-8")
            self.stream.close()
        finally:
            self.stream = None

    def on_json(self, kind, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"raw": msg.data}
        self._write(kind, data)
        if kind == "planning_status":
            gid = data.get("goal_id")
            if gid is not None:
                self.goal_id = gid
            if str(data.get("phase", "")).endswith("FAILED") and data.get("fault_evidence"):
                self._write("failure_evidence", data.get("fault_evidence"))

    def on_execution(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"raw": msg.data}
        self._write("execution_status", data)
        state = str(data.get("state") or data.get("task_state") or "")
        if state in TERMINAL:
            self._close_session(f"terminal:{state}")

    def on_body(self, msg):
        now = time.monotonic()
        if now - self.last_body < self.period:
            return
        self.last_body = now
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v = msg.twist.twist
        self._write("body_pose", {
            "frame": msg.header.frame_id, "child": msg.child_frame_id,
            "position": [round(p.x, 4), round(p.y, 4), round(p.z, 4)],
            "orientation": [round(q.x, 4), round(q.y, 4), round(q.z, 4), round(q.w, 4)],
            "linear": [round(v.linear.x, 3), round(v.linear.y, 3), round(v.linear.z, 3)],
            "angular_z": round(v.angular.z, 3),
        })

    def on_path(self, kind, msg):
        now = time.monotonic()
        if now - self.last_path < self.period:
            return
        self.last_path = now
        self._write(kind, _path_summary(msg))

    def on_grid(self, kind, msg):
        now = time.monotonic()
        if now - self.last_grid < max(1.0, self.period):
            return
        self.last_grid = now
        self._write(kind, _grid_summary(msg))


def main(args=None):
    rclpy.init(args=args)
    node = NavigationDiagnostics()
    stopping = [False]
    def _stop(*_):
        stopping[0] = True
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node._close_session("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
