#!/usr/bin/env python3
"""Measure only the live /robot_pose publication rate and timestamp freshness."""
import argparse
import json
import math
import platform
import sys
import time
from pathlib import Path

IMPORT_ERROR = None
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
except Exception as exc:
    IMPORT_ERROR = exc
    rclpy = None


def percentile(values, fraction):
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return None
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return round(ordered[index], 6)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 5.0 <= args.duration <= 300.0:
        parser.error("--duration must be between 5 and 300 seconds")

    args.output.parent.mkdir(parents=True, exist_ok=True)

    if IMPORT_ERROR is not None:
        report = {
            "schema": "n01.robot_pose_realtime_audit.v1",
            "status": "ROS_PYTHON_IMPORT_FAILED",
            "exception_type": type(IMPORT_ERROR).__name__,
            "exception": str(IMPORT_ERROR),
            "python_executable": sys.executable,
            "python_version": platform.python_version(),
            "topic": "/robot_pose",
            "read_only": True,
            "publishers_created": False,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 2

    class PoseRealtimeNode(Node):
        def __init__(self):
            super().__init__("n01_robot_pose_realtime_audit")
            self.started_monotonic = time.monotonic()
            self.receive_times = []
            self.stamp_times_ns = []
            self.ros_stamp_ages_s = []
            self.wall_stamp_ages_s = []
            self.invalid_stamp_count = 0
            self.recent_samples = []
            try:
                self.use_sim_time = bool(self.get_parameter("use_sim_time").value)
            except Exception:
                self.use_sim_time = False
            # Use a private name; some rclpy Node versions expose a read-only
            # subscriptions property.
            self._pose_subscription = self.create_subscription(
                Odometry, "/robot_pose", self._on_pose, qos_profile_sensor_data
            )

        def _on_pose(self, message):
            received_monotonic = time.monotonic()
            received_wall_ns = time.time_ns()
            received_ros_ns = self.get_clock().now().nanoseconds
            stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            self.receive_times.append(received_monotonic)
            if stamp_ns <= 0:
                self.invalid_stamp_count += 1
                return

            ros_age = (received_ros_ns - stamp_ns) / 1e9
            wall_age = (received_wall_ns - stamp_ns) / 1e9
            self.stamp_times_ns.append(stamp_ns)
            self.ros_stamp_ages_s.append(ros_age)
            self.wall_stamp_ages_s.append(wall_age)

            self.recent_samples.append({
                "elapsed_s": round(received_monotonic - self.started_monotonic, 4),
                "stamp_age_ros_s": round(ros_age, 6),
                "stamp_age_wall_s": round(wall_age, 6),
                "stamp_ns": stamp_ns,
            })
            if len(self.recent_samples) > 20:
                del self.recent_samples[:-20]

        def build_report(self):
            now_monotonic = time.monotonic()
            now_ros_ns = self.get_clock().now().nanoseconds
            receive_gaps = [
                later - earlier
                for earlier, later in zip(self.receive_times, self.receive_times[1:])
                if later >= earlier
            ]
            stamp_gaps = [
                (later - earlier) / 1e9
                for earlier, later in zip(self.stamp_times_ns, self.stamp_times_ns[1:])
                if later > earlier
            ]
            receive_duration = (
                self.receive_times[-1] - self.receive_times[0]
                if len(self.receive_times) > 1 else 0.0
            )
            stamp_duration = (
                (self.stamp_times_ns[-1] - self.stamp_times_ns[0]) / 1e9
                if len(self.stamp_times_ns) > 1
                and self.stamp_times_ns[-1] > self.stamp_times_ns[0] else 0.0
            )
            receive_rate = (
                (len(self.receive_times) - 1) / receive_duration
                if receive_duration > 0 else 0.0
            )
            stamp_rate = (
                len(stamp_gaps) / stamp_duration if stamp_duration > 0 else 0.0
            )

            latest_receive_age = (
                now_monotonic - self.receive_times[-1]
                if self.receive_times else None
            )
            latest_ros_stamp_age = (
                (now_ros_ns - self.stamp_times_ns[-1]) / 1e9
                if self.stamp_times_ns else None
            )
            latest_wall_stamp_age = (
                (time.time_ns() - self.stamp_times_ns[-1]) / 1e9
                if self.stamp_times_ns else None
            )

            blockers = []
            if not self.receive_times:
                status = "NO_ROBOT_POSE_MESSAGES_RECEIVED"
            else:
                if receive_gaps and max(receive_gaps) > 0.7:
                    blockers.append("receive_gap_exceeded_0.7s")
                if self.ros_stamp_ages_s and max(self.ros_stamp_ages_s) > 1.5:
                    blockers.append("pose_stamp_age_exceeded_1.5s")
                if latest_receive_age is not None and latest_receive_age > 0.7:
                    blockers.append("latest_pose_receive_age_exceeded_0.7s")
                if latest_ros_stamp_age is not None and latest_ros_stamp_age > 1.5:
                    blockers.append("latest_pose_stamp_age_exceeded_1.5s")
                status = "POSE_REALTIME_WITHIN_OBSERVED_BUDGET" if not blockers else "POSE_REALTIME_BUDGET_EXCEEDED"

            return {
                "schema": "n01.robot_pose_realtime_audit.v1",
                "status": status,
                "topic": "/robot_pose",
                "message_type": "nav_msgs/msg/Odometry",
                "duration_s": round(now_monotonic - self.started_monotonic, 3),
                "sample_count": len(self.receive_times),
                "read_only": True,
                "publishers_created": False,
                "clients_created": False,
                "goal_published": False,
                "parameter_write": False,
                "process_restart": False,
                "actuator_output": "DISABLED",
                "safety_authorization": "NEVER",
                "clock": {
                    "use_sim_time": self.use_sim_time,
                    "ros_clock_type": str(self.get_clock().clock_type),
                    "note": "ROS stamp age is the primary metric; wall-clock age is included as a cross-check.",
                },
                "thresholds": {
                    "max_pose_receive_age_s": 0.7,
                    "max_pose_stamp_age_s": 1.5,
                },
                "metrics": {
                    "receive_rate_hz": round(receive_rate, 3),
                    "stamp_advance_rate_hz": round(stamp_rate, 3),
                    "receive_gap_p50_s": percentile(receive_gaps, 0.50),
                    "receive_gap_p95_s": percentile(receive_gaps, 0.95),
                    "receive_gap_max_s": round(max(receive_gaps), 6) if receive_gaps else None,
                    "stamp_interval_p50_s": percentile(stamp_gaps, 0.50),
                    "stamp_interval_p95_s": percentile(stamp_gaps, 0.95),
                    "stamp_age_ros_p50_s": percentile(self.ros_stamp_ages_s, 0.50),
                    "stamp_age_ros_p95_s": percentile(self.ros_stamp_ages_s, 0.95),
                    "stamp_age_ros_max_s": round(max(self.ros_stamp_ages_s), 6) if self.ros_stamp_ages_s else None,
                    "stamp_age_wall_p50_s": percentile(self.wall_stamp_ages_s, 0.50),
                    "stamp_age_wall_p95_s": percentile(self.wall_stamp_ages_s, 0.95),
                    "stamp_age_wall_max_s": round(max(self.wall_stamp_ages_s), 6) if self.wall_stamp_ages_s else None,
                    "latest_pose_receive_age_at_end_s": round(latest_receive_age, 6) if latest_receive_age is not None else None,
                    "latest_pose_stamp_age_ros_at_end_s": round(latest_ros_stamp_age, 6) if latest_ros_stamp_age is not None else None,
                    "latest_pose_stamp_age_wall_at_end_s": round(latest_wall_stamp_age, 6) if latest_wall_stamp_age is not None else None,
                    "invalid_or_zero_stamp_count": self.invalid_stamp_count,
                },
                "budget_exceeded": blockers,
                "recent_samples": self.recent_samples,
            }

    node = None
    initialized = False
    try:
        rclpy.init(args=None)
        initialized = True
        node = PoseRealtimeNode()
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        report = node.build_report()
        args.output.write_text(
            json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True))
        return 0 if report["status"] != "NO_ROBOT_POSE_MESSAGES_RECEIVED" else 4
    except Exception as exc:
        report = {
            "schema": "n01.robot_pose_realtime_audit.v1",
            "status": "ROS_RUNTIME_AUDIT_FAILED",
            "exception_type": type(exc).__name__,
            "exception": str(exc),
            "topic": "/robot_pose",
            "read_only": True,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 3
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
