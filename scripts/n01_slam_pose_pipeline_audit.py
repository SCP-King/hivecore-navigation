#!/usr/bin/env python3
"""Read-only forensic audit of the live Hivecore SLAM pose pipeline.

Subscribes to existing sensor/status topics and reads ROS graph, parameters,
process metadata and any locally available SLAM source tree. It does not create
publishers/clients, send goals, call services/actions, or change parameters.
"""
import argparse
import collections
import json
import math
import os
import platform
import re
import socket
import subprocess
import sys
import time
from pathlib import Path

IMPORT_ERROR = None
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2
    from geometry_msgs.msg import Vector3Stamped
    from rosidl_runtime_py.utilities import get_message
except Exception as exc:
    IMPORT_ERROR = exc
    rclpy = None


def percentile(values, fraction):
    values = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not values:
        return None
    index = min(len(values) - 1, max(0, math.ceil(fraction * len(values)) - 1))
    return values[index]


def run_command(command, timeout=12, limit=30000):
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=timeout, check=False)
        return {
            "exit_code": result.returncode,
            "stdout": result.stdout[-limit:],
            "stderr": result.stderr[-5000:],
        }
    except Exception as exc:
        return {"error": type(exc).__name__ + ": " + str(exc)}


def node_full_name(namespace, name):
    namespace = str(namespace or "/").rstrip("/")
    name = str(name or "").strip("/")
    return (namespace + "/" + name) if namespace else "/" + name


def endpoint_summary(info):
    try:
        gid = bytes(info.endpoint_gid).hex()
    except Exception:
        gid = str(getattr(info, "endpoint_gid", ""))
    qos = getattr(info, "qos_profile", None)
    return {
        "node": node_full_name(info.node_namespace, info.node_name),
        "gid": gid,
        "type": str(info.topic_type),
        "qos": {
            "history": str(getattr(qos, "history", "UNKNOWN")),
            "depth": int(getattr(qos, "depth", 0) or 0),
            "reliability": str(getattr(qos, "reliability", "UNKNOWN")),
            "durability": str(getattr(qos, "durability", "UNKNOWN")),
        } if qos is not None else None,
    }


def stamp_ns_from_message(msg):
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else None
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else None


def simple_fields(msg, depth=0):
    if depth > 2 or not hasattr(msg, "get_fields_and_field_types"):
        return None
    result = {}
    try:
        fields = msg.get_fields_and_field_types()
    except Exception:
        return None
    for name in fields:
        try:
            value = getattr(msg, name)
        except Exception:
            continue
        if isinstance(value, (bool, int, float, str)):
            result[name] = value
        elif isinstance(value, (list, tuple)):
            if len(value) <= 12 and all(isinstance(x, (bool, int, float, str)) for x in value):
                result[name] = list(value)
        elif hasattr(value, "get_fields_and_field_types"):
            nested = simple_fields(value, depth + 1)
            if nested is not None:
                result[name] = nested
    return result


def process_snapshot():
    result = []
    try:
        ps = subprocess.run(
            ["ps", "-eo", "pid=,ppid=,%cpu=,%mem=,rss=,etimes=,args="],
            capture_output=True, text=True, timeout=5, check=False)
    except Exception as exc:
        return [{"error": type(exc).__name__ + ": " + str(exc)}]
    for line in ps.stdout.splitlines():
        parts = line.strip().split(None, 6)
        if len(parts) < 7:
            continue
        pid_text, ppid, cpu, mem, rss, elapsed, command = parts
        if not re.search(r"(^|/)slam3d_node(?:\s|$)", command):
            continue
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        item = {
            "pid": pid, "ppid": ppid, "cpu_percent_ps_average": cpu,
            "memory_percent": mem, "rss_kib": rss, "elapsed_seconds": elapsed,
            "command": command,
        }
        proc = Path("/proc") / str(pid)
        try:
            item["executable"] = os.readlink(str(proc / "exe"))
        except Exception as exc:
            item["executable_error"] = str(exc)
        try:
            item["cwd"] = os.readlink(str(proc / "cwd"))
        except Exception as exc:
            item["cwd_error"] = str(exc)
        try:
            raw = (proc / "cmdline").read_bytes()
            item["cmdline"] = raw.replace(b"\0", b" ").decode("utf-8", "replace").strip()
        except Exception as exc:
            item["cmdline_error"] = str(exc)
        try:
            status = (proc / "status").read_text(encoding="utf-8", errors="replace")
            item["status_memory_lines"] = [
                line for line in status.splitlines()
                if line.startswith(("VmPeak:", "VmSize:", "VmRSS:", "Threads:"))
            ]
        except Exception as exc:
            item["status_error"] = str(exc)
        try:
            maps = (proc / "maps").read_text(encoding="utf-8", errors="replace").splitlines()
            libs = sorted({
                line.split()[-1] for line in maps
                if "/" in line and re.search(r"(hivecore|slam|mola|gtsam|pcl|fastdds)", line, re.I)
            })
            item["relevant_loaded_libraries"] = libs[:100]
        except Exception as exc:
            item["maps_error"] = str(exc)
        result.append(item)
    return result


def inspect_source_tree(processes):
    executable = None
    for process in processes:
        candidate = process.get("executable")
        if candidate and candidate.startswith("/"):
            executable = candidate
            break
    workspace = None
    if executable:
        for parent in Path(executable).parents:
            if parent.name == "install":
                workspace = parent.parent
                break
    result = {
        "slam_executable": executable,
        "workspace_root_inferred_from_install_prefix": str(workspace) if workspace else None,
        "source_directory": str(workspace / "src") if workspace and (workspace / "src").is_dir() else None,
        "source_files_found": 0,
        "robot_pose_source_matches": [],
        "publisher_creation_matches": [],
        "config_matches": [],
        "note": "No source conclusion is made if the installed workspace does not contain source files.",
    }
    if not workspace or not (workspace / "src").is_dir():
        return result

    source_extensions = {".cpp", ".cc", ".cxx", ".h", ".hpp", ".hh", ".py"}
    config_extensions = {".yaml", ".yml", ".xml", ".py"}
    excluded = {"build", "install", "log", ".git", "__pycache__"}
    file_count = 0
    for base, dirs, files in os.walk(workspace / "src"):
        dirs[:] = [name for name in dirs if name not in excluded]
        for filename in files:
            path = Path(base) / filename
            suffix = path.suffix.lower()
            if suffix not in source_extensions | config_extensions:
                continue
            file_count += 1
            if file_count > 15000:
                result["scan_limit_reached"] = True
                break
            try:
                if path.stat().st_size > 2_000_000:
                    continue
                lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
            except Exception:
                continue
            for index, line in enumerate(lines):
                lower = line.lower()
                record = {
                    "file": str(path),
                    "line": index + 1,
                    "text": line.strip()[:500],
                }
                if "robot_pose" in lower:
                    result["robot_pose_source_matches"].append(record)
                if suffix in source_extensions and (
                    "create_publisher" in lower
                    or "advertise<nav_msgs" in lower
                    or "nav_msgs::msg::odometry" in lower
                ):
                    result["publisher_creation_matches"].append(record)
                if suffix in config_extensions and re.search(
                    r"pose|publish|frequency|rate|timestamp|imu|sync", lower
                ):
                    if any(term in lower for term in (
                        "robot_pose", "pose_publish", "publish_pose", "publish_rate",
                        "publish_frequency", "imu", "sync", "frequency", "rate",
                    )):
                        result["config_matches"].append(record)
                if len(result["robot_pose_source_matches"]) >= 160:
                    break
            if len(result["robot_pose_source_matches"]) >= 160:
                break
        if file_count > 15000 or len(result["robot_pose_source_matches"]) >= 160:
            break
    result["source_files_found"] = file_count
    result["robot_pose_source_matches"] = result["robot_pose_source_matches"][:160]
    result["publisher_creation_matches"] = result["publisher_creation_matches"][:160]
    result["config_matches"] = result["config_matches"][:120]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 10.0 <= args.duration <= 120.0:
        parser.error("--duration must be between 10 and 120 seconds")
    args.output.parent.mkdir(parents=True, exist_ok=True)

    if IMPORT_ERROR is not None:
        report = {
            "schema": "n01.slam_pose_pipeline_audit.v1",
            "status": "ROS_PYTHON_IMPORT_FAILED",
            "exception_type": type(IMPORT_ERROR).__name__,
            "exception": str(IMPORT_ERROR),
            "python_executable": sys.executable,
            "python_version": platform.python_version(),
            "read_only": True,
            "publishers_created": False,
            "clients_created": False,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n",
                               encoding="utf-8")
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 2

    class PipelineAuditNode(Node):
        def __init__(self):
            super().__init__("n01_slam_pose_pipeline_audit")
            self.started = time.monotonic()
            self.records = {}
            self.subscriptions = []
            self.endpoint_topics = (
                "/robot_pose", "/odom", "/imu/data",
                "/livox/lidar/geo_filtered", "/cloud",
                "/mapping_processing_timings",
                "/mapping_delta_registration_orientation_at_scan",
                "/mapping_robot_pose_orientation_rpy",
                "/localization_status",
            )
            self._add("/robot_pose", Odometry)
            self._add("/odom", Odometry)
            self._add("/imu/data", Imu)
            self._add("/livox/lidar/geo_filtered", PointCloud2)
            self._add("/cloud", PointCloud2)
            self._add("/mapping_processing_timings", Vector3Stamped)
            self._add("/mapping_delta_registration_orientation_at_scan", Vector3Stamped)
            self._add("/mapping_robot_pose_orientation_rpy", Vector3Stamped)
            try:
                localization_type = get_message("hivecore_slam3d/msg/LocalizationStatus")
                self._add("/localization_status", localization_type)
                self.localization_import_error = ""
            except Exception as exc:
                self.localization_import_error = type(exc).__name__ + ": " + str(exc)
            self.graph_error = ""

        def _add(self, topic, msg_type):
            self.records[topic] = {
                "count": 0, "rx": [], "stamps": [], "stamp_ages": [],
                "out_of_order": 0, "duplicate_stamp": 0,
                "same_stamp_equivalent_pose": 0, "same_stamp_conflicting_pose": 0,
                "same_stamp_seen": {}, "samples": [],
            }
            subscription = self.create_subscription(
                msg_type, topic, lambda msg, topic_name=topic: self._on_message(topic_name, msg),
                qos_profile_sensor_data)
            self.subscriptions.append(subscription)

        def _on_message(self, topic, msg):
            now = time.monotonic()
            record = self.records[topic]
            record["count"] += 1
            record["rx"].append(now)
            if len(record["rx"]) > 5000:
                del record["rx"][:-5000]
            stamp_ns = stamp_ns_from_message(msg)
            age = None
            if stamp_ns is not None:
                age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
                record["stamp_ages"].append(age)
                if len(record["stamp_ages"]) > 5000:
                    del record["stamp_ages"][:-5000]
                if record["stamps"]:
                    previous = record["stamps"][-1]
                    if stamp_ns < previous:
                        record["out_of_order"] += 1
                    elif stamp_ns == previous:
                        record["duplicate_stamp"] += 1
                record["stamps"].append(stamp_ns)
                if len(record["stamps"]) > 5000:
                    del record["stamps"][:-5000]

            sample = {
                "elapsed_s": round(now - self.started, 5),
                "stamp_ns": stamp_ns,
                "stamp_age_s": round(age, 6) if age is not None else None,
            }
            header = getattr(msg, "header", None)
            if header is not None:
                sample["frame_id"] = str(getattr(header, "frame_id", ""))
            if isinstance(msg, Odometry):
                sample["child_frame_id"] = str(msg.child_frame_id)
                position = (
                    float(msg.pose.pose.position.x),
                    float(msg.pose.pose.position.y),
                    float(msg.pose.pose.position.z),
                )
                quaternion = (
                    float(msg.pose.pose.orientation.x),
                    float(msg.pose.pose.orientation.y),
                    float(msg.pose.pose.orientation.z),
                    float(msg.pose.pose.orientation.w),
                )
                sample["position_xyz"] = [round(v, 6) for v in position]
                if topic == "/robot_pose" and stamp_ns is not None:
                    previous = record["same_stamp_seen"].get(stamp_ns)
                    if previous is not None:
                        position_delta = math.sqrt(sum(
                            (a - b) ** 2 for a, b in zip(position, previous[0])))
                        quaternion_dot = abs(sum(
                            a * b for a, b in zip(quaternion, previous[1])))
                        if position_delta > 1e-3 or 1.0 - min(1.0, quaternion_dot) > 1e-5:
                            record["same_stamp_conflicting_pose"] += 1
                        else:
                            record["same_stamp_equivalent_pose"] += 1
                    else:
                        record["same_stamp_seen"][stamp_ns] = (position, quaternion)
                        if len(record["same_stamp_seen"]) > 2048:
                            record["same_stamp_seen"].pop(next(iter(record["same_stamp_seen"])))
            elif isinstance(msg, PointCloud2):
                sample["point_count"] = int(msg.width) * int(msg.height)
                sample["fields"] = [str(field.name) for field in msg.fields]
            elif isinstance(msg, Vector3Stamped):
                sample["vector_xyz"] = [
                    round(float(msg.vector.x), 8),
                    round(float(msg.vector.y), 8),
                    round(float(msg.vector.z), 8),
                ]
            elif isinstance(msg, Imu):
                sample["angular_velocity_xyz"] = [
                    round(float(msg.angular_velocity.x), 7),
                    round(float(msg.angular_velocity.y), 7),
                    round(float(msg.angular_velocity.z), 7),
                ]
                sample["linear_acceleration_xyz"] = [
                    round(float(msg.linear_acceleration.x), 7),
                    round(float(msg.linear_acceleration.y), 7),
                    round(float(msg.linear_acceleration.z), 7),
                ]
            else:
                fields = simple_fields(msg)
                if fields:
                    sample["message_fields"] = fields
            record["samples"].append(sample)
            if len(record["samples"]) > 80:
                del record["samples"][:-80]

        def report_topic(self, topic, record):
            rx = record["rx"]
            stamps = record["stamps"]
            rx_gaps = [b-a for a, b in zip(rx, rx[1:]) if b >= a]
            stamp_gaps = [(b-a)/1e9 for a, b in zip(stamps, stamps[1:]) if b > a]
            rx_duration = rx[-1] - rx[0] if len(rx) > 1 else 0.0
            stamp_duration = (
                (stamps[-1]-stamps[0])/1e9
                if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0)
            return {
                "samples": record["count"],
                "receive_rate_hz": round((len(rx)-1)/rx_duration, 3)
                    if rx_duration > 0 else 0.0,
                "receive_gap_p50_s": percentile(rx_gaps, 0.5),
                "receive_gap_p95_s": percentile(rx_gaps, 0.95),
                "receive_gap_max_s": max(rx_gaps) if rx_gaps else None,
                "stamp_advance_rate_hz": round(len(stamp_gaps)/stamp_duration, 3)
                    if stamp_duration > 0 else 0.0,
                "stamp_interval_p50_s": percentile(stamp_gaps, 0.5),
                "stamp_interval_p95_s": percentile(stamp_gaps, 0.95),
                "stamp_interval_max_s": max(stamp_gaps) if stamp_gaps else None,
                "stamp_age_p50_s": percentile(record["stamp_ages"], 0.5),
                "stamp_age_p95_s": percentile(record["stamp_ages"], 0.95),
                "stamp_age_max_s": max(record["stamp_ages"])
                    if record["stamp_ages"] else None,
                "out_of_order_stamps": record["out_of_order"],
                "adjacent_duplicate_stamps": record["duplicate_stamp"],
                "same_stamp_equivalent_pose": record["same_stamp_equivalent_pose"],
                "same_stamp_conflicting_pose": record["same_stamp_conflicting_pose"],
                "recent_samples": record["samples"],
            }

        def report(self):
            publishers = {}
            for topic in self.endpoint_topics:
                try:
                    publishers[topic] = [
                        endpoint_summary(item)
                        for item in self.get_publishers_info_by_topic(topic)
                    ]
                except Exception as exc:
                    publishers[topic] = {"error": type(exc).__name__ + ": " + str(exc)}
            try:
                names = [
                    node_full_name(namespace, name)
                    for name, namespace in self.get_node_names_and_namespaces()
                ]
                counts = collections.Counter(names)
                duplicates = {name: count for name, count in counts.items() if count > 1}
            except Exception as exc:
                names, duplicates = [], {"graph_error": str(exc)}

            topic_cli = run_command(
                ["ros2", "topic", "info", "/robot_pose", "--verbose"], 10)
            node_cli = run_command(["ros2", "node", "info", "/slam3d_node"], 10)
            params_cli = run_command(["ros2", "param", "dump", "/slam3d_node"], 15, 50000)
            param_list_cli = run_command(["ros2", "param", "list", "/slam3d_node"], 10)
            package_cli = run_command(["ros2", "pkg", "prefix", "hivecore_slam3d"], 10)
            processes = process_snapshot()
            source_tree = inspect_source_tree(processes)

            return {
                "schema": "n01.slam_pose_pipeline_audit.v1",
                "status": "CAPTURED",
                "duration_s": round(time.monotonic() - self.started, 3),
                "read_only": True,
                "publishers_created": False,
                "clients_created": False,
                "service_or_action_calls": False,
                "goal_published": False,
                "parameter_write": False,
                "process_restart": False,
                "actuator_output": "DISABLED",
                "safety_authorization": "NEVER",
                "attribution_limit": (
                    "The normal rclpy subscription callback does not expose the sending "
                    "publisher GID. Duplicate timestamps/payloads are evidence of repeated "
                    "delivery, but are not a per-message DDS endpoint identity."),
                "environment": {
                    "hostname": socket.gethostname(),
                    "python_executable": sys.executable,
                    "python_version": platform.python_version(),
                    "ros_distro": os.environ.get("ROS_DISTRO"),
                    "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
                    "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
                    "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY"),
                },
                "topic_metrics": {
                    topic: self.report_topic(topic, record)
                    for topic, record in self.records.items()
                },
                "publisher_endpoints": publishers,
                "duplicate_ros_node_names": duplicates,
                "ros_node_names": sorted(names),
                "localization_status_import_error": self.localization_import_error,
                "slam_cli": {
                    "robot_pose_topic_info_verbose": topic_cli,
                    "slam3d_node_info": node_cli,
                    "slam3d_node_param_dump": params_cli,
                    "slam3d_node_param_list": param_list_cli,
                    "hivecore_slam3d_package_prefix": package_cli,
                },
                "slam_processes": processes,
                "slam_source_tree": source_tree,
            }

    node = None
    try:
        rclpy.init(args=None)
        node = PipelineAuditNode()
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        report = node.report()
        args.output.write_text(
            json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8")
        print(json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True))
        return 0
    except Exception as exc:
        report = {
            "schema": "n01.slam_pose_pipeline_audit.v1",
            "status": "ROS_RUNTIME_AUDIT_FAILED",
            "exception_type": type(exc).__name__,
            "exception": str(exc),
            "read_only": True,
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        return 3
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
