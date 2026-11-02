#!/usr/bin/env python3
"""Read-only check that a rosbag2 contains FAST-LIO-compatible Livox inputs."""
import argparse
import json
from pathlib import Path
import sys

EXPECTED_LIDAR_TYPE = "livox_ros_driver2/msg/CustomMsg"
EXPECTED_IMU_TYPE = "sensor_msgs/msg/Imu"


def inspect(bag: Path, lidar_topic: str, imu_topic: str):
    root = bag.expanduser()
    if root.is_file():
        root = root.parent
    metadata = root / "metadata.yaml"
    result = {
        "schema": "n01.fast_lio2.bag_input_preflight.v1",
        "bag_path": str(root),
        "metadata_exists": metadata.is_file(),
        "lidar_topic": lidar_topic,
        "imu_topic": imu_topic,
        "lidar_expected_type": EXPECTED_LIDAR_TYPE,
        "imu_expected_type": EXPECTED_IMU_TYPE,
        "status": "NOT_READY",
        "fast_lio_launch_allowed_by_this_check": False,
    }
    if not metadata.is_file():
        result["reason"] = "missing_rosbag2_metadata_yaml"
        return result
    try:
        import yaml
        data = yaml.safe_load(metadata.read_text(encoding="utf-8")) or {}
        info = data["rosbag2_bagfile_information"]
        topics = {}
        for row in info.get("topics_with_message_count", []):
            meta = row.get("topic_metadata", {})
            name = meta.get("name")
            if name:
                topics[name] = {
                    "type": meta.get("type"),
                    "count": int(row.get("message_count", 0)),
                }
        lidar = topics.get(lidar_topic)
        imu = topics.get(imu_topic)
        filtered = topics.get("/livox/lidar/geo_filtered")
        result.update({
            "lidar": lidar,
            "imu": imu,
            "vendor_filtered_cloud_not_used_by_fast_lio": filtered,
            "storage_identifier": info.get("storage_identifier"),
            "duration_nanoseconds": info.get("duration", {}).get("nanoseconds"),
        })
        errors = []
        if not lidar or lidar["count"] <= 0:
            errors.append("raw_livox_custom_message_missing")
        elif lidar["type"] != EXPECTED_LIDAR_TYPE:
            errors.append("raw_lidar_type_must_be_livox_ros_driver2_CustomMsg")
        if not imu or imu["count"] <= 0:
            errors.append("livox_imu_missing")
        elif imu["type"] != EXPECTED_IMU_TYPE:
            errors.append("imu_type_must_be_sensor_msgs_Imu")
        result["errors"] = errors
        if not errors:
            result["status"] = "INPUT_TYPES_PRESENT"
            result["fast_lio_launch_allowed_by_this_check"] = True
            result["note"] = (
                "Topic metadata only. Per-point timestamp values, clock alignment, "
                "extrinsic correctness and estimator convergence still require one replay."
            )
        else:
            result["reason"] = "required_fast_lio_input_contract_not_met"
    except (OSError, KeyError, ValueError, TypeError, ImportError) as exc:
        result["reason"] = type(exc).__name__ + ": " + str(exc)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--imu-topic", default="/livox/imu")
    args = parser.parse_args()
    result = inspect(Path(args.bag), args.lidar_topic, args.imu_topic)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["fast_lio_launch_allowed_by_this_check"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
