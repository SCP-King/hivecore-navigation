#!/usr/bin/env python3
"""Extract /robot_pose (or any Odometry topic) from a rosbag2 into CSV."""
import argparse
import csv
import math
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--topic", default="/robot_pose")
    parser.add_argument("--output", required=True)
    parser.add_argument("--min-distance", type=float, default=.05)
    args = parser.parse_args()
    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.bag, storage_id="sqlite3"),
                ConverterOptions("", ""))
    if hasattr(reader, "set_filter"):
        reader.set_filter(StorageFilter(topics=[args.topic]))
    types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    if args.topic not in types:
        raise SystemExit(f"topic {args.topic} not in bag; available: {sorted(types)}")
    msg_type = get_message(types[args.topic])
    last = None
    count = 0
    with open(args.output, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if topic != args.topic:
                continue
            msg = deserialize_message(data, msg_type)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            if last is not None and math.hypot(p.x-last[0], p.y-last[1]) < args.min_distance:
                continue
            last = (p.x, p.y)
            writer.writerow([f"{stamp*1e-9:.9f}", p.x, p.y, p.z,
                             q.x, q.y, q.z, q.w])
            count += 1
    print(f"wrote {count} poses to {args.output}")


if __name__ == "__main__":
    main()
