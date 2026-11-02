#!/usr/bin/env python3
"""Apply the small N-01 isolation patch to the pinned upstream FAST-LIO2 ROS 2 source.

The upstream source is fetched separately; no FAST-LIO implementation code is
copied into hivecore-navigation. This patch only parameterizes frame IDs and
makes the upstream TF broadcaster opt-in (OFF by default).
"""
import argparse
from pathlib import Path
import subprocess
import sys

PIN = "cc3c9e8b1469b33b8072764d6db2348ed6dc597f"
MARKER = "// N01_FAST_LIO_ISOLATION_PATCH_V1"


def replace_exact(text: str, old: str, new: str, expected: int) -> str:
    count = text.count(old)
    if count != expected:
        raise RuntimeError(
            "Upstream source contract changed: expected %d occurrence(s), found %d for %r"
            % (expected, count, old)
        )
    return text.replace(old, new)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="src/third_party/FAST_LIO")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    root = Path(args.source).expanduser().resolve()
    cpp = root / "src" / "laserMapping.cpp"
    if not cpp.is_file():
        print("FAST-LIO source not found: %s" % cpp, file=sys.stderr)
        return 2
    try:
        head = subprocess.check_output(
            ["git", "-C", str(root), "rev-parse", "HEAD"], text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        print("FAST-LIO source must be a Git checkout: %s" % exc, file=sys.stderr)
        return 2
    if head != PIN:
        print("Refusing unpinned FAST-LIO source: expected %s, got %s" % (PIN, head),
              file=sys.stderr)
        return 2

    source = cpp.read_text(encoding="utf-8")
    required = (
        MARKER,
        'common.map_frame_id',
        'common.imu_frame_id',
        'common.publish_tf',
        'if (n01_publish_tf) tf_br->sendTransform(trans);',
        'odomAftMapped.header.frame_id = n01_map_frame_id;',
        'odomAftMapped.child_frame_id = n01_imu_frame_id;',
    )
    if MARKER in source:
        missing = [item for item in required if item not in source]
        if missing:
            print("FAST-LIO N-01 patch is partial/corrupt: %r" % missing, file=sys.stderr)
            return 2
        print("FAST-LIO2 N-01 isolation patch already present; no changes.")
        return 0

    if args.check:
        print("FAST-LIO2 N-01 isolation patch is NOT applied.", file=sys.stderr)
        return 1

    patched = source
    patched = replace_exact(
        patched,
        "string map_file_path, lid_topic, imu_topic;",
        "string map_file_path, lid_topic, imu_topic;\n"
        'string n01_map_frame_id = "n01_fast_lio_map";\n'
        'string n01_imu_frame_id = "imu_link";\n'
        "bool n01_publish_tf = false;\n"
        + MARKER,
        1,
    )
    patched = replace_exact(
        patched,
        'this->declare_parameter<string>("common.imu_topic", "/livox/imu");',
        'this->declare_parameter<string>("common.imu_topic", "/livox/imu");\n'
        '        this->declare_parameter<string>("common.map_frame_id", "n01_fast_lio_map");\n'
        '        this->declare_parameter<string>("common.imu_frame_id", "imu_link");\n'
        '        this->declare_parameter<bool>("common.publish_tf", false);',
        1,
    )
    patched = replace_exact(
        patched,
        'this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");',
        'this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");\n'
        '        this->get_parameter_or<string>("common.map_frame_id", n01_map_frame_id, "n01_fast_lio_map");\n'
        '        this->get_parameter_or<string>("common.imu_frame_id", n01_imu_frame_id, "imu_link");\n'
        '        this->get_parameter_or<bool>("common.publish_tf", n01_publish_tf, false);',
        1,
    )

    # Keep the FAST-LIO frame tree private and prevent it from competing with
    # the vendor SLAM / Hive2 motion TF authorities.
    patched = replace_exact(
        patched, 'laserCloudmsg.header.frame_id = "camera_init";',
        "laserCloudmsg.header.frame_id = n01_map_frame_id;", 2)
    patched = replace_exact(
        patched, 'laserCloudFullRes3.header.frame_id = "camera_init";',
        "laserCloudFullRes3.header.frame_id = n01_map_frame_id;", 1)
    patched = replace_exact(
        patched, 'odomAftMapped.header.frame_id = "camera_init";',
        "odomAftMapped.header.frame_id = n01_map_frame_id;", 1)
    patched = replace_exact(
        patched, 'trans.header.frame_id = "camera_init";',
        "trans.header.frame_id = n01_map_frame_id;", 1)
    patched = replace_exact(
        patched, 'msg_body_pose.header.frame_id = "camera_init";',
        "msg_body_pose.header.frame_id = n01_map_frame_id;", 1)
    patched = replace_exact(
        patched, 'path.header.frame_id ="camera_init";',
        "path.header.frame_id = n01_map_frame_id;", 1)
    patched = replace_exact(
        patched, 'laserCloudmsg.header.frame_id = "body";',
        "laserCloudmsg.header.frame_id = n01_imu_frame_id;", 1)
    patched = replace_exact(
        patched, 'odomAftMapped.child_frame_id = "body";',
        "odomAftMapped.child_frame_id = n01_imu_frame_id;", 1)
    patched = replace_exact(
        patched, 'trans.child_frame_id = "body";',
        "trans.child_frame_id = n01_imu_frame_id;", 1)
    patched = replace_exact(
        patched, "tf_br->sendTransform(trans);",
        "if (n01_publish_tf) tf_br->sendTransform(trans);", 1)

    if any(item not in patched for item in required):
        raise RuntimeError("Post-patch verification failed; source was not written.")
    tmp = cpp.with_suffix(".cpp.n01.tmp")
    tmp.write_text(patched, encoding="utf-8")
    tmp.replace(cpp)
    print("FAST-LIO2 N-01 isolation patch applied.")
    print("  private map frame: n01_fast_lio_map")
    print("  IMU body frame: imu_link")
    print("  FAST-LIO TF broadcaster: disabled by default")
    print("  vendor SLAM topics and source files: untouched")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
