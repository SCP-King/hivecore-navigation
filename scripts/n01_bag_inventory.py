#!/usr/bin/env python3
"""Read-only rosbag2 metadata inventory; never plays /cmd_vel or starts ROS."""
import argparse
import json
from pathlib import Path

REQUIRED = (
    "/livox/lidar/geo_filtered", "/tf", "/tf_static", "/localization_status"
)
OPTIONAL = ("/robot_pose", "/livox/lidar", "/clock", "/planning/bspline")


def inspect(bag):
    root=Path(bag).expanduser()
    if root.is_file():
        root=root.parent
    meta=root/"metadata.yaml"
    data={"test_type":"N01_BAG_READ_ONLY_INVENTORY", "bag_path":str(root),
          "metadata_exists":meta.is_file(), "result":"NOT_AVAILABLE"}
    if not meta.is_file():
        data["reason"]="missing_rosbag2_metadata_yaml"
        return data
    try:
        import yaml
        content=yaml.safe_load(meta.read_text(encoding="utf-8")) or {}
        info=content["rosbag2_bagfile_information"]
        rows=info.get("topics_with_message_count", [])
        topics={}
        for item in rows:
            topic=item.get("topic_metadata",{})
            if topic.get("name"):
                topics[topic["name"]]={"type":topic.get("type"),
                                        "count":int(item.get("message_count",0))}
        missing=[name for name in REQUIRED if name not in topics or topics[name]["count"] <= 0]
        data.update({
            "result":"INVENTORY_COMPLETE", "topics":topics,
            "storage_identifier":info.get("storage_identifier"),
            "duration_nanoseconds":info.get("duration",{}).get("nanoseconds"),
            "missing_core_topics":missing,
            "optional_topics":{key:topics.get(key) for key in OPTIONAL},
            "ready_for_strict_localization_replay":not missing,
            "recorded_data_runtime_tf_unique": "NOT_PROVEN",
            "slam_rsp_disabled_runtime": "NOT_PROVEN",
            "pointcloud_to_map_verified": "NOT_TESTED",
            "full_slam_status_contract_verified": "NOT_TESTED",
            "automatic_bag_replay": "DISABLED_BY_DEFAULT",
            "note":"Inventory only. Presence of topics cannot prove fresh tracking, TF consistency, collision safety or motion clearance."
        })
    except (OSError, KeyError, ValueError, TypeError, ImportError) as exc:
        data["result"]="INVENTORY_ERROR"
        data["reason"]=type(exc).__name__+": "+str(exc)
    return data


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--bag",required=True)
    parser.add_argument("--output",required=True)
    args=parser.parse_args()
    result=inspect(args.bag)
    target=Path(args.output)
    target.parent.mkdir(parents=True,exist_ok=True)
    pending=target.with_suffix(".tmp")
    pending.write_text(json.dumps(result,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    pending.replace(target)
    print("N01_BAG_INVENTORY="+json.dumps(
        {k:result.get(k) for k in ("result","bag_path","missing_core_topics")},ensure_ascii=False))
    return 0 if result["result"]=="INVENTORY_COMPLETE" else 1


if __name__=="__main__":
    raise SystemExit(main())
