#!/usr/bin/env python3
"""Read-only TF/cloud/status evidence extraction from an existing rosbag2.

No ROS node, no DDS domain interaction, no bag playback, no /cmd_vel.
Bounded reads; unknown message packages are recorded instead of guessed.
"""
import argparse
import json
from pathlib import Path

WATCHED = {"/tf", "/tf_static", "/livox/lidar/geo_filtered", "/localization_status", "/robot_pose"}


def report(bag, limit):
    root=Path(bag).expanduser()
    if root.is_file():
        root=root.parent
    output={"test_type":"N01_RECORDED_BAG_READ_ONLY_EVIDENCE", "bag_path":str(root),
            "result":"INCOMPLETE", "messages_scanned":0,
            "tf_edges":{}, "base_height_observations_m":[],
            "cloud_header_frames":{}, "cloud_stamp_samples":[],
            "status_type":None, "status_messages":0,
            "registration_tf_flag_true":0,
            "unknown_types":[], "no_ros_publications":True,
            "runtime_unique_tf_authority":"NOT_PROVEN"}
    if not (root/"metadata.yaml").exists():
        output["reason"]="metadata.yaml missing"
        return output
    try:
        import rosbag2_py
        import yaml
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        meta=yaml.safe_load((root/"metadata.yaml").read_text(encoding="utf-8"))
        storage_id=meta.get("rosbag2_bagfile_information",{}).get("storage_identifier","sqlite3")
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(root),storage_id=storage_id),
                    rosbag2_py.ConverterOptions(
                        input_serialization_format="cdr",output_serialization_format="cdr"))
        topic_types={t.name:t.type for t in reader.get_all_topics_and_types()}
        output["status_type"]=topic_types.get("/localization_status")
        classes={}
        for name in WATCHED:
            typename=topic_types.get(name)
            if not typename:
                continue
            try:
                classes[name]=get_message(typename)
            except Exception:
                output["unknown_types"].append({"topic":name,"type":typename})
        while reader.has_next() and output["messages_scanned"] < limit:
            topic,raw,_=reader.read_next()
            output["messages_scanned"]+=1
            cls=classes.get(topic)
            if cls is None:
                continue
            if topic=="/localization_status":
                output["status_messages"]+=1
                if output["status_messages"]>100:
                    continue
                msg=deserialize_message(raw,cls)
                # Only validate exact reported boolean; no guessed FSM field names.
                if getattr(msg,"registration_output_tf_published",None) is True:
                    output["registration_tf_flag_true"]+=1
                continue
            if topic=="/livox/lidar/geo_filtered":
                if sum(output["cloud_header_frames"].values())>=100:
                    continue
                msg=deserialize_message(raw,cls)
                frame=msg.header.frame_id
                output["cloud_header_frames"][frame]=output["cloud_header_frames"].get(frame,0)+1
                if len(output["cloud_stamp_samples"])<5:
                    output["cloud_stamp_samples"].append({
                        "sec":msg.header.stamp.sec,"nanosec":msg.header.stamp.nanosec})
                continue
            if topic in ("/tf", "/tf_static"):
                msg=deserialize_message(raw,cls)
                for t in msg.transforms:
                    key=t.header.frame_id.lstrip("/")+"->"+t.child_frame_id.lstrip("/")
                    output["tf_edges"][key]=output["tf_edges"].get(key,0)+1
                    if key=="base_footprint->base_link":
                        z=round(t.transform.translation.z,5)
                        if z not in output["base_height_observations_m"]:
                            output["base_height_observations_m"].append(z)
            # /robot_pose metadata type is sufficient for now.
        zvals=output["base_height_observations_m"]
        output["contains_old_0_30_height"]=any(abs(z-.3)<.02 for z in zvals)
        output["contains_selected_0_90_height"]=any(abs(z-.9)<.02 for z in zvals)
        output["recorded_conflicting_height_values"]=len(zvals)>1
        output["result"]="EVIDENCE_COLLECTED" if output["messages_scanned"] else "EMPTY_BAG"
        output["interpretation"]="Bag values describe a past session, not current runtime publisher authority."
    except Exception as exc:
        output["reason"]=type(exc).__name__+": "+str(exc)
        output["result"]="PARTIAL_OR_UNAVAILABLE"
    return output


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--bag",required=True)
    parser.add_argument("--output",required=True)
    parser.add_argument("--max-messages",type=int,default=120000)
    args=parser.parse_args()
    if not 100 <= args.max_messages <= 1000000:
        parser.error("invalid scan bound")
    data=report(args.bag,args.max_messages)
    path=Path(args.output)
    path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    tmp.replace(path)
    print("Bag evidence",data["result"],"messages_scanned",data["messages_scanned"])
    return 0 if data["result"]=="EVIDENCE_COLLECTED" else 1


if __name__=="__main__":
    raise SystemExit(main())
