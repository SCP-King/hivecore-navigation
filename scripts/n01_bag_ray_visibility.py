#!/usr/bin/env python3
"""Reconstruct observational 2D ray visibility from recorded Bag and PCD bounds.

READ ONLY. No ros2 bag play, ROS node, DDS, TF publisher or chassis command.
This tests a near-horizontal lidar slice; it DOES NOT establish safe 2D
navigation or full-height free volume.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"/"n01_offline_integration"))

from n01_offline_integration.bag_ray_visibility import RayVisibilityGrid
from n01_offline_integration.historical_tf import (
    HistoricalMapTf, stamp_to_ns, mapped_mounts, rotate)
from n01_offline_integration.pcd_static_preview import sample_pcd
from n01_offline_integration.pcd_goal_grid import save_outputs

CLOUD="/livox/lidar/geo_filtered"


def _reader(bag, topics):
    import rosbag2_py
    import yaml
    meta=yaml.safe_load((bag/"metadata.yaml").read_text(encoding="utf-8")) or {}
    storage=meta.get("rosbag2_bagfile_information",{}).get("storage_identifier","sqlite3")
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=storage),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                            output_serialization_format="cdr"))
    names={t.name:t.type for t in reader.get_all_topics_and_types()}
    required={
        "/tf":"tf2_msgs/msg/TFMessage",
        CLOUD:"sensor_msgs/msg/PointCloud2",
    }
    for name, expected in required.items():
        if names.get(name)!=expected:
            raise ValueError(f"Missing or unexpected recorded topic type: {name}")
    available=[t for t in topics if t in names]
    reader.set_filter(rosbag2_py.StorageFilter(topics=available))
    return reader


def _tf_history(bag):
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    hist=HistoricalMapTf(max_age_s=1.0,max_interval_s=2.)
    reader=_reader(bag,["/tf","/tf_static"])
    mount_values=set()
    while reader.has_next():
        topic,raw,_=reader.read_next()
        if topic not in ("/tf","/tf_static"):
            continue
        msg=deserialize_message(raw,TFMessage)
        for transform in msg.transforms:
            parent=transform.header.frame_id.lstrip("/")
            child=transform.child_frame_id.lstrip("/")
            t=transform.transform.translation
            q=transform.transform.rotation
            if parent=="map" and child=="base_footprint":
                hist.add(stamp_to_ns(transform.header.stamp),
                         (t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
            elif parent=="base_footprint" and child=="base_link" and math.isfinite(t.z):
                mount_values.add(round(t.z,5))
    hist.finish()
    if not hist.times:
        raise ValueError("Bag has no valid stamped map->base_footprint historical TF")
    return hist, sorted(mount_values)


def _cloud_points(msg, stride, limit=300000):
    """Validate XYZ offsets and organized PointCloud2 binary layout."""
    n=msg.width*msg.height
    if (n<0 or n>limit or msg.point_step<12 or
            msg.row_step<msg.point_step*msg.width or
            msg.row_step*msg.height>len(msg.data)):
        raise ValueError("invalid or oversized cloud binary layout")
    fields={}
    for field in msg.fields:
        if field.name in ("x","y","z"):
            if (field.name in fields or field.datatype!=7 or field.count!=1 or
                    field.offset<0 or field.offset+4>msg.point_step):
                raise ValueError("invalid XYZ field layout")
            fields[field.name]=field.offset
    if set(fields)!={"x","y","z"} or len(set(fields.values()))!=3:
        raise ValueError("missing or overlapping XYZ fields")
    fmt=struct.Struct((">" if msg.is_bigendian else "<")+"f")
    for index in range(0,n,stride):
        row,col=divmod(index,msg.width)
        base=row*msg.row_step+col*msg.point_step
        xyz=tuple(fmt.unpack_from(msg.data,base+fields[k])[0] for k in ("x","y","z"))
        if all(math.isfinite(v) for v in xyz):
            yield xyz


def reconstruct(bag, pcd, output, *, resolution=.25, max_frames=928,
                point_stride=16, min_free_rays=3, max_range=5.,
                max_vertical_delta=.35, clearance=.5,
                static_z_band=None):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    if (type(max_frames) is not int or not 1<=max_frames<=2000 or
            type(point_stride) is not int or not 1<=point_stride<=200 or
            not (bag/"metadata.yaml").is_file()):
        raise ValueError("invalid Bag path or bounded replay sampling")
    if static_z_band is not None and (
            len(static_z_band)!=2 or
            not all(math.isfinite(z) for z in static_z_band) or
            static_z_band[0]>=static_z_band[1]):
        raise ValueError("invalid manually selected PCD obstacle height band")
    # In this release PCD establishes the exact output-map XY extent.
    # Static collision geometry requires an operator-selected Z band.
    pcd_points,pcd_meta=sample_pcd(pcd)
    bounds=[(min(p[i] for p in pcd_points),max(p[i] for p in pcd_points))
            for i in range(2)]
    evidence=RayVisibilityGrid(bounds,resolution=resolution,
                 max_range_m=max_range,max_vertical_delta_m=max_vertical_delta,
                 min_free_rays=min_free_rays)
    history,mounts=_tf_history(bag)
    reader=_reader(bag,[CLOUD])
    stats=dict(cloud_frames_seen=0,cloud_frames_processed=0,
               rejected_frame_id=0,rejected_no_matching_tf=0,
               rejected_cloud_layout=0,points_sampled=0,
               recorded_tf_samples=len(history.times),
               historical_mount_heights=mounts)
    while reader.has_next() and stats["cloud_frames_seen"]<max_frames:
        topic,raw,_=reader.read_next()
        if topic!=CLOUD:
            continue
        stats["cloud_frames_seen"]+=1
        msg=deserialize_message(raw,PointCloud2)
        if msg.header.frame_id.lstrip("/")!="lidar_link":
            stats["rejected_frame_id"]+=1
            continue
        tf=history.lookup(stamp_to_ns(msg.header.stamp))
        if tf is None:
            stats["rejected_no_matching_tf"]+=1
            continue
        _,sensor=mapped_mounts(tf)
        origin,quaternion=sensor
        try:
            samples=list(_cloud_points(msg,point_stride))
        except (ValueError,struct.error,ZeroDivisionError):
            stats["rejected_cloud_layout"]+=1
            continue
        for xyz in samples:
            rotated=rotate(quaternion,xyz)
            endpoint=tuple(origin[k]+rotated[k] for k in range(3))
            evidence.ray(origin,endpoint)
        stats["points_sampled"]+=len(samples)
        stats["cloud_frames_processed"]+=1
    if not stats["cloud_frames_processed"] or not evidence.stats["rays_accepted"]:
        raise ValueError("no time-aligned near-horizontal rays; refusing fabricated free map")
    obstacles=[]
    if static_z_band:
        a,b=static_z_band
        obstacles=[(p[0],p[1]) for p in pcd_points if a<=p[2]<=b]
    m=evidence.project(static_obstacle_xy=obstacles,clearance_m=clearance)
    # Keep input evidence distinct from verified traversability.
    for suffix in (".pgm",".yaml",".svg",".json"):
        if output.with_suffix(suffix).exists():
            raise FileExistsError(f"refuse overwrite: {output.with_suffix(suffix)}")
    summary=save_outputs(m,output)
    summary.update({
        "schema":"n01.offline.bag_ray_visibility.v1",
        "status":"RAY_LEVEL_VISIBILITY_ONLY_NOT_TRAVERSABILITY",
        "source_pcd":str(pcd.resolve()),
        "source_bag":str(bag.resolve()),
        "pcd_points":pcd_meta["count"],
        "static_pcd_obstacle_z_band_map_m":static_z_band,
        "pcd_used_for_xy_bounds":True,
        "pcd_used_for_obstacles":bool(static_z_band),
        "bag_cloud_stats":stats,
        "historical_tf_is_not_localization_proof":True,
        "livox_mount_used_m":"historical TF with replay-only height 0.90 and x 0.295",
        "ray_free_does_not_prove_ground_or_robot_volume":True,
        "unknown_never_auto_freed":True,
        "global_reference_algorithm":"NONE_NO_ROUTE_COMPUTED",
        "scan_native_planner_connected":False,
        "robot_control_enabled":False,
        "safety_authorization":"NEVER",
        "actuator_output":"DISABLED",
    })
    output.with_suffix(".json").write_text(
        json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+"\n",
        encoding="utf-8")
    return summary


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag",type=Path,required=True)
    p.add_argument("--pcd",type=Path,required=True)
    p.add_argument("--output-prefix",type=Path,required=True)
    p.add_argument("--resolution",type=float,default=.25)
    p.add_argument("--max-frames",type=int,default=928)
    p.add_argument("--point-stride",type=int,default=16)
    p.add_argument("--max-range",type=float,default=5.)
    p.add_argument("--max-vertical-delta",type=float,default=.35)
    p.add_argument("--min-free-rays",type=int,default=3)
    p.add_argument("--clearance",type=float,default=.5)
    p.add_argument("--pcd-obstacle-z-min",type=float)
    p.add_argument("--pcd-obstacle-z-max",type=float)
    args=p.parse_args(argv)
    if (args.pcd_obstacle_z_min is None)!=(args.pcd_obstacle_z_max is None):
        p.error("Both --pcd-obstacle-z-min and --pcd-obstacle-z-max required together")
    band=None
    if args.pcd_obstacle_z_min is not None:
        band=[args.pcd_obstacle_z_min,args.pcd_obstacle_z_max]
    result=reconstruct(args.bag,args.pcd,args.output_prefix,
              resolution=args.resolution,max_frames=args.max_frames,
              point_stride=args.point_stride,max_range=args.max_range,
              max_vertical_delta=args.max_vertical_delta,
              min_free_rays=args.min_free_rays,clearance=args.clearance,
              static_z_band=band)
    print("OFFLINE BAG RAY VISIBILITY MAP: NOT A NAVIGATION MAP")
    print("processed_clouds:",result["bag_cloud_stats"]["cloud_frames_processed"])
    print("accepted_rays:",result["ray_count_stats"]["rays_accepted"])
    print("free_cells:",result["free_cells"],
          "unknown_cells:",result["unknown_cells"],
          "occupied_cells:",result["inflated_occupied_cells"])
    print("PCD obstacle band:",band,"(None means PCD used for XY extent only)")
    print("MAP:",args.output_prefix.with_suffix(".pgm"))
    print("VIEW:",args.output_prefix.with_suffix(".svg"))
    print("REPORT:",args.output_prefix.with_suffix(".json"))
    print("No SCAN trajectory, ROS topics, or chassis commands generated.")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
