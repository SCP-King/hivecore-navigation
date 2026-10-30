#!/usr/bin/env python3
"""G4.8 *read-only*: first accepted historical Bag scan pose -> map-anchored A/B.

Uses the exact TF interpolation and map-frame height override of the existing
historical Bag SCAN replay. Requires recorded /tf and Livox PointCloud2.
Does NOT assert a floor Z, create a 3D SCAN reference, launch ROS nodes, or
open any robot control interface. All route cells are lidar-height visibility,
not driveable terrain.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "n01_offline_integration"))

from n01_offline_integration.bag_anchored_goal import choose_research_goal
from n01_offline_integration.core import xyz_transform_bytes
from n01_offline_integration.historical_tf import (
    HistoricalMapTf, stamp_to_ns, mapped_mounts,
    SELECTED_MOUNT_HEIGHT_M,
)
from n01_offline_integration.pcd_bag_evidence_audit import audit, write_outputs
from n01_offline_integration.pcd_static_preview import sample_pcd
from n01_offline_integration.pcd_route_height_profile import profile_route_surfaces
from n01_offline_integration.g410_route_surface_ambiguity import audit_surface_ambiguity
from n01_offline_integration.g411_mode3_z_alignment import check_mode3_z_alignment


def make_reader(bag_dir, topics):
    import rosbag2_py
    import yaml

    meta = yaml.safe_load((bag_dir / "metadata.yaml").read_text(encoding="utf-8"))
    storage_id = meta.get("rosbag2_bagfile_information", {}).get("storage_identifier", "sqlite3")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    required = {
        "/tf": "tf2_msgs/msg/TFMessage",
        "/livox/lidar/geo_filtered": "sensor_msgs/msg/PointCloud2",
    }
    for topic, expected in required.items():
        if types.get(topic) != expected:
            raise ValueError("Bag missing or wrong type: " + topic)
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader


def first_accepted_scan_pose(bag_dir, *, max_age_s=1.0):
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import PointCloud2

    tf = HistoricalMapTf(max_age_s=max_age_s, max_interval_s=2*max_age_s)
    reader = make_reader(bag_dir, ("/tf", "/tf_static"))
    messages = 0
    heights = set()
    mount_by_topic = {
        "/tf": {"counts_by_z_m": {}, "first_tf_stamp_ns": None,
                "last_tf_stamp_ns": None},
        "/tf_static": {"counts_by_z_m": {}, "first_tf_stamp_ns": None,
                       "last_tf_stamp_ns": None},
    }
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        msg = deserialize_message(raw, TFMessage)
        messages += 1
        if messages > 1_000_000:
            raise ValueError("Bag contains excessive TF messages")
        for tr in msg.transforms:
            parent = tr.header.frame_id.lstrip("/")
            child = tr.child_frame_id.lstrip("/")
            if parent == "base_footprint" and child == "base_link":
                z = float(tr.transform.translation.z)
                if -100 < z < 100:
                    rounded=round(z,5)
                    heights.add(rounded)
                    channel=mount_by_topic[topic]
                    key=f"{rounded:.5f}"
                    channel["counts_by_z_m"][key]=channel["counts_by_z_m"].get(key,0)+1
                    t_ns=stamp_to_ns(tr.header.stamp)
                    if channel["first_tf_stamp_ns"] is None or t_ns < channel["first_tf_stamp_ns"]:
                        channel["first_tf_stamp_ns"]=t_ns
                    if channel["last_tf_stamp_ns"] is None or t_ns > channel["last_tf_stamp_ns"]:
                        channel["last_tf_stamp_ns"]=t_ns
            if parent == "map" and child == "base_footprint":
                p, q = tr.transform.translation, tr.transform.rotation
                tf.add(stamp_to_ns(tr.header.stamp),
                       (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
    tf.finish()
    if not tf.times:
        raise ValueError("NO_MAP_BASE_FOOTPRINT_TF_IN_RECORDED_BAG")
    reader = make_reader(bag_dir, ("/livox/lidar/geo_filtered",))
    counters = {"examined_clouds": 0, "wrong_frame": 0,
                "no_valid_timestamped_map_tf": 0, "invalid_cloud_layout": 0}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != "/livox/lidar/geo_filtered":
            continue
        counters["examined_clouds"] += 1
        if counters["examined_clouds"] > 2000:
            raise ValueError("no acceptable scan within 2000 historical clouds")
        msg = deserialize_message(raw, PointCloud2)
        if msg.header.frame_id.lstrip("/") != "lidar_link":
            counters["wrong_frame"] += 1
            continue
        stamp_ns = stamp_to_ns(msg.header.stamp)
        pose = tf.lookup(stamp_ns)
        if pose is None:
            counters["no_valid_timestamped_map_tf"] += 1
            continue
        body, lidar = mapped_mounts(pose)
        mapping_body, mapping_lidar = mapped_mounts(pose, mount_height_m=0.30)
        try:
            xyz_transform_bytes(
                msg.data, msg.width, msg.height, msg.point_step, msg.row_step,
                [(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
                msg.is_bigendian, lidar[0], lidar[1], 300000)
        except (ValueError, TypeError, OverflowError):
            counters["invalid_cloud_layout"] += 1
            continue
        return {
            "schema": "n01.offline.first_accepted_bag_pose.v1",
            "bag_source": str(bag_dir.resolve()),
            "accepted_filtered_cloud_index_1_based": counters["examined_clouds"],
            "original_cloud_stamp_ns": stamp_ns,
            "source_cloud_frame": "lidar_link",
            "map_to_base_footprint_xyz_m": list(pose[0]),
            "map_to_base_footprint_xy_m": list(pose[0][:2]),
            # Existing 0.90m replay geometry is RETAINED as a legacy
            # comparison; it is not the proven mapping-time TF convention.
            "offline_reconstructed_base_link_xyz_m": list(body[0]),
            "offline_reconstructed_lidar_xyz_m": list(lidar[0]),
            "legacy_selected_mount_height_m": SELECTED_MOUNT_HEIGHT_M,
            "historical_mapping_candidate_mount_height_m": 0.30,
            "historical_mapping_candidate_base_link_xyz_m": list(mapping_body[0]),
            "historical_mapping_candidate_lidar_xyz_m": list(mapping_lidar[0]),
            "tf_mount_edge_evidence_by_topic": mount_by_topic,
            "historical_mapping_authority_verified_inside_slam_code": False,
            "timestamped_tf_nearest_sample_age_s": pose[2],
            "historic_tf_samples": len(tf.times),
            "historical_mount_z_values_m": sorted(heights),
            "historical_mount_conflict": len(heights) > 1,
            "replay_mount_override_z_m": SELECTED_MOUNT_HEIGHT_M,
            "replay_frame_not_slam_localization_tracking": True,
            "cloud_counters_before_accepted": counters,
            "actual_ground_z_known": False,
            "frame_alignment": "OPERATOR_REPORTED_SAME_MAPPING_SESSION_NOT_INDEPENDENTLY_REGISTERED",
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
    raise ValueError("NO_ACCEPTED_FILTERED_CLOUD_WITH_VALID_MAP_TF")


def surface_z_evidence(pcd_path, xy, radius_m=1.0):
    points, meta = sample_pcd(pcd_path)
    values = sorted(p[2] for p in points
                    if (p[0]-xy[0])**2 + (p[1]-xy[1])**2 <= radius_m**2)
    def quantile(frac):
        if not values:
            return None
        return round(values[round((len(values)-1)*frac)], 4)
    return {
        "source_pcd": str(pcd_path.resolve()),
        "pcd_header_points": meta["count"],
        "pcd_sampled_points": meta["sampled_points"],
        "search_xy_m": xy,
        "search_radius_m": radius_m,
        "nearby_sample_count": len(values),
        "nearby_surface_z_quantiles_m": {
            "min": quantile(0.), "p10": quantile(.1),
            "p25": quantile(.25), "median": quantile(.5),
            "p75": quantile(.75), "p90": quantile(.9),
            "max": quantile(1.),
        },
        "interpretation": "XYZ surface samples only; could include equipment, floor, walls",
        "ground_z_estimate_selected": None,
        "ground_z_verified": False,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--ray-prefix", type=Path, required=True)
    parser.add_argument("--pcd-prefix", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    if not (args.bag / "metadata.yaml").is_file() or not args.pcd.is_file():
        parser.error("require existing historical rosbag2 metadata.yaml and original PCD")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    manifest = {
        "schema": "n01.offline.g48_bag_anchored_preflight.v1",
        "result": "INCOMPLETE",
        "stage": "start",
        "ground_z_map_m": None,
        "ground_verified": False,
        "pcd_bag_same_map": "OPERATOR_CONFIRMED_NOT_AUTOMATICALLY_PROVEN",
        "route_to_scan_executed": False,
        "actuator_output": "DISABLED",
        "safety_authorization": "NEVER",
    }
    try:
        manifest["stage"] = "first_accepted_bag_pose"
        pose = first_accepted_scan_pose(args.bag)
        manifest["first_accepted_bag_pose"] = pose
        # G4.11: read-only numerical "what if map Z=0" check against the
        # REAL SCAN mode3 pathCallback and existing G4 vertical-offset gate.
        # This does not select ground, change SCAN params or unlock launch.
        height_check = check_mode3_z_alignment(manifest)
        with (args.output_dir / "g411_scan_mode3_height_alignment.json").open(
                "x", encoding="utf-8") as height_file:
            json.dump(height_check, height_file, ensure_ascii=False,
                      indent=2, allow_nan=False)
            height_file.write("\n")
        manifest["g411_scan_mode3_height_alignment"] = (
            "g411_scan_mode3_height_alignment.json")
        manifest["g411_z0_hypothesis_would_meet_reference_gate"] = (
            height_check["would_pass_reference_publisher_z_gate_on_these_recorded_values"])
        manifest["stage"] = "pcd_surface_evidence"
        manifest["local_pcd_surfaces"] = surface_z_evidence(
            args.pcd, pose["map_to_base_footprint_xy_m"])
        manifest["stage"] = "anchored_goal_selection"
        _, grid, _, _ = audit(args.ray_prefix, args.pcd_prefix)
        selection = choose_research_goal(
            grid, pose["map_to_base_footprint_xy_m"])
        manifest["selected_research_goal"] = selection
        manifest["stage"] = "g4_3_bag_anchored_research_path"
        evidence, grid, path, conflicts = audit(
            args.ray_prefix, args.pcd_prefix,
            start_xy=pose["map_to_base_footprint_xy_m"],
            goal_xy=selection["goal_xy_m"])
        evidence["selection"] = "FIRST_ACCEPTED_BAG_SCAN_ANCHORED_RESEARCH"
        evidence["first_accepted_cloud_stamp_ns"] = pose["original_cloud_stamp_ns"]
        evidence["first_accepted_bag_pose_xy_m"] = pose["map_to_base_footprint_xy_m"]
        write_outputs(args.output_dir / "g43_evidence", evidence, grid, path, conflicts)
        if not path:
            raise ValueError("anchored G4.3 A*: " +
                             evidence.get("route_error", "no path"))
        # PCD Z along EVERY real-Bag-anchored XY waypoint. Height bands are
        # measured surfaces only; never infer "floor" from a low percentile.
        manifest["stage"] = "pcd_route_surface_height_profile"
        xyz, _ = sample_pcd(args.pcd)
        surface_profile = profile_route_surfaces(
            xyz, evidence["research_path_xy_m"])
        with (args.output_dir / "g49_surface_height_profile.json").open(
                "x", encoding="utf-8") as profile_file:
            json.dump(surface_profile, profile_file, ensure_ascii=False,
                      indent=2, allow_nan=False)
            profile_file.write("\n")
        manifest["g49_surface_height_profile"] = "g49_surface_height_profile.json"
        manifest["g49_waypoints_without_nearby_pcd_samples"] = (
            surface_profile["waypoints_without_nearby_pcd_samples"])
        # G4.10: triage missing / vertically ambiguous surface samples
        # along the SAME XY route. NEVER pick a lowest Z bin as floor.
        manifest["stage"] = "g410_surface_ambiguity_audit"
        ambiguity = audit_surface_ambiguity(surface_profile)
        with (args.output_dir / "g410_route_surface_ambiguity.json").open(
                "x", encoding="utf-8") as ambiguity_file:
            json.dump(ambiguity, ambiguity_file, ensure_ascii=False,
                      indent=2, allow_nan=False)
            ambiguity_file.write("\n")
        manifest["g410_route_surface_ambiguity"] = (
            "g410_route_surface_ambiguity.json")
        manifest["g410_waypoints_multi_height_or_high_z_variation"] = (
            ambiguity["waypoints_multi_height_or_high_z_variation"])
        manifest["g410_route_to_scan_allowed"] = False
        manifest["g43_evidence"] = "g43_evidence.json"
        manifest["g43_preview"] = "g43_evidence.svg"
        manifest["result"] = "BAG_ANCHORED_XY_RESEARCH_ROUTE_READY_Z_UNKNOWN"
        manifest["stage"] = "ready_for_floor_z_review_not_scan"
        manifest["route_length_m"] = evidence["research_path_length_m"]
        manifest["path_waypoints"] = len(path)
        manifest["next_gate"] = (
            "Ground Z and planner body-height/real robot pose frame must be "
            "validated before producing 3D SCAN reference or claiming traversability")
    except (ValueError, RuntimeError, OSError, KeyError, TypeError) as exc:
        manifest["result"] = "BLOCKED"
        manifest["reason"] = str(exc)
    with (args.output_dir / "g48_preflight.json").open("x", encoding="utf-8") as out:
        json.dump(manifest, out, ensure_ascii=False, indent=2, allow_nan=False)
        out.write("\n")
    print("G4.8:", manifest["result"])
    print("Evidence:", args.output_dir / "g48_preflight.json")
    if manifest.get("reason"):
        print("Blocker:", manifest["reason"])
    return 0 if manifest["result"].startswith("BAG_ANCHORED") else 3


if __name__ == "__main__":
    raise SystemExit(main())
