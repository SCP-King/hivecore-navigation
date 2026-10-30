"""G4.12: exact SE(3) map-vs-Hive2 mount compatibility diagnostic, read-only.

The archived mapping session contains both /tf base_footprint->base_link=0.30
and /tf_static =0.90. The same stored map/PCD must not be silently paired
with a new +0.90 mount and unadjusted map->base_footprint pose.

This is a coordinate identity calculation, NOT a request to change SLAM's
localization TF, set map ground Z or move any wheels.
"""
import math
from .historical_tf import mapped_mounts, rotate, normalize

SCHEMA="n01.offline.g412_historical_mapping_mount_alignment.v1"
HISTORICAL_CANDIDATE_M=.30
HIVE2_MOTION_STACK_M=.90
LEGACY_REPLAY_SELECTED_M=.90


def audit_mapping_mount_alignment(preflight):
    """Exact robot mount-delta compensation preserving SAME map-frame lidar.

    T_map_lidar_original = T_map_bf * T_bf_bl(.30) * T_bl_lidar(.295,0,0).
    If we switch body mounting to .90 while requiring the same map LiDAR,
    T_map_bf_new = T_map_bf_old * T_bf_bl(-.60) for fixed orientation.
    That correction may have x/y components for pitched/rolled frames.
    No TF is published: the output is an engineering diagnosis only.
    """
    if not isinstance(preflight,dict) or preflight.get("schema") != (
            "n01.offline.g48_bag_anchored_preflight.v1"):
        raise ValueError("requires G4.8 preflight source schema")
    p=preflight.get("first_accepted_bag_pose")
    if not isinstance(p,dict):
        raise ValueError("missing first accepted historical Bag pose")
    xyz=p.get("map_to_base_footprint_xyz_m")
    q=p.get("map_to_base_footprint_quaternion_xyzw")
    if (not isinstance(xyz,list) or len(xyz)!=3 or
        any(type(v) not in (int,float) or not math.isfinite(v) for v in xyz)):
        raise ValueError("missing finite historical map->base_footprint XYZ")
    if (not isinstance(q,list) or len(q)!=4 or
        any(type(v) not in (int,float) or not math.isfinite(v) for v in q)):
        raise ValueError("missing finite map->base_footprint orientation")
    q=normalize(tuple(q))
    old_body,old_lidar=mapped_mounts((tuple(xyz),q),mount_height_m=.30)
    current_body,current_lidar=mapped_mounts((tuple(xyz),q),mount_height_m=.90)
    original_lidar_map=old_lidar[0]
    wrong_delta=tuple(current_lidar[0][i]-original_lidar_map[i]
                      for i in range(3))
    inverse_delta=rotate(q,(0.,0.,HISTORICAL_CANDIDATE_M-HIVE2_MOTION_STACK_M))
    translated=tuple(xyz[i]+inverse_delta[i] for i in range(3))
    adjusted_body,adjusted_lidar=mapped_mounts(
        (translated,q),mount_height_m=HIVE2_MOTION_STACK_M)
    residual=math.dist(original_lidar_map,adjusted_lidar[0])
    if residual>1e-9:
        raise ValueError("mount composition discrepancy")
    return {
        "schema":SCHEMA,
        "status":"HISTORICAL_MAP_VS_MOTION_STACK_MOUNT_CONFLICT_REQUIRES_FRAME_RECONCILIATION",
        "tf_history_input":"USER_REPORTED_DYNAMIC_0P30_AND_STATIC_0P90_WITH_PCD_SUPPORT",
        "recorded_map_to_base_footprint_xyz_m":list(xyz),
        "recorded_map_to_base_footprint_quaternion_xyzw":list(q),
        "historical_mapping_candidate_mount_z_m":HISTORICAL_CANDIDATE_M,
        "legacy_replay_and_hive2_motion_mount_z_m":HIVE2_MOTION_STACK_M,
        "historical_candidate_map_base_link_xyz_m":list(old_body[0]),
        "historical_candidate_map_lidar_xyz_m":list(original_lidar_map),
        "legacy_hive2_mount_unmodified_map_base_link_xyz_m":list(current_body[0]),
        "legacy_hive2_mount_unmodified_map_lidar_xyz_m":list(current_lidar[0]),
        "legacy_lidar_position_shift_magnitude_m":round(math.dist(
            original_lidar_map,current_lidar[0]),9),
        "legacy_lidar_position_shift_map_xyz_m":list(wrong_delta),
        "hypothetical_corrected_map_base_footprint_xyz_m":list(translated),
        "hypothetical_corrected_lidar_xyz_m":list(adjusted_lidar[0]),
        "hypothetical_correction_map_xyz_m":list(inverse_delta),
        "hypothetical_new_lidar_residual_m":round(residual,12),
        "correction_is_approx_negative_0p60_z_if_level":True,
        "correction_can_include_xy_if_tilted":True,
        "automatic_live_tf_adjustment_allowed":False,
        "map_or_pcd_shift_produced":False,
        "true_floor_z_map_verified":False,
        "local_floor_candidate_from_user_report_map_z_interval_m":[-.6,-.5],
        "floor_candidate_is_not_global_traversability":True,
        "original_slam_tf_query_authority_independently_verified":False,
        "four_wheel_steering_execution_verified":False,
        "physical_motion_authorized":False,
        "actuator_output":"DISABLED",
        "safety_authorization":"NEVER",
    }
