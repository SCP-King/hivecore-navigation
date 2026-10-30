"""G4.11 read-only check of SCAN mode-3 Z semantics against recorded Bag.

NEVER infers floor, overrides a gate or publishes ROS topics/commands.
It deliberately performs no 3D reference generation.
"""
import math

SCHEMA="n01.offline.g411_scan_mode3_z_alignment_preflight.v1"
EXPECTED_G48="n01.offline.g48_bag_anchored_preflight.v1"


def check_mode3_z_alignment(preflight, *, assumed_reference_z_m=0.,
                            scan_body_height_m=.4,
                            max_start_vertical_offset_m=.5):
    """Inspect *only* geometry of a hypothesis; not traversability clearance.

    Upstream SCAN scan_replan_fsm.cpp pathCallback:
    wp(2) = pose_stamped.pose.position.z + body_height_;
    Research reference publisher compares body_z to path_z+body_height.
    """
    for x,bounds in ((assumed_reference_z_m,(-3.,4.)),
                     (scan_body_height_m,(.1,1.5)),
                     (max_start_vertical_offset_m,(.1,1.))):
        if type(x) not in (float,int) or not math.isfinite(x) or not bounds[0]<=x<=bounds[1]:
            raise ValueError("invalid height hypothesis or configured gate")
    if not isinstance(preflight,dict) or preflight.get("schema")!=EXPECTED_G48:
        raise ValueError("requires original G4.8 read-only Bag preflight")
    measured=preflight.get("first_accepted_bag_pose")
    if not isinstance(measured,dict):
        raise ValueError("G4.8 has no accepted timestamped historical pose")
    body=measured.get("offline_reconstructed_base_link_xyz_m")
    if (not isinstance(body,list) or len(body)!=3 or
            any(type(v) not in (float,int) or not math.isfinite(v) for v in body)):
        raise ValueError("G4.8 does not contain finite reconstructed base_link")
    if preflight.get("ground_verified") is not False:
        raise ValueError("cannot use this unverified-height research check as ground validation")
    projected=assumed_reference_z_m+scan_body_height_m
    error=abs(body[2]-projected)
    return {
        "schema":SCHEMA,
        "status":("REFERENCE_HEIGHT_GATE_DIFFERENCE_IN_CONFIG_RANGE_NOT_TERRAIN_VERIFIED"
                  if error <= max_start_vertical_offset_m else
                  "REFERENCE_HEIGHT_SEMANTICS_MISMATCH_FOR_HYPOTHESIS"),
        "source":"SOURCE_VERIFIED_SCAN_MODE3_WP_Z_PLUS_BODY_HEIGHT_AND_G4_REFERENCE_POSE_GATE",
        "height_input_type":"EXPLICIT_UNVERIFIED_MAP_Z_HYPOTHESIS",
        "reference_z_hypothesis_m":float(assumed_reference_z_m),
        "scan_body_height_config_m":float(scan_body_height_m),
        "recorded_reconstructed_base_link_z_m":float(body[2]),
        "scan_mode3_path_target_z_m":round(projected,6),
        "reference_start_vertical_offset_m":round(error,6),
        "reference_publisher_vertical_gate_limit_m":float(max_start_vertical_offset_m),
        "would_pass_reference_publisher_z_gate_on_these_recorded_values":error <= max_start_vertical_offset_m,
        "real_map_floor_z_verified":False,
        "scan_body_height_is_a_configured_planner_offset_not_floor_measurement":True,
        "true_4ws_and_whole_body_collision_verified":False,
        "route_to_scan_executed":False,
        "hardware_control_authorized":False,
        "actuator_output":"DISABLED",
        "safety_authorization":"NEVER",
    }
