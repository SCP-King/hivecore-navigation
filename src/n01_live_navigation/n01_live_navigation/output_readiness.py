"""Readiness refers to the successful published scan, not a later attempt."""
def compensated_output_ready(snapshot, processed_stamp, output_current,
                             pose_fresh, odom_fresh, prediction_fault):
    return bool(isinstance(snapshot,dict) and processed_stamp
        and snapshot.get('scan_stamp_ns')==processed_stamp
        and snapshot.get('extrinsic_verified') is True
        and snapshot.get('anchor_stamp_ns') is not None
        and output_current and pose_fresh and odom_fresh and not prediction_fault)
