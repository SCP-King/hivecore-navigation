"""Shared policy: fresh control estimates may be built from delayed global fixes."""
import math

CONTROL_STAMP_AGE = .4
CONTROL_RECEIVE_AGE = .5
PREDICTION_SPEED = .03


def control_pose_fresh(stamp_age, receive_age, stamp_limit=CONTROL_STAMP_AGE, receive_limit=CONTROL_RECEIVE_AGE):
    return (isinstance(stamp_age,(float,int)) and isinstance(receive_age,(float,int))
            and math.isfinite(stamp_age) and math.isfinite(receive_age)
            and -.2 <= stamp_age <= stamp_limit
            and 0 <= receive_age <= receive_limit)


def tracking_mode(bridge):
    if (not isinstance(bridge,dict) or bridge.get('status')!='LIVE_SCAN_INPUTS_READY'
            or bridge.get('body_pose_live_fresh') is not True):
        return 'WAITING'
    p=bridge.get('pose_prediction')
    if not isinstance(p,dict) or p.get('fault'):
        return 'WAITING'
    if (bridge.get('robot_pose_stamp_age_s',float('inf')) > .7
            or bridge.get('robot_pose_raw_receive_fresh') is False):
        return 'PREDICTED_TRACKING'
    return 'MEASURED_TRACKING'
