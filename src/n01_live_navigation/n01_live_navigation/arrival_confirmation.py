"""A predicted pose can stop a robot, but cannot alone confirm arrival."""
import math


def prediction_metadata(health):
    return (health or {}).get('bridge',{}).get('pose_prediction')


def measurement_confirms(health, minimum_stamp_ns, xy=None, yaw=None,
                         pos_tolerance_m=0.10, yaw_tolerance_rad=math.radians(5.0)):
    prediction=prediction_metadata(health)
    # Legacy profiles use direct fresh measured poses and have no predictor.
    if prediction is None:return True
    try:
        if prediction.get('fault') or prediction['source_stamp_ns']<minimum_stamp_ns:return False
        pose=prediction['source_pose']
        if xy is not None:
            if not math.hypot(pose['x']-xy[0],pose['y']-xy[1])<=pos_tolerance_m:return False
        if yaw is not None:
            x,y,z,w=pose['quaternion']
            actual=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            if not abs(math.atan2(math.sin(actual-yaw),math.cos(actual-yaw)))<=yaw_tolerance_rad:return False
        return True
    except (ValueError,TypeError,KeyError,IndexError):return False
