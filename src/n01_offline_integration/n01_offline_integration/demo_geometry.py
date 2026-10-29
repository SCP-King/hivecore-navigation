"""Pure-Python geometry for the read-only historical N01 demo.

Only hypothesis visuals, not detection truth or authorized control. This module
has no ROS/robot dependency so it can be regression-tested on any computer.
"""
import math


def finite_xy(v):
    return (isinstance(v,(list,tuple)) and len(v)>=2
            and all(isinstance(x,(int,float)) and not isinstance(x,bool)
                    and math.isfinite(x) for x in v[:2]))


def preview_tracks(message,max_tracks=20):
    """Strictly accept processed map-frame moving *hypotheses*."""
    if (not isinstance(message,dict)
            or message.get("schema")!="n01.offline.tracks.v1"
            or message.get("frame_id")!="map"
            or message.get("status")!="PROCESSED"
            or type(message.get("stamp_ns")) is not int):
        return []
    rows=message.get("tracks")
    if not isinstance(rows,list):
        return []
    clean=[]
    for row in rows:
        if not isinstance(row,dict):
            continue
        if (row.get("classification")!="MOVING_HYPOTHESIS_UNVERIFIED"
                or row.get("motion_consistency_passed") is not True
                or type(row.get("hits")) is not int or row["hits"]<4):
            continue
        p=row.get("position_map_m")
        v=row.get("velocity_xy_mps")
        ext=row.get("extent_xy_m")
        if not all(finite_xy(a) for a in (p,v,ext)):
            continue
        if max(abs(v[0]),abs(v[1]))>4. or min(ext[:2])<0. or max(ext[:2])>3.:
            continue
        clean.append({
            "id":str(row.get("id","?"))[:16],
            "xy":(float(p[0]),float(p[1])),
            "velocity":(float(v[0]),float(v[1])),
            "extent":(max(.15,float(ext[0])),max(.15,float(ext[1]))),
        })
        if len(clean)>=max_tracks:
            break
    return clean


def predict_xy(xy,velocity,horizon_s=1.5,dt_s=.25):
    if not finite_xy(xy) or not finite_xy(velocity):
        return []
    if not (.1<=dt_s<=.5 and .1<=horizon_s<=3.):
        return []
    steps=int(math.ceil(horizon_s/dt_s))
    return [(xy[0]+velocity[0]*min(horizon_s,k*dt_s),
             xy[1]+velocity[1]*min(horizon_s,k*dt_s))
            for k in range(steps+1)]


def reference_result(message):
    """Return sanitized G3 drawing inputs; drop heartbeats and unsolved frames."""
    if (not isinstance(message,dict)
            or message.get("schema")!="n01.offline.g3_mpc_cbf.v1"
            or message.get("is_new_solver_input") is not True
            or message.get("solver_executed") is not True
            or message.get("safety_authorization")!="NEVER"
            or message.get("actuator_output")!="DISABLED"):
        return None
    stamp=message.get("recorded_scan_stamp_ns")
    if type(stamp) is not int or stamp<=0:
        return None
    tracks=message.get("moving_constraint_tracks")
    if type(tracks) is not int or tracks<0:
        return None
    nominal=message.get("nominal_xy_m")
    candidate=message.get("candidate_xy_m")
    if (not isinstance(nominal,list) or not isinstance(candidate,list)
            or not 2<=len(nominal)<=30 or not 2<=len(candidate)<=30
            or any(not finite_xy(p) for p in nominal+candidate)):
        return None
    pose=message.get("model_input_pose_map_xyz_m")
    z=float(pose[2]) if (isinstance(pose,(list,tuple)) and len(pose)>=3 and
                         isinstance(pose[2],(int,float)) and
                         math.isfinite(pose[2])) else .9
    return {
        "stamp":stamp,
        "nominal":[(float(p[0]),float(p[1])) for p in nominal],
        "candidate":[(float(p[0]),float(p[1])) for p in candidate],
        "z":z,
        "posthoc":message.get("post_hoc_reference_comparison") is True,
        "dynamic_constraints":tracks,
        "status":str(message.get("status","UNKNOWN"))[:120],
        "nominal_h":message.get("minimum_nominal_h_m2"),
        "candidate_h":message.get("minimum_candidate_h_m2"),
    }
