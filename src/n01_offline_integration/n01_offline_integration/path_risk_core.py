"""Offline-only path-vs-moving-hypothesis clearance diagnostic.

Not a control-barrier-function solver: no control-affine model, physical robot
velocity, certified obstacle radius or verified timestamp synchronization.
Path traversal time is an explicit ASSUMED constant nominal speed.
Failure/no target does not mean that a path is safe.
"""
import math


def _xy(value):
    if not isinstance(value,(list,tuple)) or len(value)<2:
        return None
    if not all(isinstance(a,(int,float)) and math.isfinite(a) for a in value[:2]):
        return None
    return (float(value[0]),float(value[1]))


def _path_xy_at_distance(path, distance):
    """Linearly interpolate a sequence of map XY waypoints by arc-length."""
    if not path:
        return None
    if len(path)==1:
        return path[0]
    remaining=distance
    for a,b in zip(path,path[1:]):
        segment=math.hypot(b[0]-a[0],b[1]-a[1])
        if segment<=1e-9:
            continue
        if remaining<=segment:
            alpha=remaining/segment
            return (a[0]+alpha*(b[0]-a[0]),a[1]+alpha*(b[1]-a[1]))
        remaining-=segment
    return path[-1]


def evaluate_path_preview(path_points, tracks, *,
                          assumed_speed_mps=.45,
                          assumed_clearance_m=.60,
                          horizon_s=1.5,
                          time_step_s=.25):
    """Estimate a h_proxy=d_xy^2-R^2 on an assumed path timing.

    Returns diagnostics ONLY, never approval to move, and never a command.
    """
    if not (.05<=assumed_speed_mps<=2.0 and
            .2<=assumed_clearance_m<=3. and
            .2<=horizon_s<=5 and .05<=time_step_s<=1):
        raise ValueError("risk preview assumptions out of range")
    report={
        "schema":"n01.offline.path_risk.v1",
        "mode":"DIAGNOSTIC_PROXY_NOT_A_CBF_CONTROLLER",
        "assumed_speed_mps":assumed_speed_mps,
        "assumed_clearance_m":assumed_clearance_m,
        "horizon_s":horizon_s,
        "robot_trajectory_timing":"ASSUMED_CONSTANT_SPEED_NOT_MEASURED",
        "safety_authorization":"NEVER",
        "physical_collision_clearance_verified":False,
        "cbf_solver_implemented":False,
        "mpc_solver_implemented":False,
        "actuator_output":"DISABLED",
        "status":"NOT_EVALUABLE",
        "evaluated_pairs":0,
        "candidate_tracks":0,
        "minimum_barrier_proxy_m2":None,
        "minimum_center_clearance_m":None,
        "maximum_overlap_radius_m":None,
        "nearest_track_id":None,
        "at_time_s":None,
        "nearest_robot_path_xy_m":None,
        "nearest_predicted_obstacle_xy_m":None,
        "robot_path_distance_assumed_m":None,
        "time_basis":"RELATIVE_TO_EXACT_MATCHED_RECORDED_SCAN",
        "dynamics_model":"CONSTANT_VELOCITY_HYPOTHESIS",

    }
    path=[]
    if isinstance(path_points,(list,tuple)):
        for point in path_points[:3000]:
            pos=_xy(point)
            if pos is None:
                report["status"]="MALFORMED_PATH"
                return report
            path.append(pos)
    if len(path)<2:
        report["status"]="NO_USABLE_REFERENCE_PATH"
        return report
    if not isinstance(tracks,list) or len(tracks)>200:
        report["status"]="MALFORMED_TRACKS"
        return report
    valid=[]
    for item in tracks:
        if not isinstance(item,dict) or item.get("classification")!="MOVING_HYPOTHESIS_UNVERIFIED":
            continue
        if (item.get("motion_consistency_passed") is not True or
                not isinstance(item.get("hits"),int) or item["hits"]<4):
            continue
        center=_xy(item.get("position_map_m"))
        velocity=_xy(item.get("velocity_xy_mps"))
        extent=_xy(item.get("extent_xy_m"))
        if center is None or velocity is None or extent is None or min(extent)<0:
            report["status"]="INVALID_MOVING_TRACK"
            return report
        radius=assumed_clearance_m + .5*math.hypot(*extent)
        valid.append((item.get("id"),center,velocity,radius))
    report["candidate_tracks"]=len(valid)
    if not valid:
        report["status"]="NO_CONFIRMED_MOVING_HYPOTHESES"
        return report
    minimum=float("inf")
    for step in range(round(horizon_s/time_step_s)+1):
        dt=min(horizon_s,step*time_step_s)
        ego=_path_xy_at_distance(path,dt*assumed_speed_mps)
        for track_id,center,velocity,radius in valid:
            obstacle=(center[0]+velocity[0]*dt,center[1]+velocity[1]*dt)
            d=math.hypot(ego[0]-obstacle[0],ego[1]-obstacle[1])
            barrier=d*d-radius*radius
            report["evaluated_pairs"]+=1
            if barrier<minimum:
                minimum=barrier
                report["minimum_center_clearance_m"]=round(d,4)
                report["maximum_overlap_radius_m"]=round(radius,4)
                report["nearest_track_id"]=track_id
                report["at_time_s"]=round(dt,3)
                report["nearest_robot_path_xy_m"]=[round(ego[0],4),round(ego[1],4)]
                report["nearest_predicted_obstacle_xy_m"]=[
                    round(obstacle[0],4),round(obstacle[1],4)]
                report["robot_path_distance_assumed_m"]=round(
                    dt*assumed_speed_mps,4)
    report["minimum_barrier_proxy_m2"]=round(minimum,5)
    report["status"]=("POTENTIAL_PROXIMITY_CONFLICT" if minimum<=0 else
                      "NO_PROXY_OVERLAP_UNDER_UNVERIFIED_ASSUMPTIONS")
    return report
