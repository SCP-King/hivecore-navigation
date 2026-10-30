"""G3 functional prototype: bounded discrete holonomic MPC with discrete CBF constraints.

This is a finite-control-set, beam-search APPROXIMATION of MPC-CBF,
not a continuous optimal solver and not a Hive2 locomotion model.
Dynamics: p[k+1] = p[k] + dt*v[k]. The exact discrete CBF condition for
each constant-velocity obstacle is h[k+1] >= (1-alpha)*h[k] and
h[k+1]>=0, h=||p-obs||^2-R^2. Velocity and acceleration are bounded.

All inputs are research hypotheses. Never returns safety authorization or a
control command. A feasible numerical solution is NOT collision certification.
"""
import math

from .replay_path_sync import project_path


def _xy(value):
    if (not isinstance(value,(tuple,list)) or len(value)<2 or
        any(not isinstance(x,(int,float)) or isinstance(x,bool) or
            not math.isfinite(x) for x in value[:2])):
        return None
    return tuple(float(v) for v in value[:2])


def _position_on_polyline(path,distance):
    if not path:
        return None
    remain=max(0.,distance)
    for a,b in zip(path,path[1:]):
        length=math.hypot(b[0]-a[0],b[1]-a[1])
        if length<=1e-9:
            continue
        if remain<=length:
            k=remain/length
            return (a[0]+k*(b[0]-a[0]),a[1]+k*(b[1]-a[1]))
        remain-=length
    return tuple(path[-1])


def _valid_obstacles(tracks,clearance):
    if not isinstance(tracks,list) or len(tracks)>200:
        return None
    obstacles=[]
    for o in tracks:
        if not isinstance(o,dict):
            return None
        if o.get("classification")!="MOVING_HYPOTHESIS_UNVERIFIED":
            continue
        if o.get("motion_consistency_passed") is not True or (
                not isinstance(o.get("hits"),int) or o["hits"]<4):
            continue
        xy=_xy(o.get("position_map_m"))
        v=_xy(o.get("velocity_xy_mps"))
        extent=_xy(o.get("extent_xy_m"))
        if xy is None or v is None or extent is None or min(extent)<0:
            return None
        if max(extent)>2. or math.hypot(*v)>4.:
            return None
        radius=clearance+.5*math.hypot(*extent)
        obstacles.append((o.get("id"),xy,v,radius))
    return obstacles


def solve_discrete_mpc_cbf(path_xy,body_xy,tracks, *,
                           dt=.25, steps=6, nominal_speed=.45,
                           max_speed=.75, max_accel=3.2,
                           cbf_alpha=.35, clearance=.55,
                           max_path_deviation=1.5, beam_width=24,
                           control_lattice="legacy_fixed_speed"):
    """Search a finite control set for a kinematically valid CBF horizon.

    control_lattice='acceleration_increment' explores velocity changes
    bounded per dt, allowing the first step to ramp up under small acceleration
    limits. The default legacy lattice is kept for historical G3 comparisons.
    Both are HOLONOMIC POINT-MASS proxies, not verified Hive2 wheel dynamics.

    Returns a candidate XY trajectory, not a Twist. The FIRST nominal
    reference point is based on the exact recorded body stamp, not a live
    localization estimate. Every attempt is labeled offline and unverified.
    """
    base={
        "schema":"n01.offline.g3_mpc_cbf.v1",
        "method":"FINITE_CONTROL_SET_BEAM_SEARCH_DISCRETE_CBF_APPROX",
        "robot_model":"PLANAR_HOLONOMIC_POINT_MASS_UNVERIFIED_FOR_HIVE2",
        "timestamp_basis":"EXACT_RECORDED_SCAN_POSE_AND_TRACKS",
        "trajectory_is_control_command":False,
        "safety_authorization":"NEVER",
        "real_robot_ready":False,
        "collision_safety_verified":False,
        "actuator_output":"DISABLED",
        "cbf_inequality":"h_next >= (1-alpha)*h_current AND h_next >= 0",
        "solver_executed":False,
        "cbf_constraints_checked":0,
        "cbf_constraints_rejected":0,
        # CBF hypothesis presence is not evidence that a constraint intervened.
        "cbf_candidate_pruning_observed":False,
        "nominal_cbf_proxy_overlap_observed":False,
        "moving_constraint_tracks":0,
        "status":"INVALID_INPUT",
        "candidate_xy_m":[],
        "nominal_xy_m":[],
        "minimum_candidate_h_m2":None,
        "minimum_nominal_h_m2":None,
        "objective_value":None,
        "search_truncated":False,
        "control_lattice":control_lattice,
        "model_initial_velocity_xy_mps":[0.0,0.0],
        "initial_velocity_source":"ASSUMED_ZERO_NOT_MEASURED_BAG_TWIST",
        "planar_velocity_limited_not_hive2_wheel_kinematics":True,
    }
    if (not isinstance(path_xy,(tuple,list)) or
        not 2<=len(path_xy)<=3000 or _xy(body_xy) is None or
        not (0.1<=dt<=.5 and 2<=steps<=12 and
             .1<=nominal_speed<=1.5 and nominal_speed<=max_speed<=2 and
             .1<=max_accel<=10 and .05<=cbf_alpha<=1 and
             control_lattice in ("legacy_fixed_speed","acceleration_increment") and
             .2<=clearance<=2 and .1<=max_path_deviation<=5
             and 4<=beam_width<=100)):
        return base
    path=[_xy(p) for p in path_xy]
    if any(p is None for p in path):
        return base
    body=_xy(body_xy)
    projection=project_path(path,body)
    if projection is None:
        base["status"]="INVALID_PATH_GEOMETRY"
        return base
    dist,progress,remaining=projection
    base["body_to_reference_distance_m"]=round(dist,4)
    base["projected_reference_progress_m"]=round(progress,4)
    if dist>max_path_deviation:
        base["status"]="BODY_OUTSIDE_REFERENCE_CORRIDOR"
        return base
    if len(remaining)<2 or sum(math.dist(a,b) for a,b in zip(
            remaining,remaining[1:]))<=1e-8:
        base["status"]="PATH_HAS_NO_REMAINING_PROGRESS"
        return base
    obstacles=_valid_obstacles(tracks,clearance)
    if obstacles is None:
        base["status"]="INVALID_TRACKS"
        return base
    base["moving_constraint_tracks"]=len(obstacles)
    base["solver_executed"]=True
    base["nominal_xy_m"]=[[round(v,4) for v in _position_on_polyline(
        remaining,nominal_speed*dt*k)] for k in range(steps+1)]
    # Nominal h is diagnostic only; it is NOT a feasible MPC-CBF solution.
    if obstacles:
        base["minimum_nominal_h_m2"]=round(min(
            math.dist(base["nominal_xy_m"][k],
                      (obs[1][0]+obs[2][0]*k*dt,
                       obs[1][1]+obs[2][1]*k*dt))**2-obs[3]**2
            for k in range(steps+1) for obs in obstacles),5)
    else:
        base["minimum_nominal_h_m2"]=None
    base["nominal_cbf_proxy_overlap_observed"]=(
        base["minimum_nominal_h_m2"] is not None and
        base["minimum_nominal_h_m2"] < 0)
    for _,xy,_,radius in obstacles:
        if math.dist(body,xy)**2-radius**2<0:
            base["status"]="INFEASIBLE_INITIAL_CBF_PROXY_OVERLAP"
            return base
    # (objective, x, y, prev_vx, prev_vy, trajectory, min_h).
    beam=[(0.,body[0],body[1],0.,0.,[body],float("inf"))]
    checks=rejected=0
    speeds=(0., .5*nominal_speed, nominal_speed, max_speed)
    headings=tuple(2*math.pi*j/16 for j in range(16))
    controls=[(0.,0.)]+[(v*math.cos(a),v*math.sin(a))
                        for v in speeds[1:] for a in headings]
    # No import of optimization or control/robot SDK: bounded deterministic
    # search. Beam pruning sacrifices global optimality.
    for k in range(steps):
        next_beam=[]
        reference=_position_on_polyline(
            remaining,nominal_speed*dt*(k+1))
        for cost,x,y,old_vx,old_vy,history,min_h in beam:
            if control_lattice == "acceleration_increment":
                # Modelled velocity lattice is STATE-DEPENDENT. The old
                # global lattice lacks any nonzero controls below 0.225m/s,
                # so it cannot accelerate smoothly at SCAN's 0.5m/s^2 and
                # 0.25s time step from its assumed rest state.
                delta = max_accel * dt
                choices=[(old_vx,old_vy),(0.,0.)]
                choices.extend((old_vx+factor*delta*math.cos(a),
                                old_vy+factor*delta*math.sin(a))
                               for factor in (0.5,1.0) for a in headings)
            else:
                choices=controls
            for vx,vy in choices:
                if (math.hypot(vx,vy)>max_speed+1e-9 or
                        math.hypot(vx-old_vx,vy-old_vy)>max_accel*dt+1e-9):
                    continue
                nx=x+vx*dt
                ny=y+vy*dt
                valid=True
                local_min=min_h
                for _,obs,v,radius in obstacles:
                    px=obs[0]+v[0]*k*dt
                    py=obs[1]+v[1]*k*dt
                    npx=px+v[0]*dt
                    npy=py+v[1]*dt
                    h=(x-px)**2+(y-py)**2-radius**2
                    hnext=(nx-npx)**2+(ny-npy)**2-radius**2
                    checks+=1
                    if hnext < -1e-8 or hnext < (1-cbf_alpha)*h-1e-8:
                        rejected+=1
                        valid=False
                        break
                    local_min=min(local_min,hnext)
                if not valid:
                    continue
                dx=nx-reference[0]
                dy=ny-reference[1]
                # Tracking + moderate velocity change + progress reward.
                stage=6.0*(dx*dx+dy*dy)+.25*((vx-old_vx)**2+
                                               (vy-old_vy)**2)
                next_beam.append((
                    cost+stage,nx,ny,vx,vy,history+[(nx,ny)],local_min))
        if not next_beam:
            base["status"]="INFEASIBLE_DISCRETE_CBF_OR_DYNAMICS"
            base["cbf_constraints_checked"]=checks
            base["cbf_constraints_rejected"]=rejected
            base["cbf_candidate_pruning_observed"]=rejected>0
            return base
        next_beam.sort(key=lambda state:(state[0],
                                        round(state[1],4),round(state[2],4)))
        # Beam pruning can discard feasible globally optimal branches.
        beam=next_beam[:beam_width]
    best=min(beam,key=lambda state:state[0])
    base["status"]=("FEASIBLE_CBF_CONSTRAINED_CANDIDATE_UNVERIFIED"
                    if obstacles else
                    "FEASIBLE_NOMINAL_ONLY_NO_DYNAMIC_CONSTRAINTS")
    base["candidate_xy_m"]=[[round(v,4) for v in p] for p in best[5]]
    base["objective_value"]=round(best[0],5)
    base["minimum_candidate_h_m2"]=(
        round(best[6],5) if obstacles else None)
    base["cbf_constraints_checked"]=checks
    base["cbf_constraints_rejected"]=rejected
    base["cbf_candidate_pruning_observed"]=rejected>0
    base["search_truncated"]=True
    base["discretization_dt_s"]=dt
    base["horizon_s"]=dt*steps
    base["assumed_clearance_m"]=clearance
    base["model_speed_limit_mps"]=max_speed
    base["model_acceleration_limit_mps2"]=max_accel
    return base
