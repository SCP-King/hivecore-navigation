#!/usr/bin/env python3
"""Deterministic, non-ROS G3 positive evidence: true constrained solve executed.

Synthetic moving obstacle has KNOWN scripted motion. This is not the user's
recorded Bag and never claims a real moving person/vehicle was detected.
"""
import json
from n01_offline_integration.g3_mpc_cbf_core import solve_discrete_mpc_cbf
from n01_offline_integration.g3_pipeline_core import G3HistoricalCoordinator

path=[(0.,0.),(1.,0.),(2.,0.),(3.,0.)]
moving={
    "id":11,"classification":"MOVING_HYPOTHESIS_UNVERIFIED",
    "motion_consistency_passed":True,"hits":6,
    "position_map_m":[1.,0.,.9],"velocity_xy_mps":[-.1,0.],
    "extent_xy_m":[.3,.4],
}
stamp=1_765_000_000_000_000_000
pipe=G3HistoricalCoordinator()
pipe.set_path(path,header_stamp_ns=999)
assert not pipe.put_pose(stamp,(0.,0.,.9))
solved=pipe.put_tracks({
    "schema":"n01.offline.tracks.v1",
    "frame_id":"map","status":"PROCESSED",
    "stamp_ns":stamp,"tracks":[moving],
})
assert len(solved)==1
s=solved[0]
assert s["solver_executed"],s
assert s["status"]=="FEASIBLE_CBF_CONSTRAINED_CANDIDATE_UNVERIFIED",s
assert s["cbf_constraints_checked"]>0 and s["cbf_constraints_rejected"]>0,s
assert s["minimum_nominal_h_m2"]<0,s
assert s["minimum_candidate_h_m2"]>=-1e-6,s
assert s["candidate_xy_m"] != s["nominal_xy_m"],s
assert s["safety_authorization"]=="NEVER" and s["actuator_output"]=="DISABLED"
nominal=solve_discrete_mpc_cbf(path,(0.,0.),[])
assert nominal["status"]=="FEASIBLE_NOMINAL_ONLY_NO_DYNAMIC_CONSTRAINTS"
evidence={
    "result":"G3_SYNTHETIC_DISCRETE_CBF_SOLVER_PASS",
    "evidence_type":"SYNTHETIC_KNOWN_MOTION_NOT_REAL_BAG",
    "algorithm":s["method"],
    "calibrated_hive2_robot_dynamics":False,
    "constraints_checked":s["cbf_constraints_checked"],
    "constraints_rejected":s["cbf_constraints_rejected"],
    "nominal_min_h_m2":s["minimum_nominal_h_m2"],
    "candidate_min_h_m2":s["minimum_candidate_h_m2"],
    "original_xy_m":s["nominal_xy_m"],
    "constrained_candidate_xy_m":s["candidate_xy_m"],
    "no_obstacle_result":nominal["status"],
    "actuator_output":"DISABLED",
    "safety_authorization":"NEVER",
}
print(json.dumps(evidence,ensure_ascii=False,indent=2))
