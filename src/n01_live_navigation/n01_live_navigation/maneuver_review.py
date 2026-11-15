"""ROS integration for the guide-driven, fresh-scene local maneuver planner."""
import math,time,json
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from .heading_search import FootprintCollision
from .maneuver_planner import plan_maneuver
from .navigation_path import execution_prefix
from .pose_policy import control_pose_fresh
from .core import four_wheel_steer_screen
from .hive2_kinematics import inverse
from .rotation_task import live_points
from .motion_capability import ForwardCapability
from .maneuver_continuity import retained_maneuver
from .maneuver_search_job import poll_maneuver


def publish_guide_maneuver(node):
    """Return true when a current global goal owns this planning cycle."""
    reference=getattr(node,'global_reference',None)
    if reference is None or not reference.poses:return False
    goal=getattr(node,'accepted_goal_xy',None);end=reference.poses[-1].pose.position
    if goal is None or math.hypot(end.x-goal[0],end.y-goal[1])>.12:return False
    now=time.monotonic();now_ns=node.get_clock().now().nanoseconds
    try:
        # The bound global route is a reference, not a time-limited drive permit.
        # Current body, scene, wheel feedback and every output segment are rechecked.
        if node.body_pose is None or not control_pose_fresh((now_ns-node.body_pose_stamp_ns)/1e9,now-node.body_pose_rx):raise ValueError('body_pose_stale')
        if (node.obstacles is None or now-node.obstacles_rx>.6 or
            not -.1<=(now_ns-node.obstacles_stamp_ns)/1e9<=.6 or
            node.ground_plane is None or now-node.plane_rx>2. or node.map_window is None or
            node.measured_twist is None or now-node.odom_rx>.3):raise ValueError('local_scene_or_odometry_stale')
        if node.joint_rx is None or now-node.joint_rx>.5 or (now_ns-node.joint_stamp_ns)/1e9>.5:raise ValueError('leg_and_steering_feedback_stale')
        if node._pitch_reference_match() is not True:raise ValueError('leg_feedback_not_at_supported_posture')
        pose=node.body_pose.pose.pose;p=pose.position;yaw=node._yaw_from_quaternion(pose.orientation)
        if math.hypot(end.x-p.x,end.y-p.y)<.08:raise ValueError('goal_position_already_reached')
        guide=[(v.pose.position.x,v.pose.position.y,v.pose.position.z)for v in reference.poses]
        target=execution_prefix(guide,(p.x,p.y),max(.6,node.execution_preview))[-1]
        poly=node.footprint_client.get();collision=FootprintCollision(node.obstacles,node.ground_plane,p.z,node.map_window,footprint=poly)
        turn=None
        try:turn=FootprintCollision(live_points(node),node.ground_plane,p.z,node.map_window,footprint=poly)
        except ValueError:pass
        v,w=node.measured_twist;k=w/v if abs(v)>=.01 else 0.
        if v<-.01:raise ValueError('waiting_for_forward_compatible_measured_motion')
        steering=[node.joint_positions[f'{leg}_ankle_yaw']for leg in ('fl','fr','rl','rr')]
        geometry=node.native_model_sync.geometry()
        capability=ForwardCapability(geometry,steering,max(node.max_speed,geometry.spin_threshold+.002))
        start=(p.x,p.y,yaw,k)
        def check_forward(candidate):
            preview=execution_prefix([(s[0],s[1],p.z)for s in candidate],(p.x,p.y),node.execution_preview)
            return four_wheel_steer_screen(preview,max_speed=node.max_speed,max_accel=node.max_accel,
                initial_steering=steering,geometry_override=geometry,allow_retime=True,
                body_yaw=None)['necessary_conditions_pass'] is True
        try:
            maneuver=retained_maneuver(getattr(node,'maneuver_cache',None),node.accepted_goal_ns,
                start,goal,collision,turn,capability,check_forward)
        except ValueError:
            maneuver=None
        retained=maneuver is not None
        if retained and not maneuver.rotation:
            tail=maneuver.forward.states
            remaining=sum(math.dist(a[:2],b[:2])for a,b in zip(tail,tail[1:]))
            if remaining<.5 and math.dist(tail[-1][:2],goal)>.08:
                extension=poll_maneuver(node,node.accepted_goal_ns,start,target[:2],collision,turn,capability)
                if extension is not None and not extension.rotation and extension.forward.states:
                    length=sum(math.dist(a[:2],b[:2])for a,b in zip(extension.forward.states,extension.forward.states[1:]))
                    if length>remaining+.2 and check_forward(extension.forward.states):
                        maneuver=extension;retained=False
        if maneuver is None:
            maneuver=poll_maneuver(node,node.accepted_goal_ns,start,target[:2],collision,turn,capability)
            if maneuver is None:raise ValueError('local_search_in_progress')
        if not maneuver.forward.states or (maneuver.reason!='forward_heading_path'and not maneuver.rotation):raise ValueError(maneuver.reason)
        rotation=maneuver.rotation;states=rotation+ maneuver.forward.states[1:]if rotation else maneuver.forward.states
        xyz=[(s[0],s[1],p.z)for s in maneuver.forward.states]
        steering=[node.joint_positions[f'{leg}_ankle_yaw']for leg in ('fl','fr','rl','rr')]
        geometry=node.native_model_sync.geometry()
        if rotation:
            omega=math.copysign(.08,rotation[-1][2]-rotation[0][2]);angles,drive=inverse((0.,0.,omega),steering,geometry=geometry)
            if max(abs(a)for a in angles)>min(math.pi,geometry.controller_steering_limit)+1e-6 or max(abs(d)for d in drive)>20.:raise ValueError('native_rotation_ik_rejected')
            screen={'necessary_conditions_pass':True,'kinematic_failures':[]}
            prefix=len(rotation)
        else:
            preview=execution_prefix(xyz,(p.x,p.y),node.execution_preview)
            screen=four_wheel_steer_screen(preview,max_speed=node.max_speed,max_accel=node.max_accel,
                initial_steering=steering,geometry_override=geometry,allow_retime=True,body_yaw=None)
            if screen['necessary_conditions_pass']is not True:raise ValueError('native_forward_ik_rejected')
            prefix=len(preview)
        path=Path();path.header.frame_id='map';path.header.stamp=node.get_clock().now().to_msg()
        for x,y,a,_ in states:
            msg=PoseStamped();msg.header=path.header;msg.pose.position.x=float(x);msg.pose.position.y=float(y);msg.pose.position.z=float(p.z)
            msg.pose.orientation.z=math.sin(a/2);msg.pose.orientation.w=math.cos(a/2);path.poses.append(msg)
        stamp=path.header.stamp.sec*10**9+path.header.stamp.nanosec
        stage='rotation'if rotation else'forward'
        stages=([{'type':'rotate','target_yaw_rad':rotation[-1][2],'origin_xy':[p.x,p.y],'sweep_checked':True},
                 {'type':'replan_forward','requires_measured_heading':True}]if rotation else[{'type':'forward'}])
        review=dict(schema='n01.hive2_candidate_feasibility.v1',path_model='rotate_then_forward_se2'if rotation else'forward_se2_curvature_lattice',
            planning_footprint=node.footprint_client.description(),candidate_stamp_ns=stamp,candidate_review_ready=True,trajectory_active=True,
            status='ROTATION_STAGE_READY'if rotation else'NECESSARY_CONDITIONS_PASS',reason=maneuver.reason,
            execution_stage=stage,stages=stages,post_rotation_preview_is_conditional=bool(rotation),validated_prefix_points=prefix,
            trajectory_id=stamp,trajectory_start_time={'sec':path.header.stamp.sec,'nanosec':path.header.stamp.nanosec},trajectory_duration_s=.8,
            goal_stamp_ns=node.accepted_goal_ns,goal_xy=list(goal),max_speed_limit_mps=node.max_speed,
            forward_capability=dict(source='runtime_native_ik_fk',speed_mps=capability.speed,
                curvature_samples=list(capability.choices)),
            path_selection='retained_after_fresh_review' if retained else 'new_maneuver',
            heading_search_ms=maneuver.forward.elapsed_ms,heading_search_expanded=maneuver.forward.expanded,
            candidate_only=True,actuator_output='DISABLED',safety_authorization='NEVER',**screen)
        node.path_pub.publish(path);node.diag_pub.publish(String(data=json.dumps(review)))
        node.maneuver_cache=dict(goal_id=node.accepted_goal_ns,forward=maneuver.forward.states,rotation=rotation)
        node.last_path=path;node.last_path_active=True;node.last_error=''
    except (ValueError,KeyError,TypeError)as exc:
        node._clear_candidate_visualization();node.last_error=str(exc)
        node.diag_pub.publish(String(data=json.dumps(dict(schema='n01.hive2_candidate_feasibility.v1',path_model='maneuver_search',
            planning_footprint=node.footprint_client.description(),status='MANEUVER_NOT_READY',reason=str(exc),
            goal_stamp_ns=node.accepted_goal_ns,candidate_review_ready=False,trajectory_active=False,
            actuator_output='DISABLED',safety_authorization='NEVER'))))
    return True
