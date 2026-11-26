"""Read-only task flow. No ROS calls, movement permissions, or pointcloud copies."""
import re
import html
import json

REASONS={
 'ready':'融合地图已就绪',
 'occupancy_stale':'占用图的原始扫描过期',
 'occupancy_missing':'尚未取得占用图',
 'ground_plane_unavailable':'尚未取得有效地面平面',
 'ground_plane_stale':'地面来源暂未更新',
 'local_window_missing':'尚未取得局部地图范围',
 'waiting_for_static_cloud':'尚未收到全局背景地图',
 'scene_not_ready':'尚未取得有效场景（扫描或地面未就绪）',
 'active_window_outside_static_map_bounds':'当前窗口尚未与累积地图范围重合',
 'map_loaded_window_has_no_occupied_voxels':'地图已收到，当前窗口没有确认占用',
 'no_reachable_observed_frontier':'没有找到机身可通过的观测前沿',
 'local_target_outside_reachable_window':'局部目标超出当前可达地图窗口',
 'no_collision_free_local_target':'路线附近没有无碰撞的局部目标',
 'scan_optimizer_no_collision_free_solution':'SCAN优化未找到无碰撞轨迹',
 'scan_refinement_or_dynamic_limit_failed':'SCAN轨迹细化或速度／加速度检查失败',
 'local_initialization_has_no_path':'局部轨迹初始化没有有效路径',
 'local_target_too_close_without_arrival':'局部目标过近，但尚未确认到达最终目标',
 'global_no_route':'融合地图中没有找到全局通路',
 'start_occupied':'起点机身包络与障碍冲突', 'goal_occupied':'终点在膨胀障碍内（可能离障碍 < 足迹+6 cm）',
 'goal_footprint_occupied':'终点足迹与障碍冲突（目标中心可能为空，但车体角/边压障碍）',
 'global_reference_collision':'最终全局参考未能保持避障',
 'map_reference_unavailable':'全局背景地图尚未取得',
 'urdf_footprint_missing_or_stale':'机器人包络缺失或过期',
 'global_search_budget_exceeded':'目标范围超出全局搜索资源预算',
 'scan_computing':'SCAN正在计算', 'scan_trajectory_available':'SCAN局部轨迹已生成',
 'reference_available':'全局参考已生成',
 'waiting_for_native_scan_trajectory':'执行器尚未取得本次目标的SCAN轨迹',
 'no_bspline_received':'审核器尚未收到SCAN样条',
 'scan_footprint_enters_unknown_space; frontier_required':'轨迹机身覆盖不足，需要调整局部目标',
 'scan_prefix_collision_or_outside_window; replan_required':'局部轨迹碰撞或越出地图窗口',
 'scan_review_waiting_for_fresh_map_or_ground':'局部检查缺少有效地图或地面',
 'waiting_for_observed_local_space':'观测覆盖缺失、过期或尚未与占用配齐',
 'control_pose_stale':'控制位姿过期', 'chassis_twist_stale':'底盘实测速度反馈过期',
 'wheel_feedback_stale':'轮组反馈过期','scene_or_admission_stale':'执行场景或轨迹依据过期',
 'observed_space_stale':'执行观测覆盖过期',
 'packet_scene_inputs_not_ready':'执行器收到轨迹，但其场景输入未就绪',
 'packet_or_scene_stale':'执行器收到的轨迹或场景已经过期',
 'scan_local_yaw_rate_exceeded':'局部轨迹角速度超过底盘 yaw 限制',
 'scan_local_curvature_exceeded':'局部轨迹曲率半径小于最小转弯半径',
 'scan_local_velocity_limit':'局部轨迹速度超过限制',
 'scan_local_acceleration_limit':'局部轨迹加速度超过限制',
 'a_star_init_zero_paths':'局部 A* 初始化返回 0 条路径',
 'a_star_init_short_path':'局部 A* 初始化只有 1 个路径点',
 'global_path_curvature_exceeds_min_turn_radius':'全局路径曲率超过最小转弯半径',
 'reverse_recovery_start':'进入受限倒车恢复',
 'reverse_recovery':'正在受限倒车',
 'reverse_complete_replan':'倒车完成，重新规划',
 'reverse_timeout_replan':'倒车超时，重新规划',
 'reverse_not_safe':'后方不安全，停止倒车',
 'reverse_rear_footprint_not_clear':'后方机身包络不安全',
 'reverse_native_model_rejected':'原生模型拒绝倒车',
 'safety_hold_released':'安全保持已解除',
 'feedback_timeout':'反馈等待超时',
 'replan_timeout':'重规划等待超时',
 'steering_timeout':'转向等待超时',
 'alignment_timeout':'对准等待超时',
 'settling_after_rotation':'旋转后停稳确认',
 'braking_after_rotation':'旋转后仍在减速',
 'leg_home_gate_released_settle':'腿未回 Home，退出停稳',
 'heading_changed_during_settle':'停稳期间朝向变化，重新对准',
 'final_approach_distance_increased':'最终进近距离变大，重新规划',
 'final_approach_no_safe_direct_command':'最终进近无安全直行指令',
 'native wheel model rejected rotation at all feasible rates':'原生模型拒绝所有可行旋转',
 'waiting for full-azimuth scan':'等待全向扫描覆盖',
 'rotation swept volume occupied; SCAN replan requested':'旋转扫掠体积被占用，请求重规划',



 'packet_goal_frame_or_revision_mismatch':'轨迹目标、坐标系或版本不匹配',
 'new_observation_or_alternative_frontier':'新观测到达后自动重选可达前沿', 'local_detour_retry':'局部绕行重试：只调整当前段，不改全局参考',
 'local_detour_exhausted':'局部绕行已用尽，改为全局重规划',
 'global_replan_retry':'全局重规划重试：从当前位姿重新搜索',
 'goal_unreachable_observed':'局部与全局均失败：零速等待，目标保留，可手动继续',
 'RECOVERY_RESUME':'操作员已请求继续，重新检查目标与场景',
 'RECOVERY_TIMEOUT':'等待超时后自动重试原目标',

 'retry_on_fused_scene':'根据融合场景自动重试；有效旧段继续复核',
 'update_goal_or_scene':'检查目标占用及地图更新后重新规划',
 # 全局规划
 'invalid_global_inputs':'全局规划输入无效（数值或分辨率异常）',
 'start_or_goal_outside_map_bounds':'起点或终点超出地图与搜索范围',
 'goal_occupied':'终点在膨胀障碍内（可能离障碍 < 足迹+6 cm）',
 'global_refresh_failed':'全局参考刷新失败',
 'goal_collision_or_unavailable':'目标不可用或与障碍冲突',
 # 局部规划
 'scan_trajectory_available':'局部轨迹已生成',
 'scan_no_solution':'局部规划无解',
 'scan_footprint_enters_unknown_space':'足迹进入未观测空间（旧覆盖逻辑）',
 'footprint_coverage_missing':'足迹覆盖不足（旧覆盖逻辑）',
 # 执行包件与反馈
 'packet_map_id_missing':'轨迹缺少地图版本标识',
 'packet_expected_goal_mismatch':'轨迹目标与当前目标不一致',
 'packet_geometry_or_scene_size_invalid':'几何或场景数据非法',
 'chassis_twist_stale':'底盘速度反馈过期',
 'scene_or_admission_stale':'场景或准入信息过期',
 'native_ik_fk_twist_mismatch':'原生四轮模型正逆解不一致',
 'live_wheel_angle_speed_or_steering_rate_limit':'轮角、轮速或舵速超限',
 'no_safe_command_above_native_translation_threshold':'没有高于底盘启动阈值的可行速度',
 'corrected_command_collision_unknown_or_outside_scene':'纠偏指令会碰撞或超出场景',
 'accepted_goal_changed':'目标已变更，原轨迹作废',
 'map_or_coordinate_model_changed':'地图或坐标模型已切换',
 # 轨迹检查
 'scan_review_waiting_for_urdf_footprint':'等待 URDF 包络模型',
 'scan_prefix_collision_or_outside_window':'轨迹碰撞或越出地图窗口',
 'trajectory_from_previous_goal':'收到上一目标的轨迹',
 'trajectory_window_expired':'轨迹时间窗口已过期',
 'missing_body_pose_for_execution_preview':'缺少机身位姿，无法预判执行',
 'no_verified_initial_motion_corridor':'没有可验证的初始运动走廊',
 'candidate_too_far_from_robot':'候选轨迹离机器人太远',
 'conflicting_trajectory_revision':'轨迹版本冲突',
 'execution_preview_too_short':'执行预判距离过短',
 'invalid_execution_preview':'执行预判参数无效',
 'collision_or_window_boundary':'碰撞或到达地图窗口边界',
 # 场景与准入
 'waiting_system_health':'等待系统健康状态',
 'invalid_system_health_message':'系统健康消息非法',
 'system_health_missing_or_stale':'系统健康状态缺失或过期',
 'goal_event_recorded':'目标已记录',
}

# Controller emits a few English sentences instead of codes; keep the display in
# Chinese and leave the original text in the expandable facts/log only.
_SENTENCES = {
 'external command traffic; explicit resume required':'手动接管中，需要显式继续',
 'chassis stop or emergency stop':'底盘急停或已停车',
 'braking before alignment':'转向之前正在制动',
 'station position changed; scan trajectory required':'站点位置变化，需要新的局部轨迹',
 'native wheel model rejected rotation':'原生四轮模型拒绝该转向',
 'rotation swept volume occupied; scan replan requested':'旋转扫掠空间被占，已请求重规划',
 'waiting for full-azimuth scan':'等待全向扫描',
}

_ENVELOPE_CAUSES = {
 'live_occupied':'实测障碍',
 'static_occupied':'静态地图障碍',
 'outside_map':'超出地图范围',
 'footprint_missing':'包络模型缺失或过期',
 'not_observed':'未观测区域（旧覆盖逻辑）',
}

FAULT_DOMAINS = {
    "map": "地图", "localization": "定位", "perception": "感知",
    "planning": "规划", "execution": "执行", "chassis": "底盘",
    "config": "配置", "operator": "人工", "unknown": "数据不足",
}
FAULT_ACTIONS = {
    "map": "检查目标/起点占用与地图版本，更新 PCD 或重选目标",
    "localization": "检查定位 tracking/TF/时间戳，重定位后再试",
    "perception": "检查 /cloud、地面与场景时效",
    "planning": "检查窄通道/搜索预算并重规划",
    "execution": "检查轨迹包/场景/反馈，等重规划或人工挪车",
    "chassis": "释放急停/手动接管，检查轮组与底盘",
    "config": "检查 URDF/包络/参数配置",
    "operator": "确认停止/取消/改目标",
    "unknown": "查日志与话题，不自动重试",
}


def fault_domains(reason, evidence=None):
    """Return primary/candidate fault domains with evidence.

    Attribution is deliberately conservative: it only uses reason keywords and
    caller-supplied evidence. Missing data stays in ``unknown``.
    """
    text = str(reason or "").strip()
    low = text.lower()
    cand = []

    def add(domain, why):
        if domain not in [c[0] for c in cand]:
            cand.append((domain, why))

    if text and "not_ready" not in low and "missing" not in low and "stale" not in low and (
            low.endswith("_ready") or low.endswith("_available") or low == "ready" or low == "scan_computing"):
        return {"primary": "none", "label": "正常", "action": "无需动作",
                "candidates": [{"domain": "none", "label": "正常", "why": "状态为正常/就绪"}],
                "evidence": evidence or {}}
    if not text or low in ("", "none", "unknown", "nan"):
        add("unknown", "没有原因码或状态数据")
    if any(k in low for k in ("localization", "robot_pose", "pose_scan", "body_pose",
                              "odom_", "reloc", "anchor", "scan_motion_compensation",
                              "control_pose", "tf_")):
        add("localization", "位姿/里程计/TF/扫描对齐")
    if any(k in low for k in ("estop", "emergency", "急停", "manual_override", "手动接管",
                              "wheel_", "chassis_twist", "native_ik", "native_translation",
                              "steering", "cmd_")):
        add("chassis", "急停/底盘/轮组/原生模型")
    if any(k in low for k in ("ground_", "scene_not_ready", "occupancy_stale",
                              "live_", "point_cloud", "cloud_", "sensor_", "感知")):
        add("perception", "激光/点云/地面/场景数据")
    if any(k in low for k in ("urdf", "footprint_missing", "invalid_global_inputs",
                              "packet_geometry", "geometry_or_scene_size", "配置")):
        add("config", "URDF/包络/参数/接口配置")
    if any(k in low for k in ("goal_invalid", "user_stop", "manual_stop", "人工停止", "目标非法")):
        add("operator", "人工停止/取消或目标非法")
    if any(k in low for k in ("packet_", "scene_or_admission", "corrected_command",
                              "feedback_", "waiting_for_replan", "trajectory_",
                              "execution_", "arrival", "no_safe_command",
                              "no_verified_initial", "执行", "纠偏", "反馈")):
        add("execution", "轨迹包/场景准入/跟踪/纠偏")
    if any(k in low for k in ("global_", "scan_", "local_", "optimizer", "frontier",
                              "reference_", "planning", "budget", "route", "规划")):
        add("planning", "全局/局部搜索或轨迹生成")
    if any(k in low for k in ("goal_", "start_", "map_", "static_", "occupied",
                              "no_route", "reference_unavailable", "地图", "障碍", "包络")):
        add("map", "地图/占用格/目标格/起点包络")
    if not cand:
        add("unknown", "原因码未匹配到任何域")
    primary = cand[0][0]
    if primary == "planning" and "global_no_route" in low:
        cand[0] = ("map", "全局图无可通行路线")
        primary = "map"
    if primary == "map" and "start_occupied" in low:
        add("localization", "若起点实际空，检查位姿/足迹是否错位")
    return {
        "primary": primary,
        "label": FAULT_DOMAINS.get(primary, primary),
        "action": FAULT_ACTIONS.get(primary, FAULT_ACTIONS["unknown"]),
        "candidates": [{"domain": d, "label": FAULT_DOMAINS.get(d, d), "why": w} for d, w in cand],
        "evidence": evidence or {},
    }


def describe_domain(reason, evidence=None):
    d = fault_domains(reason, evidence)
    return f"[{d['label']}] {describe(reason)}；建议：{d['action']}"


def describe(reason):
    text=str(reason or '').strip()
    if not text:return '根因未取得'
    # Already-Chinese text is returned untouched (task layer writes Chinese).
    if any('\u4e00' <= ch <= '\u9fff' for ch in text):return text
    if text.startswith('start_envelope_'):
        body=text[len('start_envelope_'):]
        cause,_,where=body.partition('@')
        label=_ENVELOPE_CAUSES.get(cause,'已知障碍')
        spot=where.strip('()').replace(',',', ') if where else ''
        if cause=='footprint_missing':
            return '机器人包络模型缺失或过期，暂无法检查起点'+(f'（{spot}）' if spot else '')
        return f'起点包络被{label}挡住（{spot}）' if spot else f'起点包络被{label}挡住'
    for suffix in ('; frontier_required','; replan_required'):
        if text.endswith(suffix):text=text[:-len(suffix)]
    if ';' in text:
        base,_,detail=text.partition(';')
        head=describe(base.strip())
        tail=describe(detail.strip())
        return f"{head}（{tail}）" if tail and not tail.startswith('未登记') else head
    if text in REASONS:return REASONS[text]
    lowered=text.lower()
    for key,value in _SENTENCES.items():
        if lowered==key:return value
    # Source names (external publishers) and arbitrary text are kept verbatim;
    # the renderer escapes them. Only unregistered snake_case codes are hidden.
    if text.startswith('/') or text.endswith(('_node','_controller','_publisher',
                                              '_driver','_bridge','_adapter','_server')):
        return text
    if re.fullmatch(r'[a-z0-9_.]+',text):
        return '未登记原因（详见日志）'
    return text


def build_flow(goal_id,planning=None,review=None,execution=None,scene=None,readiness=None,
               task=None,running=False,ages=None):
    planning=dict(planning or {});review=dict(review or {});execution=dict(execution or {})
    scene=dict(scene or {});readiness=dict(readiness or {});task=dict(task or {});ages=ages or {}
    # Old goals are never evidence for this goal. Missing IDs are explicitly uncorrelated.
    planning=planning if goal_id and planning.get('goal_id')==goal_id else {}
    review=review if goal_id and review.get('goal_id')==goal_id else {}
    execution=execution if goal_id and execution.get('goal_id')==goal_id else {}
    stages=[]
    def add(key,label,state,detail='',facts=None):
        stages.append(dict(key=key,label=label,state=state,detail=detail,facts=facts or {}))
    gp=planning.get('global_phase',planning.get('phase',''))
    lp=planning.get('local_phase','')
    if not lp and planning.get('phase','').startswith(('LOCAL','WAITING')):lp=planning['phase']
    gr=planning.get('global_reason',planning.get('reason',''))
    lr=planning.get('local_reason',planning.get('reason',''))
    stale_plan=ages.get('planning') is not None and ages['planning']>3.
    add('task','任务目标','active' if goal_id else 'idle',f'目标ID {goal_id}'if goal_id else'未提交目标')
    scene_issues=[i for i in review.get('issues',[])if i.get('reason')in {'waiting_for_observed_local_space','occupancy_missing','occupancy_stale','ground_plane_unavailable','ground_plane_stale','local_window_missing'}]
    if running and scene.get('scan_ready') is False:
        scene_issues.insert(0,dict(reason=scene.get('scan_reason','occupancy_missing'),
                                  stamp_age_s=scene.get('scan_age_s'),topic='/grid_map/occupancy',
                                  upstream='/cloud → GridMap；检查SLAM与点云桥接是否运行'))
    scene_ready=not scene_issues and bool(scene.get('ready',False)) and running and (ages.get('scene') is None or ages['scene']<=3.)
    scene_detail=describe(scene_issues[0]['reason']if scene_issues else scene.get('reason','等待融合地图诊断'))
    if running and scene.get('scan_ready') is False:
        scan_age=scene.get('scan_age_s')
        scene_detail += (f'（{scan_age:.1f}s）' if scan_age is not None else '')+'；检查SLAM与 /cloud 点云桥接，地图心跳不代表扫描更新'
    add('scene','环境融合','done'if scene_ready else ('waiting'if running else'idle'),
        scene_detail,
        {'issues':scene_issues,**{k:scene[k]for k in ('scan_age_s','scan_ready','unconfirmed_risk_points','observed_columns','environment_margin_m','window_min_xyz','window_max_xyz')if k in scene}})
    add('global','全局规划','failed'if gp=='GLOBAL_FAILED' else ('done'if gp=='GLOBAL_READY' else'waiting'if goal_id else'idle'),describe(gr)if planning else'等待本次目标的规划状态')
    target_wait=lp=='WAITING_FOR_OBSERVATION'
    add('target','局部目标／前沿','waiting'if target_wait else('done'if lp.startswith('LOCAL')else'idle'),describe(lr)if target_wait else'沿全局路线选取可达局部目标',
        {k:planning[k]for k in ('local_target','frontier_expanded')if k in planning})
    local_state='done'if lp=='LOCAL_READY' else'failed'if lp=='LOCAL_FAILED' else'active'if lp=='LOCAL_PLANNING'else'waiting'if goal_id else'idle'
    add('scan','SCAN局部规划',local_state,describe(lr)if lp else'等待局部目标和规划状态',
        {k:planning[k]for k in ('local_attempts','scene_age_s')if k in planning})
    if stale_plan and stages[-1]['state']in('active','waiting'):
        stages[-1]['state']='waiting';stages[-1]['detail']='规划状态超过3秒未更新；不能确认仍在计算'
    admitted=review.get('candidate_review_ready')is True
    add('review','轨迹检查','done'if admitted else'waiting'if goal_id else'idle',
        ('有效前段可执行，已请求局部延伸'if review.get('prefix_limited')else'风险建议降速，当前段可执行'if review.get('risk_speed_advice_active')else'本次轨迹检查通过')if admitted else describe(review.get('reason','等待本次目标的检查结果')),
        {k:review[k]for k in ('issues','failure_context','candidate_review_status','trajectory_id')if k in review})
    es=execution.get('state','');moving=es in ('MOVING','ROTATING','ALIGN_PATH','ALIGN')
    add('execution','跟踪执行','active'if moving else'done'if es=='ARRIVED'else'waiting'if execution else'idle',
        '正在执行当前轨迹'if moving else describe(execution.get('reason','执行器未运行或没有本次目标反馈')),
        {k:execution[k]for k in ('speed_mps','goal_distance_m','trajectory_id','control_last_ms','trajectory_rejection')if k in execution})
    phase=task.get('phase','')
    add('result','到达／下一站','done'if phase in ('ARRIVED','CANCELLED')else'failed'if phase=='FAILED'else'waiting'if phase in ('PAUSED','MANUAL')else'idle',
        describe(task.get('reason')) if task.get('reason') else '到达并停稳后，执行站点停留或进入下一站')
    causes=[]
    if running and scene_issues:causes.extend(describe(i['reason']) for i in scene_issues)
    if gp=='GLOBAL_FAILED':causes.append(describe(gr))
    if target_wait or lp=='LOCAL_FAILED':causes.append(describe(lr))
    if review.get('reason')and not admitted:causes.append(describe(review['reason']))
    if execution.get('reason')and not moving:causes.append(describe(execution['reason']))
    rejection=execution.get('trajectory_rejection',{})
    if rejection and not moving:causes.append(describe(rejection.get('reason',''))+'：'+str(rejection.get('source_reason','')))
    if not causes:causes=['当前无已确认的阻塞原因'if admitted else'尚未取得完整原因链']
    return dict(stages=stages,causes=list(dict.fromkeys(causes)),
                recovery=describe(planning.get('recovery','')) if planning else'等待本次目标诊断',
                loops=f"局部尝试 {planning.get('local_attempts',0)} 次 · 全局重规划 {planning.get('global_replans',0)} 次",
                readiness=readiness)


def render_flow(flow):
    esc=lambda value:html.escape(str(value),quote=True)
    colors={'idle':'#64748b','active':'#38bdf8','waiting':'#f59e0b','failed':'#f87171','done':'#4ade80'}
    bgs={'idle':'rgba(15,23,42,.55)','active':'rgba(14,42,68,.55)','waiting':'rgba(56,36,10,.45)','failed':'rgba(62,18,24,.55)','done':'rgba(8,42,28,.45)'}
    names={'idle':'未开始','active':'进行中','waiting':'等待','failed':'失败／重试','done':'已完成'}
    cards=[]
    for idx,node in enumerate(flow['stages'],start=1):
        st=node['state']
        facts=json.dumps(node['facts'],ensure_ascii=False,indent=2)
        cards.append(f'''<details style="border:1px solid rgba(148,163,184,.16);border-left:3px solid {colors[st]};background:{bgs[st]};border-radius:7px;padding:6px 8px">
<summary style="cursor:pointer;display:flex;align-items:center;justify-content:space-between;gap:6px"><span style="font-size:11px;font-weight:600;color:#e2e8f0">{idx}. {esc(node['label'])}</span><small style="color:{colors[st]};font-size:10px;font-weight:600">{names[st]}</small></summary>
<div style="font-size:11px;margin-top:5px;color:#cbd5e1">{esc(node['detail'])}</div><pre style="font-size:10px;margin:4px 0 0;color:#94a3b8;white-space:pre-wrap;overflow-wrap:anywhere">{esc(facts if node['facts']else'')}</pre></details>''')
    return ('<div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:6px;align-items:start">'+
            ''.join(cards)+'</div>'+
            '<div style="font-size:11px;margin-top:7px;color:#fbbf24">原因链：'+esc(' ← '.join(reversed(flow['causes'])))+'</div>'+
            '<div style="font-size:11px;color:#94a3b8">恢复：'+esc(flow['recovery'])+' · '+esc(flow['loops'])+'</div>')
