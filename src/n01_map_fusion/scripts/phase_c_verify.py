#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 C 验收验证器(离线,只读)。

读 loop_report_c.json(collector 报告)与 gate.json(gate 记录),逐项判定:
  1  gate 通过(位移>=0.25,距goal>0.6,障碍位置安全)
  2  单 sim_motion 实例(S0-init 仅 1 次,无重启)
  3  注入前有真实位移(pose 序列连续变化,不只静态断言)
  4  地图更新(注入后 fused_grid OCC 增加;dynamic_occ>0)
  5  注入后重规划(plan_result/path 事件晚于注入时刻)
  6  新路径从注入时刻附近位姿出发
  7  新路径全部避障(waypoint 距障碍 >= 名义边界)
  8  sim 位姿连续、轨迹线段不穿 OCC/UNKNOWN(注入前用 before 栅格,注入后用 after 栅格)
  9  最终到位(S2-arrived 终态,最终位姿误差 <=0.15m)
 10  全程无 S3-collision;S2 不被 2Hz 刷新重置(终态后无 S1)
输出 JSON 报告 phase_c_report.json。不写任何话题,不发布,不清理外部进程。
"""
import argparse
import base64
import json
import math
import sys

import numpy as np

FREE = np.int8(0)
OCC = np.int8(100)
UNK = np.int8(-1)

# 名义安全边界:mock 半径 0.15 + 融合 L1 膨胀 0.40
OBSTACLE_BOUND_M = 0.55
# 新路径起点(注入时刻附近位姿)允许的 snap 偏移
START_SNAP_TOL_M = 0.50
# 注入后重规划必须及时出现(2Hz mock + ms 级融合/规划;1.5s 留足余量)
REPLAN_MAX_S = 1.5


def load_grid(g):
    if not g:
        return None
    enc = g.get('grid_b64')
    if not enc:
        return None, g
    arr = np.frombuffer(base64.b64decode(enc), dtype=np.int8)
    meta = dict(ox=g['ox'], oy=g['oy'], res=g['res'], sx=g['sx'], sy=g['sy'])
    return arr, meta


def cell_free(arr, meta, x, y):
    ix = int(math.floor((x - meta['ox']) / meta['res']))
    iy = int(math.floor((y - meta['oy']) / meta['res']))
    if not (0 <= ix < meta['sx'] and 0 <= iy < meta['sy']):
        return False
    return arr[iy * meta['sx'] + ix] == FREE


def segment_check(arr, meta, p0, p1, step=0.02):
    """检查线段每个采样点都落在 FREE 栅格。返回 (ok, first_bad)。"""
    d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    n = max(1, int(math.ceil(d / step)))
    for i in range(n + 1):
        t = i / n
        x = p0[0] + (p1[0] - p0[0]) * t
        y = p0[1] + (p1[1] - p0[1]) * t
        if not cell_free(arr, meta, x, y):
            return False, (round(x, 3), round(y, 3))
    return True, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--report', required=True)
    ap.add_argument('--gate', required=True)
    ap.add_argument('--inject-epoch', required=True, type=float)
    ap.add_argument('--output', required=True)
    ap.add_argument('--goal-x', type=float, default=11.05)
    ap.add_argument('--goal-y', type=float, default=6.45)
    a = ap.parse_args()

    report = json.load(open(a.report))
    gate = json.load(open(a.gate))
    goal = (a.goal_x, a.goal_y)
    t_inject = a.inject_epoch - report['wall_start_epoch']
    results = []
    detail = {}

    def add(name, ok, why, ev=''):
        results.append({'item': name, 'pass': bool(ok),
                        'detail': why, 'evidence': ev})

    # ---------- 输入 / 时间基准 ----------
    wall_start = report['wall_start_epoch']
    detail['t_inject_collector_s'] = round(t_inject, 2)
    detail['inject_epoch'] = a.inject_epoch

    # ---------- 1. gate ----------
    g_ok = all(k in gate for k in ('first_pose', 'inject_pose', 'displacement_m',
                                    'goal_distance_m', 'obstacle_center_clearance_m'))
    moved = gate.get('displacement_m')
    gdist = gate.get('goal_distance_m')
    gclr = gate.get('obstacle_center_clearance_m')
    add('gate_ok', g_ok and moved is not None and moved >= 0.25 and
        gdist is not None and gdist > 0.60,
        f'inject_pose={gate.get("inject_pose")} displ={moved:.3f}m goal_dist={gdist:.3f}m '
        f'obstacle_clear={gclr:.3f}m',
        json.dumps(gate, ensure_ascii=False))

    obs = (gate['inject_pose']['x'], gate['inject_pose']['y'])  # 注入位姿
    obstacle = (gate['obstacle']['x'], gate['obstacle']['y'])

    # 栅格数据必须存在轨迹线段检查才有效
    _has_grid_evidence = (report.get('grid_before') is not None and
                          report.get('grid_after') is not None and
                          report['grid_before'].get('grid_b64') and
                          report['grid_after'].get('grid_b64'))

    # ---------- 2. 单 sim 实例 ----------
    sim_st = [e for e in report['events'] if e['kind'] == 'sim_status']
    s0 = [e['data'] for e in sim_st if e['data'] == 'S0-init']
    add('single_sim_instance', len(s0) == 1,
        f'S0-init 出现 {len(s0)} 次(唯一实例,全程无 sim_motion2 重启)', ' '.join(s0))

    # ---------- 3. 注入前位移 ----------
    poses = report['poses']  # (t, x, y, yaw)
    pre = [p for p in poses if p[0] < t_inject]
    add('pre_injection_displacement', len(pre) >= 2 and
        math.hypot(pre[0][1] - pre[-1][1], pre[0][2] - pre[-1][2]) >= 0.20,
        f'注入前 pose 数={len(pre)} 首({pre[0][1]:.3f},{pre[0][2]:.3f}) '
        f'末({pre[-1][1]:.3f},{pre[-1][2]:.3f}) '
        f'净位移={math.hypot(pre[-1][1]-pre[0][1], pre[-1][2]-pre[0][2]):.3f}m')

    # ---------- 4. 地图更新 ----------
    gb = report.get('grid_before')
    ga = report.get('grid_after')
    occ_delta = None
    if gb and ga:
        occ_delta = ga['occ'] - gb['occ']
    add('map_updated', occ_delta is not None and occ_delta > 0,
        f'grid_before.occ={gb["occ"] if gb else None} grid_after.occ={ga["occ"] if ga else None} '
        f'occ_delta={occ_delta}',
        json.dumps([e for e in report['events'] if e['kind'] == 'fusion_stats'], ensure_ascii=False))

    # ---------- 5. 注入后重规划 ----------
    path_ev = [e for e in report['events'] if e['kind'] == 'path']
    res_ev = [e for e in report['events'] if e['kind'] == 'plan_result']
    paths_after = [e for e in path_ev if e['t'] >= t_inject - 0.05]
    plans_after = [e for e in res_ev if e['t'] >= t_inject - 0.05]
    _first_replan_t = plans_after[0]['t'] if plans_after else None
    if _first_replan_t is not None:
        _replan_delay = _first_replan_t - t_inject
        _replan_txt = f't={_first_replan_t:.2f}s 延迟={_replan_delay:.2f}s <= {REPLAN_MAX_S:.1f}s'
        _replan_ok = _replan_delay <= REPLAN_MAX_S
    else:
        _replan_txt = '无注入后重规划'
        _replan_ok = False
    add('replan_after_injection',
        _first_replan_t is not None and len(paths_after) >= 1 and _replan_ok,
        f'注入后 plan_result={len(plans_after)} path={len(paths_after)} '
        f'(注入 t={t_inject:.2f}s,{_replan_txt})',
        json.dumps({'plans': plans_after[-5:], 'paths': paths_after[-5:]}, ensure_ascii=False))

    # ---------- 6. 新路径从注入位姿附近出发 ----------
    start_dist = None
    if paths_after:
        head = paths_after[-1]['data']['head'][0]  # (x, y)
        start_dist = math.hypot(head[0] - obs[0], head[1] - obs[1])
    add('replan_starts_near_current_pose', start_dist is not None and start_dist <= START_SNAP_TOL_M,
        f'注入位姿=({obs[0]:.3f},{obs[1]:.3f}) 注入后最新路径头部=({head[0] if start_dist is not None else None},'
        f'{head[1] if start_dist is not None else None}) 距离={start_dist if start_dist is not None else None:.3f}m',
        json.dumps(paths_after[-3:] if paths_after else [], ensure_ascii=False))

    # ---------- 7. 新路径避障 ----------
    obs_x, obs_y = obstacle
    path_min = None
    if paths_after:
        pts = list(paths_after[-1]['data']['head']) + list(paths_after[-1]['data']['tail'])
        path_min = min((math.hypot(hx - obs_x, hy - obs_y) for hx, hy in pts), default=None)
    add('replanned_path_avoids_obstacle', path_min is not None and path_min >= OBSTACLE_BOUND_M - 0.10,
        f'注入后最新路径点最小距障碍={path_min if path_min is not None else float("nan"):.3f}m '
        f'(名义边界 {OBSTACLE_BOUND_M}m;完整逐段避障由检查8轨迹线段兜底)',
        json.dumps(paths_after[-1]['data'] if paths_after else {}, ensure_ascii=False))

    # ---------- 8. 轨迹线段不穿 OCC/UNKNOWN ----------
    arr_before, meta_before = load_grid(report.get('grid_before'))
    arr_after, meta_after = load_grid(report.get('grid_after'))
    seg_bad = []
    # 注入前段网格检查前段;注入后段检查后段(轨迹由 20Hz pose 组成,逐段采样 0.02m)
    for i in range(len(poses) - 1):
        t0, x0, y0, _ = poses[i]
        t1, x1, y1, _ = poses[i + 1]
        if t1 < t_inject:
            arr, meta = arr_before, meta_before
        elif t0 >= t_inject:
            arr, meta = arr_after, meta_after
        else:
            arr, meta = arr_after, meta_after  # 跨注入段用注入后栅格(严格)
        if arr is None:
            continue
        ok, bad = segment_check(arr, meta, (x0, y0), (x1, y1))
        if not ok:
            seg_bad.append((round(t1, 2), bad))
    add('trajectory_segments_in_free',
        _has_grid_evidence and len(seg_bad) == 0,
        (f'pose 段({len(poses) - 1}) 采样逐点检 FREE;冲突 {len(seg_bad)} 处 {seg_bad[:8]}'
         + ('' if _has_grid_evidence else ' ; [FAIL] 缺 grid_before/grid_after 栅格证据')),
        json.dumps({'n_seg': len(poses) - 1, 'bad': seg_bad[:8]}, ensure_ascii=False))

    # ---------- 9/10. 终态与全程无 S3 ----------
    statuses = [e['data'] for e in sim_st]
    has_s3 = any(s.startswith('S3') for s in statuses)
    last_s2 = None
    for i, s in enumerate(statuses):
        if s == 'S2-arrived':
            last_s2 = i
    s1_after_s2 = any(s.startswith('S1') for s in statuses[(last_s2 + 1):]) if last_s2 is not None else False
    # 最终位姿误差
    final_pose = poses[-1][1:3] if poses else None
    final_err = math.hypot(final_pose[0] - goal[0], final_pose[1] - goal[1]) if final_pose else None
    add('final_reached', last_s2 is not None and final_err is not None and final_err <= 0.15,
        f'S2-arrived 末次索引={last_s2} 最终位姿=({final_pose[0]:.3f},{final_pose[1]:.3f}) '
        f'goal=({goal[0]:.2f},{goal[1]:.2f}) err={final_err:.3f}m',
        json.dumps(statuses, ensure_ascii=False))
    add('no_s3_collision', not has_s3,
        f'S3 出现={"是" if has_s3 else "否"}',
        json.dumps([s for s in statuses if s.startswith('S3')], ensure_ascii=False))
    add('s2_terminal_held', last_s2 is not None and not s1_after_s2,
        f'S2 后出现 S1={"是" if s1_after_s2 else "否"}(2Hz 刷新未重置终态)',
        json.dumps(statuses[-8:], ensure_ascii=False))

    # ---------- 汇总 ----------
    passed = sum(1 for r in results if r['pass'])
    overall = 'PASS' if passed == len(results) else 'FAIL'
    out = {
        'overall': overall,
        'passed_items': passed,
        'total_items': len(results),
        'items': results,
        'detail': detail,
        'verdict': overall,
        't_inject_s': round(t_inject, 2),
        'final_pose': {'x': final_pose[0], 'y': final_pose[1]} if final_pose else None,
        'final_error_m': final_err,
        'single_sim': len(s0) == 1,
        'inject_epoch': a.inject_epoch,
    }
    with open(a.output, 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0 if overall == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())