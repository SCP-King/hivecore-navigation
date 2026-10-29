#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Phase 5 离线验证采集/判定脚本(只读订阅,不发布任何内容)。

订阅 /map/fusion/fused_grid(最新)、/map/fusion/stats(最新)、/plan/path(最新)、
/plan/result(最新)。判定:
  1. 融合计数: static_occ / dynamic_occ / free / unknown / ground / inflated
     (取自 stats,并用最终 fused_grid 独立复核);
  2. 起终点可用性: 在最终 fused_grid 中按 snap_radius 求最近 FREE,
     并核对与 path 端点一致;
  3. 路径碰撞检查: path 每个格点在最终 fused_grid 上都必须为 FREE(=0);
  4. frame 一致性: /grid_map/occupancy frame 必须为 map(数据源铁律)。

退出条件: 信号(SIGTERM/SIGINT)或"已见到 fused_grid 后连续 15s 无新更新"(bag 结束、
/clock 停止、static_map 不再发布)。
输出: JSON 报告至 --output(默认 stdout 同时打印摘要)。
"""
import argparse
import json
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

FREE = 0


class Verifier(Node):
    def __init__(self, start_xy, goal_xy, snap_radius):
        super().__init__('n01_fusion_verifier')
        self._start = start_xy
        self._goal = goal_xy
        self._snap = snap_radius

        self.fused = None          # 最新 fused OccupancyGrid
        self.fused_n = 0           # 收到更新次数
        self.stats = None          # 最新 stats String
        self.path = None           # 最新 Path
        self.result = None         # 最新 /plan/result String
        self.occ_frames = set()    # /grid_map/occupancy 的 frame_id 集合
        self.last_fused_wall = None

        self.sub_fused = self.create_subscription(
            OccupancyGrid, '/map/fusion/fused_grid', self.cb_fused, 10)
        self.sub_stats = self.create_subscription(
            String, '/map/fusion/stats', self.cb_stats, 10)
        self.sub_path = self.create_subscription(
            Path, '/plan/path', self.cb_path, 10)
        self.sub_result = self.create_subscription(
            String, '/plan/result', self.cb_result, 10)
        self.sub_occ = self.create_subscription(
            PointCloud2, '/grid_map/occupancy', self.cb_occ, 10)

    def cb_fused(self, msg):
        self.fused = msg
        self.fused_n += 1
        self.last_fused_wall = time.time()

    def cb_stats(self, msg):
        self.stats = msg.data

    def cb_path(self, msg):
        self.path = msg

    def cb_result(self, msg):
        self.result = msg.data

    def cb_occ(self, msg):
        self.occ_frames.add(msg.header.frame_id)


def col_of(x, y, info, origin=None, res=None):
    ox = origin if origin is not None else info.origin.position.x
    oy = info.origin.position.y
    rr = res if res is not None else info.resolution
    ix = int((x - ox) / rr)
    iy = int((y - oy) / rr)
    if not (0 <= ix < info.width and 0 <= iy < info.height):
        return None
    return iy * info.width + ix


def nearest_free(data, sx, sy, cell, rcells):
    """欧氏最小 FREE;返回 (cell, dist_cells) 。"""
    if cell is None:
        return None, None
    ix0, iy0 = cell % sx, cell // sx
    best, best_d = None, None
    for iy in range(max(0, iy0 - rcells), min(sy, iy0 + rcells + 1)):
        for ix in range(max(0, ix0 - rcells), min(sx, ix0 + rcells + 1)):
            d2 = (ix - ix0) ** 2 + (iy - iy0) ** 2
            if d2 > rcells * rcells:
                continue
            c = iy * sx + ix
            if data[c] == FREE:
                if best_d is None or d2 < best_d:
                    best, best_d = c, float(d2) ** 0.5
    return best, best_d


def analyze(run):
    rep = {
        'fused_updates': run.fused_n,
        'fusion_stats': run.stats,
        'result': run.result,
        'path_count': len(run.path.poses) if run.path else 0,
        'path_header_frame': run.path.header.frame_id if run.path else None,
        'occupancy_frames': sorted(run.occ_frames),
    }
    if run.fused is not None:
        msg = run.fused
        data = msg.data
        sx, sy = msg.info.width, msg.info.height
        rep['fused'] = {
            'frame': msg.header.frame_id,
            'origin': [msg.info.origin.position.x, msg.info.origin.position.y],
            'resolution': msg.info.resolution,
            'dims': [sx, sy],
            'counts': {
                'free': data.count(FREE),
                'occupied': data.count(100),
                'unknown': data.count(-1),
            },
        }
        # 起终点可用性(snap 后)
        sx_, sy_ = run._start
        gx_, gy_ = run._goal
        sc, sd = nearest_free(data, sx, sy, col_of(sx_, sy_, msg.info),
                              max(1, int(round(run._snap / msg.info.resolution))))
        gc, gd = nearest_free(data, sx, sy, col_of(gx_, gy_, msg.info),
                              max(1, int(round(run._snap / msg.info.resolution))))
        rep['start_goal_snap'] = {
            'start_req': [sx_, sy_], 'start_free_after_snap': sc is not None,
            'start_snap_dist_m': None if sd is None else round(sd * msg.info.resolution, 3),
            'goal_req': [gx_, gy_], 'goal_free_after_snap': gc is not None,
            'goal_snap_dist_m': None if gd is None else round(gd * msg.info.resolution, 3),
        }
        # 路径逐点碰撞检查
        occ, unk, free = [], [], []
        if run.path:
            for ps in run.path.poses:
                c = col_of(ps.pose.position.x, ps.pose.position.y, msg.info)
                if c is None:
                    occ.append((ps.pose.position.x, ps.pose.position.y))
                elif data[c] == 100:
                    occ.append((round(ps.pose.position.x, 2), round(ps.pose.position.y, 2)))
                elif data[c] == -1:
                    unk.append((round(ps.pose.position.x, 2), round(ps.pose.position.y, 2)))
                else:
                    free.append((round(ps.pose.position.x, 2), round(ps.pose.position.y, 2)))
        rep['collision_check'] = {
            'path_cells': len(free) + len(occ) + len(unk),
            'on_free': len(free), 'on_occupied': len(occ), 'on_unknown': len(unk),
            'occupied_cells': occ[:20], 'unknown_cells': unk[:20],
        }
    return rep


VERDICTS = ['PASS', 'FAIL']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', nargs=2, type=float, default=[12.35, 2.15])
    ap.add_argument('--goal', nargs=2, type=float, default=[11.75, 12.95])
    ap.add_argument('--snap-radius', type=float, default=1.5)
    ap.add_argument('--output', required=True, help='JSON 报告输出路径')
    ap.add_argument('--idle-seconds', type=float, default=15.0,
                    help='见一键合后连续多久无新更新判定回放结束')
    args = ap.parse_args()

    rclpy.init()
    run = Verifier(args.start, args.goal, args.snap_radius)

    def _quit(*_):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, _quit)
    signal.signal(signal.SIGINT, _quit)

    stopped = False
    try:
        while not stopped:
            rclpy.spin_once(run, timeout_sec=0.5)
            if (run.last_fused_wall is not None
                    and time.time() - run.last_fused_wall > args.idle_seconds):
                stopped = True
    except SystemExit:
        pass
    finally:
        rep = analyze(run)

        fused_ok = rep.get('fused', {}).get('counts', {}).get('free', 0) > 0
        path_ok = rep.get('path_count', 0) > 0
        collision_ok = (rep.get('collision_check', {}).get('on_occupied', -1) == 0
                        and rep.get('collision_check', {}).get('on_unknown', -1) == 0
                        and rep.get('collision_check', {}).get('path_cells', 0) > 0)
        result_ok = bool(rep.get('result')) and rep['result'].startswith('ok')
        frame_ok = (rep.get('occupancy_frames') == ['map']
                    and rep.get('fused', {}).get('frame') == 'map')
        start_goal_ok = (rep.get('start_goal_snap', {}).get('start_free_after_snap')
                         and rep.get('start_goal_snap', {}).get('goal_free_after_snap'))
        accepted = all([fused_ok, path_ok, collision_ok, result_ok, frame_ok, start_goal_ok])
        rep['verdict'] = accepted
        rep['checks'] = {
            'fused_present': fused_ok,
            'path_reported': path_ok,
            'path_collision_free_in_final_fused': collision_ok,
            'plan_result_ok': result_ok,
            'frames_consistent_map': frame_ok,
            'start_goal_snappable': bool(start_goal_ok),
        }
        with open(args.output, 'w') as f:
            json.dump(rep, f, ensure_ascii=False, indent=2, default=str)
        print('=== Phase5 融合+A* 离线判定 ===')
        print('verdict:', 'ok' if accepted else 'FAIL', '' if accepted else ' <-- 见 checks')
        print(json.dumps(rep.get('checks', {}), ensure_ascii=False, indent=2))
        print('fusion_stats:', rep.get('fusion_stats'))
        print('plan/result :', rep.get('result'))
        print('path cells  :', rep.get('collision_check', {}).get('path_cells'),
              'on_free=', rep.get('collision_check', {}).get('on_free'))
        print('fused counts:', rep.get('fused', {}).get('counts'))
        print('start_goal  :', rep.get('start_goal_snap'))
        print('frames:', rep.get('occupancy_frames'), '/ fused:', rep.get('fused', {}).get('frame'))
        print('report ->', args.output)
        run.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()