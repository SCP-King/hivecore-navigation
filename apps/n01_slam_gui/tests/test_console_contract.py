#!/usr/bin/env python3
"""Contract checks for the 2026-11-09 single-page grid console."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "slam_lite_gui.py").read_text()
fast_lio_helper = (ROOT.parents[1] / "scripts" / "start_fast_lio_gui_backend.sh").read_text()
loc_cfg = (ROOT / "config" / "n01_localization_lite.yaml").read_text()
map_cfg = (ROOT / "config" / "n01_mapping_lite.yaml").read_text()
nav_repo = ROOT.parents[1]
static_cfg = (nav_repo / "config" / "n01_static_tf_params.yaml")

checks = {
    "single page builder": "def build_console_page" in source
                           and "build_console_page()" in source,
    "data chain tabs present": all(x in source for x in (
        "ui.tabs(", "数据链路", "建图", "定位", "导航", "诊断",
        "build_data_chain_panel", "CHAIN_COLUMNS")),
    "data chain green/gray/red": all(x in source for x in (
        '"green"', '"gray"', '"red"', "chain-wrap")),
    "projected grid image": "ui.interactive_image(" in source
                            and "投影栅格" in source,
    "map click sets goal": "on_map_click" in source and "goal_x" in source,
    "goal publisher": 'GOAL_TOPIC = "/move_base_simple/goal"' in source
                      and "request_goal(" in source
                      and "PoseStamped" in source,
    "manual SLAM services": "LOC_SRV_DESC" in source and "MAP_SRV_DESC" in source
                            and "srv_click" in source,
    "runtime map path load": "ros2\", \"param\", \"set\"" in source
                             and "apply_map_path" in source,
    "localization uses 0.90": "tracking_to_vehicle_xyz: [0.0, 0.0, 0.9]" in loc_cfg,
    "mapping uses 0.90": "tracking_to_vehicle_xyz: [0.0, 0.0, 0.9]" in map_cfg,
    "desk passes unified params": "static_tf_params_file:={STATIC_TF_PARAMS}" in source
                                  and "LOCALIZATION_PARAMS" in source
                                  and "MAPPING_PARAMS" in source,
    "static TF YAML present": static_cfg.is_file()
                               and "auto_calibrate_from_topic: false" in static_cfg.read_text(),
    "no actuator publisher": '"/cmd_vel"' not in source and "'/cmd_vel'" not in source,
    "worker grid/target callbacks exist": all(
        f"def {name}(" in source for name in (
            "_on_nav_static", "_on_nav_dynamic", "_on_nav_ready",
            "_on_goal_status", "_on_nav_health", "request_goal", "_publish_goal")),
    "ROS failure reason displayed": "ros_reason" in source
                                      and "启动失败：" in source,
    "map path waits for localization": "loc_node_up" in source
                                       and "尚未上线" in source
                                       and "Node not found" in source,
    "modular page sections": all(x in source for x in (
        "整体项目生命周期", "共享地图视图", "① 建图模块",
        "② 定位模块", "③ 导航模块")),
    "global lifecycle controls": "启动建图定位" in source
                                  and "停止全部（保存建图）" in source,
    "dual localization backends": all(x in source for x in (
        'BACKEND_VENDOR = "甲方 SLAM3D Lite"',
        'BACKEND_FAST_LIO = "自部署 FAST-LIO2"',
        "start_fast_lio_gui_backend.sh"))
        and "fast_lio_nav_bridge.launch.py" in fast_lio_helper
        and "fast_lio_mapping.launch.py" in fast_lio_helper
        and "fast_lio_realtime_nav_bridge.launch.py" in fast_lio_helper,
    "FAST-LIO offline and realtime modes": all(x in source for x in (
        'FAST_MODE_OFFLINE = "离线（保存地图后定位导航）"',
        'FAST_MODE_REALTIME = "实时（边建图边导航）"',
        '"N01_NAV_MAP_MODE": "realtime" if realtime else "offline"')),
    "backend aware RViz restored": all(x in source for x in (
        "def start_selected_rviz", "def start_fast_lio_rviz",
        "FAST-LIO2 重定位 RViz", 'ui.button("RViz", icon="visibility"')),
    "compact expandable status": all(x in source for x in (
        "status-grid", "compact-expansion", "高级定位状态",
        "高级导航状态", "完整诊断信息")),
    "single adapter stage": all(x in source for x in (
        "总控 / 适配器", "适配器链路", "/n01/localization/pose",
        "/n01/localization/status", "/body_pose")),
    "localization match/freshness": all(x in source for x in (
        "阈值 0.045", 'UI["loc_match"]', 'UI["loc_pose_age"]',
        'UI["l_fresh"]', 'UI["loc_map_loaded"]')),
    "Chinese service labels": all(x in source for x in (
        "激活定位", "进入跟踪", "进入重定位", "停止定位服务",
        "激活建图服务", "保存地图快照")),
    "navigation Chinese status": all(x in source for x in (
        "导航就绪", "阻塞项", "目标门控")),
    "nav map/ground manual inputs": all(x in source for x in (
        "nav_pcd", "nav_ground_z", "N01_NAV_PCD", "N01_NAV_GROUND_Z",
        "nav_pcd_match", "nav_ground_match", "sync_nav_pcd", "sync_nav_ground")),
    "nav headless rviz": "N01_NAV_RVIZ" in source and '"false"' in source,
    "Pillow fromarray without deprecated mode": 'Image.fromarray(rgb[::-1], mode=' not in source,
    "X display is probed before RViz": "_x_display_works" in source
                                      and '["xdpyinfo", "-display"' in source
                                      and "启动后立即退出" in source,
    "nav candidate path is displayed": "_on_plan_path" in source
                                       and "/n01/live_navigation/hive2_candidate_path" in source
                                       and "局部路径" in source
                                       and 'GRID_COLORS["plan"' in source,
    "goal gate uses last result": "last_goal_accepted" in source
                                  and '"已接受"' in source,
    "last path retained until next plan": all(x in source for x in (
        "plan_stale", "plan_pending", "最近 {plan_pts} 点", "已过期",
        "等待新路径")),
    "goal waits for fresh path": all(x in source for x in (
        "plan_success_ns", "path_stamp_ns + 1_000_000_000",
        "全局 A* 阶段", "旧路径/过期路径不计入成功")),
    "goal reports failure reason": "_goal_failure_reason" in source
                                   and "规划器反馈" in source
                                   and "Goal Gate 拒绝" in source,
    "goal endpoint tolerance": "N01_PATH_GOAL_TOL_M" in source
                               and "路径终点偏差" in source,
    "failed goal marker": "goal_failed" in source
                          and 'GRID_COLORS["goal_failed"' in source,
    "global path is separate from local": all(x in source for x in (
        "GLOBAL_PATH_TOPIC", "global_pts", "nav_global_plan",
        "全局路径", "全局 {g_pts} 点", "局部 {l_pts} 点")),
    "goal timeout and pending guard": all(x in source for x in (
        "N01_GLOBAL_TIMEOUT_S", "N01_LOCAL_TIMEOUT_S",
        "全局 A* 阶段", "允许重新选点")),
}
failed = []
for name, ok in checks.items():
    print(("PASS " if ok else "FAIL ") + name)
    if not ok:
        failed.append(name)
raise SystemExit(1 if failed else 0)
