#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slam_lite_gui — slam3d lite 可视化操作台（Web GUI: nicegui + rclpy）

只读展示 + 显式按钮调用。所有 frame / TF / 话题 / 服务数据都来自甲方
发布端，本 GUI 不生成任何 frame/TF/话题，也不自动启动 SLAM 节点
（"节点管理"面板需用户勾选后手动触发）。

运行前置：推荐使用同目录 run_slam_lite_gui.sh 启动；脚本会 source
/opt/slam3d_release/setup_runtime.sh，然后浏览器打开：
    http://<本机IP>:8123
"""

from __future__ import annotations

import asyncio
import atexit
import base64
import builtins
import functools
import io
import json
import math
import os
import queue
import socket
import signal
import subprocess
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

APP_DIR = Path(__file__).resolve().parent
REPO_ROOT = APP_DIR.parents[1]

# ---------------------------------------------------------------------------
# 甲方 lite 消息类型（依赖 setup_runtime.sh 的环境）
# ---------------------------------------------------------------------------
_ENV_OK = False
_ENV_REASON = ""
_LOC_MSG = None
_MAP_MSG = None
try:
    from hivecore_slam3d_lite_localization.msg import LocStatus as _LOC_MSG  # noqa: F401
    from hivecore_slam3d_lite_mapping.msg import MappingStatus as _MAP_MSG  # noqa: F401
    _ENV_OK = True
except Exception as exc:  # noqa: BLE001
    _ENV_REASON = f"{type(exc).__name__}: {exc}"

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Vector3Stamped
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry, Path as RosPath
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

# ---------------------------------------------------------------------------
# 常量 / 接口清单
# ---------------------------------------------------------------------------
def _detect_rosk() -> str:
    """slam3d release 根目录。

    优先 $SLAM_ROSK；否则从 AMENT_PREFIX_PATH 里找 lite console 所在树
    （source 哪棵树就用哪棵）；都没有时退回 /opt/slam3d_release。
    """
    env = os.environ.get("SLAM_ROSK", "").strip()
    if env:
        return env.rstrip("/")
    tail = "/install/hivecore_slam3d_lite_console"
    for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
        if p.endswith(tail):
            return p[: -len(tail)]
    return "/opt/slam3d_release"


ROSK = _detect_rosk()


def _has_rosk_prefix() -> bool:
    """True when AMENT_PREFIX_PATH contains packages from the selected release."""
    root = os.path.join(ROSK, "install").rstrip("/") + "/"
    return any(prefix.startswith(root)
               for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":")
               if prefix)
MAX_PTS_VIEW = 6000          # 每帧下采样显示点数
TRAIL_MAX = 400              # 轨迹保留点数
LOC_NODE = "/localization_lite_node"
MAP_NODE = "/mapping_lite_node"
BACKEND_VENDOR = "甲方 SLAM3D Lite"
BACKEND_FAST_LIO = "自部署 FAST-LIO2"
BACKEND_OPTIONS = [BACKEND_VENDOR, BACKEND_FAST_LIO]
FAST_MODE_OFFLINE = "离线（保存地图后定位导航）"
FAST_MODE_REALTIME = "实时（边建图边导航）"
DEFAULT_FAST_LIO_MAP = "/home/demo/lyy/n01_runtime/slam/fast_lio_map.pcd"
INITIAL_POSE_TOPIC = "/initialpose"
GOAL_TOPIC = "/move_base_simple/goal"
GLOBAL_PATH_TOPIC = "/n01/navigation/path"
LOCAL_PATH_TOPIC = "/n01/live_navigation/hive2_candidate_path"
# Mapping and its replay bag are one lifecycle: one start, one close.
MAPPING_BAG_DIR = os.path.abspath(os.path.expanduser(
    os.environ.get("N01_MAPPING_BAG_DIR", "~/lyy/slam_data/bags")))
_DEFAULT_BAG_TOPICS = (
    "/livox/lidar", "/livox/imu", "/livox/lidar/geo_filtered",
    "/imu/data", "/imu/rpy/filtered", "/odom",
    "/tf", "/tf_static", "/robot_pose",
    "/mapping_status", "/cloud_map",
)
MAPPING_BAG_TOPICS = tuple(
    item.strip() for item in os.environ.get(
        "N01_MAPPING_BAG_TOPICS", ",".join(_DEFAULT_BAG_TOPICS)).split(",")
    if item.strip()) or _DEFAULT_BAG_TOPICS
FITNESS_THRESHOLD = 0.045
# Unified A: base_link is 0.90 above base_footprint; ground is 0.778 below base_link.
GROUND_OFFSET_FROM_FOOTPRINT = 0.90 - 0.778  # +0.122 m
GRID_RES = 0.10
GRID_IMAGE_PX = 640

# 会话配色（dark 面）
COL_IDLE = "#8a8a85"
COL_RELOC = "#c98500"
COL_TRACK = "#199e70"
COL_MAP = "#9ec5f4"
COL_OK = "#199e70"
COL_WARN = "#c98500"
COL_ERR = "#e66767"
ACCENT = "#d95926"
TRAIL_C = "#c3c2b7"
SURFACE = "#1a1a19"
TEXT = "#f0efec"
MUTED = "#9a9a92"
GRID = "#2a2a28"

LOC_SRV_DESC = {
    "load_map": "重新读取 map_path 参数并加载地图（成功后回重定位）",
    "activate_localization": "激活定位：空闲 → 重定位",
    "deactivate_localization": "停用定位：不再发布位姿 / TF / 地图",
    "enter_tracking": "进入追踪（先确认重定位已收敛）",
    "enter_relocalization": "回到重定位（保留位姿板）",
}
MAP_SRV_DESC = {
    "activate_mapping": "开始建图（从零开始）",
    "deactivate_mapping": "结束建图：保存 PCD、停止录包、清空内存",
    "save_map": "保存地图快照（不停止录包）",
}

TOPICS_DEF = [
    ("/loc_status", "hivecore_slam3d_lite_localization/msg/LocStatus", "订阅",
     "定位会话 / 地图 / 收敛状态"),
    ("/robot_pose", "nav_msgs/msg/Odometry", "订阅", "map 下的定位位姿（仅活动会话）"),
    ("/body_pose", "nav_msgs/msg/Odometry", "订阅", "导航输入（定位适配 + 运动补偿）"),
    ("/localization_map", "sensor_msgs/msg/PointCloud2", "订阅", "已加载 PCD（2s/次，体素 0.2m）"),
    ("/mapping_status", "hivecore_slam3d_lite_mapping/msg/MappingStatus", "订阅", "建图会话 idle | mapping"),
    ("/cloud_map", "sensor_msgs/msg/PointCloud2", "订阅", "建图累计点云（2s/次）"),
    ("/livox/lidar/geo_filtered", "sensor_msgs/msg/PointCloud2", "订阅", "SLAM 输入云（判断上游存活）"),
    ("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped", "发布", "仅重定位会话接受"),
]
SERVICES_DEF = [
    (LOC_NODE + "/load_map", "std_srvs/srv/Trigger"),
    (LOC_NODE + "/activate_localization", "std_srvs/srv/Trigger"),
    (LOC_NODE + "/deactivate_localization", "std_srvs/srv/Trigger"),
    (LOC_NODE + "/enter_tracking", "std_srvs/srv/Trigger"),
    (LOC_NODE + "/enter_relocalization", "std_srvs/srv/Trigger"),
    (MAP_NODE + "/activate_mapping", "std_srvs/srv/Trigger"),
    (MAP_NODE + "/deactivate_mapping", "std_srvs/srv/Trigger"),
    (MAP_NODE + "/save_map", "std_srvs/srv/Trigger"),
]

SEVERITY = {"info": "INFO", "ok": " OK ", "warn": "WARN", "err": "ERR "}


# ---------------------------------------------------------------------------
# 节点体检：重名检测 + 宿主进程解析（只读；停止需显式确认）
# ---------------------------------------------------------------------------
# 节点名 → 命令行特征。只用于把 ROS 节点映射到宿主进程，不用于启动任何东西。
# 注意 robot_state_publisher 在 hivecore_robot_description 里的可执行文件名是
# mount_tf_publisher_node，所以两个特征都要匹配。
NODE_EXEC_HINTS: dict[str, tuple[str, ...]] = {
    "localization_lite_node": ("localization_lite_node",),
    "mapping_lite_node": ("mapping_lite_node",),
    "complementary_filter_gain_node": ("complementary_filter_node",),
    "lidar_cloud_preprocess_node": ("lidar_cloud_preprocess_node",),
    "lidar_cloud_geometric_filter_z_node": ("lidar_cloud_geometric_filter_node",),
    "static_tf_publisher_node": ("static_tf_publisher_node",),
    "robot_state_publisher": ("mount_tf_publisher_node", "robot_state_publisher"),
    "vis_world_localization": ("vis_world_localization",),
    "vis_world_mapping": ("vis_world_mapping",),
    "foxglove_bridge": ("foxglove_bridge",),
    # Display matching only: _PROC_DENY keeps all Livox processes protected.
    "livox_lidar_publisher": ("livox_ros_driver2_node",),
}
# 命令行命中这些特征 → 一律不列入可停止目标（甲方底盘 / 腿控 / 手柄 / 传感器）。
_PROC_DENY = (
    "/opt/hivecore", "livox", "chassis", "controller_manager", "hive2_leg",
    "legs_", "joy_", "battery", "wheel", "ethercat", "motor", "lpms",
    "person_follow", "steer_drive", "slam_lite_gui",
)
# 关键话题：同一话题出现多个发布者 = 同一路数据有多个权威源。
CENSUS_TOPICS = (
    "/imu/data", "/imu/rpy/filtered", "/livox/imu", "/livox/lidar",
    "/livox/lidar/geo_filtered",
    "/odom", "/loc_status", "/robot_pose", "/localization_map", "/cloud_map",
    "/mapping_status", "/n01/localization/pose", "/n01/localization/status",
    "/cloud", "/body_pose", "/planning/global_path",
    "/n01/live_navigation/hive2_candidate_path",
    "/n01/live_navigation/readiness", "/grid_map/static_occupancy",
    "/grid_map/occupancy",
    # 静态 TF 出现多个发布者 = 重复的 robot_state_publisher，属必须可见的
    # 多源问题（GUI 本身只订阅，不发布 TF）。不在 CHAIN_COLUMNS 中，
    # 只参与普查，不影响任何节点的“新鲜度”判定。
    "/tf_static",
)
# desk 的前置链节点：起 new desk 前若已存在，说明有旧 desk 残留（或外部终端在跑）。
DESK_CHAIN_NODES = (
    "/complementary_filter_gain_node", "/lidar_cloud_preprocess_node",
    "/static_tf_publisher_node", "/robot_state_publisher",
)
FAST_LIO_NODES = (
    "/n01/fast_lio/mapping", "/n01_fast_lio_relocalizer",
    "/n01_fast_lio_pose_adapter",
)
# 看门狗：GUI 进程消失时自动停掉整个 desk 进程组，避免 launch 被收养后
# complementary_filter 等叶子以 ppid=1 孤儿形式继续发布。
GUARD_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "tools", "desk_guard.py")


def _abs_name(name: str, ns: str) -> str:
    ns = (ns or "/").rstrip("/")
    return f"{ns}/{name}" if ns else f"/{name}"


def _proc_start_ticks(pid: int) -> int | None:
    """Linux /proc starttime（字段 22），用于防止 PID 复用误发信号。"""
    try:
        with open(f"/proc/{pid}/stat", "rb") as fh:
            raw = fh.read().decode("ascii", "replace")
        fields = raw.rsplit(") ", 1)[1].split()
        return int(fields[19])
    except (OSError, IndexError, ValueError):
        return None


def _safe_signal_record(record: dict, sig: int) -> bool:
    """只向快照中同一 PID、同一启动时刻的进程发信号。"""
    pid = int(record["pid"])
    expected = record.get("start_ticks")
    if expected is None or _proc_start_ticks(pid) != expected:
        return False
    try:
        os.kill(pid, sig)
        return True
    except ProcessLookupError:
        return False


def _ps_snapshot() -> list[dict]:
    """一次 ps 快照：(pid, ppid, pgid, etime, cmd)。失败返回空列表。"""
    try:
        out = subprocess.run(["ps", "-eo", "pid=,ppid=,pgid=,etime=,cmd="],
                             capture_output=True, text=True, timeout=5)
    except Exception:  # noqa: BLE001
        return []
    rows = []
    for line in out.stdout.splitlines():
        parts = line.strip().split(None, 4)
        if len(parts) < 5:
            continue
        try:
            pid, ppid, pgid = int(parts[0]), int(parts[1]), int(parts[2])
        except ValueError:
            continue
        start_ticks = _proc_start_ticks(pid)
        rows.append({"pid": pid, "ppid": ppid, "pgid": pgid,
                     "etime": parts[3], "cmd": parts[4],
                     "start_ticks": start_ticks})
    return rows


def _is_protected(cmd: str) -> bool:
    return any(tok in cmd for tok in _PROC_DENY)


def _pid_alive(pid: int) -> bool:
    """进程是否还活着。僵尸（Z）算已死——它只剩一个 pid，不再执行任何代码。"""
    try:
        with open(f"/proc/{pid}/stat", "rb") as f:
            raw = f.read().decode("ascii", "replace")
        rest = raw.rsplit(") ", 1)[1].split()
        if rest and rest[0] == "Z":
            return False
        return True
    except FileNotFoundError:
        return False
    except (OSError, IndexError):
        pass                      # /proc 不可读时退回 kill(0) 探测
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _fmt_proc(p: dict) -> str:
    """宿主进程 → “pid 123（pgid 123，up 01:02）”。

    pgid 用来判断两个同名节点是不是同一个 launch 拉起的；up 用来区分
    “谁更老”——重名时通常该停的是启动更早的那一份。
    """
    extra = []
    if p.get("pgid"):
        extra.append(f"pgid {p['pgid']}")
    if p.get("etime"):
        extra.append(f"up {p['etime']}")
    return "pid " + str(p["pid"]) + ("（" + "，".join(extra) + "）" if extra else "")


def _node_hints(node_name: str) -> tuple[str, tuple | None]:
    """节点绝对名 → (匹配用短名, 白名单 hints)。命名空间节点按最后一段匹配。"""
    base = node_name.lstrip("/")
    short = base.rsplit("/", 1)[-1]
    return short, (NODE_EXEC_HINTS.get(base) or NODE_EXEC_HINTS.get(short))


def host_procs(node_name: str, snap: list[dict] | None = None) -> tuple[list[dict], bool]:
    """节点 → (宿主进程, 是否命中可停止白名单)。只读，仅供展示。

    白名单外的节点（例如 N01 导航栈）按节点名做宽松匹配，仅用于在体检页
    指出是哪个 pid/pgid 占了同名节点；本 GUI 不会停止白名单外的进程。
    """
    short, hints = _node_hints(node_name)    # 命名空间节点按最后一段匹配
    known = hints is not None
    if not known:
        if len(short) < 5:     # /tf、/map 这类短名匹配面太大，宁可不猜
            return [], False
        hints = (short,)
    snap = _ps_snapshot() if snap is None else snap
    me = os.getpid()
    return ([r for r in snap if r["pid"] != me and any(h in r["cmd"] for h in hints)],
            known)


# 未托管残留识别仅用于只读展示。PPID=1 不能证明进程属于本 GUI，
# 因此本 GUI 不会自动向这类进程发送信号；只有 ProcManager 登记的专属会话组可自动清理。
UNMANAGED_HINTS: tuple[str, ...] = tuple(dict.fromkeys(
    [h for hints in NODE_EXEC_HINTS.values() for h in hints]
    + ["localization_lite_desk.launch.py", "mapping_lite_desk.launch.py",
       "desk_guard.py"]
))

def collect_unmanaged_residue(snap: list[dict] | None = None) -> list[dict]:
    """只读列出疑似未托管残留；PPID=1 不是所有权证明，禁止自动清理。"""
    snap = _ps_snapshot() if snap is None else snap
    me = os.getpid()
    try:
        my_pgid = os.getpgid(0)
    except ProcessLookupError:
        my_pgid = -1
    out = []
    for r in snap:
        if r["pid"] == me or r.get("ppid") != 1:
            continue
        if r.get("pgid") == my_pgid or _is_protected(r["cmd"]):
            continue
        if any(h in r["cmd"] for h in UNMANAGED_HINTS):
            out.append(r)
    return out


def node_kill_plan(node_name: str, snap: list[dict] | None = None) -> dict:
    """节点 → {宿主进程, 可停止进程, 受保护进程, 相关 launch 父进程}。

    纯计算（ps 快照可注入，便于测试）。不发送任何信号。
    """
    short, hints = _node_hints(node_name)
    if not hints:
        return {"node": node_name, "supported": False, "procs": [],
                "killable": [], "protected": [], "launch": []}
    snap = _ps_snapshot() if snap is None else snap
    by_pid = {r["pid"]: r for r in snap}
    me = os.getpid()
    procs = [r for r in snap
             if r["pid"] != me and any(h in r["cmd"] for h in hints)]
    protected = [r for r in procs if _is_protected(r["cmd"])]
    managed_pgids = set()
    manager = globals().get("PROCS")
    if manager is not None:
        with manager.lock:
            managed_pgids = {int(item.get("pgid") or item["proc"].pid)
                             for item in manager.procs.values()}
    candidates = [r for r in procs if not _is_protected(r["cmd"])]
    killable = [r for r in candidates if r.get("pgid") in managed_pgids]
    unmanaged = [r for r in candidates if r.get("pgid") not in managed_pgids]
    launch: list[dict] = []
    seen: set[int] = set()
    for r in killable:
        cur, depth = r, 0
        while depth < 8:
            par = by_pid.get(cur["ppid"])
            if par is None or par["pid"] == me or par.get("pgid") != r.get("pgid"):
                break
            if ".launch.py" in par["cmd"]:
                if par["pid"] not in seen:
                    seen.add(par["pid"])
                    launch.append(par)
                break
            cur = par
            depth += 1
    return {"node": node_name, "supported": True, "procs": procs,
            "killable": killable, "unmanaged": unmanaged,
            "protected": protected, "launch": launch}


def execute_node_kill(node_name: str) -> None:
    """SIGINT 相关 launch 父进程 + 叶子进程，3s 后对残余 SIGKILL。

    先杀 launch 是因为 desk 里 complementary_filter 是 respawn=True，
    只杀叶子会被 launch 立刻重新拉起。
    """
    plan = node_kill_plan(node_name)
    if not plan["supported"]:
        push_log("warn", f"{node_name}: 不在可停止白名单内（不是 SLAM desk 链节点）")
        return
    if plan["protected"]:
        push_log("warn", f"{node_name}: 命中保护名单，跳过 pid="
                          f"{[r['pid'] for r in plan['protected']]}")
    pids = {r["pid"] for r in plan["launch"]} | {r["pid"] for r in plan["killable"]}
    pids.discard(os.getpid())
    if not pids:
        push_log("warn", f"{node_name}: 没有找到可停止的宿主进程（已退出 / 无权限）")
        return
    push_log("info", f"停止 {node_name}: SIGINT -> {sorted(pids)}"
                      f"（含 launch {[r['pid'] for r in plan['launch']]}）")
    records = {r["pid"]: r for r in plan["launch"] + plan["killable"]}
    for pid in sorted(pids):
        record = records.get(pid)
        if record is not None:
            _safe_signal_record(record, signal.SIGINT)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if not any(_pid_alive(p) for p in pids):
            break
        time.sleep(0.2)
    left = [p for p in sorted(pids) if _pid_alive(p)]
    for pid in left:
        record = records.get(pid)
        if record is not None:
            _safe_signal_record(record, signal.SIGKILL)
    if left:
        push_log("warn", f"{node_name}: SIGINT 未生效，已 SIGKILL {left}")
    else:
        push_log("ok", f"{node_name}: 已停止")


# ---------------------------------------------------------------------------
# PointCloud2 解析
# ---------------------------------------------------------------------------
def pc2_to_xyz(msg: PointCloud2) -> np.ndarray | None:
    """按字段偏移把 PointCloud2 解析为 (N,3) float32。失败返回 None。"""
    if msg is None or not msg.data or msg.point_step <= 0:
        return None
    try:
        by_name = {f.name: f for f in msg.fields}
        if not all(n in by_name for n in ("x", "y", "z")):
            return None
        offs = [by_name[n].offset for n in ("x", "y", "z")]
        step = msg.point_step
        n = len(msg.data) // step
        if n == 0:
            return None
        arr = np.frombuffer(msg.data[: n * step], dtype=np.uint8).reshape(n, step)
        xyz = np.empty((n, 3), dtype=np.float32)
        for i, off in enumerate(offs):
            raw = arr[:, off: off + 4].copy()
            xyz[:, i] = raw.view(np.float32)[:, 0]
        if msg.is_bigendian:
            xyz = xyz.astype(">f4").astype(np.float32)
        return xyz
    except Exception as exc:  # noqa: BLE001
        push_log("warn", f"点云解析失败: {exc}")
        return None


# ---------------------------------------------------------------------------
# 共享状态
# ---------------------------------------------------------------------------
class AppState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.recv_t: dict[str, float] = {
            k: 0.0 for k in
            ("loc_status", "mapping_status", "pose", "loc_map", "cloud_map",
             "input_cloud", "nav_static", "nav_dynamic", "nav_ready",
             "goal_status", "nav_health", "plan_path", "feasibility", "global_path")
        }
        # 最近 2 s 内各话题的接收时间戳，用于在数据链上显示实时 Hz（只读监测）。
        self.rate_win: dict[str, list[float]] = {}
        self.loc = {
            "session": "—", "map_loaded": None, "reloc_ok": None,
            "reloc_fitness": None, "map_path": "—",
        }
        self.mapping = {"session": "—", "save_dir": "—"}
        self.pose = None            # (x, y, yaw)
        self.pose_z = None          # map<-base_footprint z
        self.pose_frame = ""
        self.trail: deque[tuple[float, float, float]] = deque(maxlen=TRAIL_MAX)
        self.loc_pts: np.ndarray | None = None
        self.loc_seq = 0
        self.cloud_pts: np.ndarray | None = None
        self.cloud_seq = 0
        self.nav_static_pts: np.ndarray | None = None
        self.nav_dyn_pts: np.ndarray | None = None
        self.nav_map_seq = 0
        self.nav_ready = None
        self.goal_status: dict = {}
        self.nav_health: dict = {}
        self.readiness: dict = {}   # 桥接 0.5s 诊断 JSON（各上游新鲜度门控）
        self.feasibility: dict = {}
        self.goal_target: tuple[float, float] | None = None
        self.goal_failed = False
        self.plan_goal_distance: float | None = None
        self.plan_xy: list[tuple[float, float]] = []
        self.plan_pts = 0
        self.plan_frame = ""
        self.plan_seq = 0
        self.plan_active = False      # latest candidate is inside its time window
        self.plan_stale = False       # last non-empty candidate expired
        self.plan_pending = False     # a new goal is waiting for its plan
        self.plan_goal_ns = 0          # wall-clock time of the newest goal
        self.plan_success_ns = 0       # wall-clock time a fresh active local path arrived
        self.plan_success_stamp_ns = 0  # Path header stamp of that success
        self.goal_seq = 0              # newest goal transaction
        self.global_xy: list[tuple[float, float]] = []
        self.global_pts = 0
        self.global_seq = 0
        self.global_frame = ""
        self.global_active = False
        self.global_stale = False
        self.global_pending = False
        self.global_success_ns = 0
        self.global_goal_distance: float | None = None
        self.ros_ready = False
        self.ros_error = ""     # rclpy 未能启动时的具体原因（页面显示用）
        self.cmdq: queue.Queue = queue.Queue()
        self.pending_initialpose = None
        self.pending_goal = None
        self.log_q: queue.Queue = queue.Queue()
        # 节点在位（由 /get_parameters 服务就绪推断，~1s 刷新）
        self.loc_node_up = False
        self.map_node_up = False
        # 节点普查（1 Hz，rclpy 图 API，只读）
        self.node_rows: list[dict] = []       # [{name,count,procs,protected,supported}]
        self.dup_groups: list[str] = []       # 出现 >1 次的节点名
        self.topic_pubs: dict[str, list[str]] = {}   # 话题 -> 发布者绝对名
        self.census_at = 0.0

    def recv(self, key: str) -> None:
        now = time.monotonic()
        with self.lock:
            self.recv_t[key] = now
            ts = self.rate_win.setdefault(key, [])
            ts.append(now)
            # 只保留 2 s 窗口内的事件；短列表不裁剪，长列表一次成型
            if len(ts) > 64:
                self.rate_win[key] = [t for t in ts if now - t <= 2.0]

    def age_of(self, key: str) -> float | None:
        with self.lock:
            t = self.recv_t.get(key, 0.0)
        return None if t == 0.0 else time.monotonic() - t

    def rate_of(self, key: str) -> float:
        """最近 2 s 内该话题的近似发布频率（一次性的规划结果可能为 0）。"""
        now = time.monotonic()
        with self.lock:
            ts = self.rate_win.get(key, ())
            cnt = sum(1 for t in ts if now - t <= 2.0)
        return cnt / 2.0


# Share the complete process state as well as the worker.  A few embedding
# launchers can load this file under more than one module name (for example
# ``__main__`` and ``slam_lite_gui``).  Reusing only the worker in that case
# leaves the UI reading a newly-created AppState while the worker updates the
# first module's AppState, which looks exactly like "ROS connected but GUI is
# waiting forever".  The builtins registry is process-local and PID-scoped.
_state_registry = getattr(builtins, "_N01_SLAM_GUI_PROCESS_STATE_REGISTRY", None)
if not isinstance(_state_registry, dict) or _state_registry.get("pid") != os.getpid():
    _state_registry = {"pid": os.getpid(), "state": AppState()}
    setattr(builtins, "_N01_SLAM_GUI_PROCESS_STATE_REGISTRY", _state_registry)
STATE = _state_registry["state"]


def push_log(sev: str, text: str) -> None:
    STATE.log_q.put(f"[{time.strftime('%H:%M:%S')}] {SEVERITY.get(sev, sev)} {text}")


def walk(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ---------------------------------------------------------------------------
# ROS 工作线程
# ---------------------------------------------------------------------------
class RosWorker:
    def __init__(self) -> None:
        self.stop_flag = threading.Event()
        self.node: Node | None = None
        self.thread: threading.Thread | None = None
        self.pending_calls: list = []      # (name, fut, cb, desc, t0)
        self.svc_wait: deque = deque()     # (kind, client, desc, cb, t0)
        self.srv_clients: dict = {}
        self.param_clients: dict = {}
        self.param_next = 0.0
        self.last_node_check = 0.0
        self.last_census = 0.0
        self.last_dup_sig: tuple = ()
        self.last_orphan_sig: tuple = ()
        self.pose_pub = None
        self.goal_pub = None
        self.executor = None
        self._owns_context = False
        # Event is authoritative across UI/module aliases; STATE.ros_ready is
        # mirrored for the page and diagnostics.
        self.connected = threading.Event()
        self.started_monotonic = 0.0

    # ---- 外部 API ----
    def start(self) -> None:
        # Process-wide singleton makes start() idempotent across module reloads.
        if self.thread is not None and self.thread.is_alive():
            connected_event = getattr(self, "connected", None)
            if connected_event is not None and connected_event.is_set():
                push_log("info", "ROS worker 已运行且已连接；复用当前唯一 worker")
                return
            # Never start a second executor while an older worker is alive.
            # But do not silently treat an unconnected live thread as success.
            elapsed = (time.monotonic() - self.started_monotonic
                       if self.started_monotonic else 0.0)
            msg = ("检测到旧版/未完成初始化的 ROS Worker 仍在运行；"
                   f"已运行 {elapsed:.1f}s，不能安全创建第二个 Worker。"
                   "请完全退出旧 GUI 进程后重新启动。")
            with STATE.lock:
                STATE.ros_ready = False
                STATE.ros_error = msg
            push_log("err", msg)
            return
        stale = [t for t in threading.enumerate()
                 if t.name in ("ros-worker", "n01-ros-worker")
                 and t.is_alive() and t is not self.thread]
        if stale:
            msg = (f"检测到 {len(stale)} 个无法归属到全局管理器的 ROS worker 线程；"
                   "为避免重复订阅和 executor 竞争，拒绝创建第二个 worker。"
                   "请完全退出 GUI 解释器后重新启动。")
            with STATE.lock:
                STATE.ros_error = msg
                STATE.ros_ready = False
            push_log("err", msg)
            return
        self.stop_flag.clear()
        self.connected.clear()
        self.started_monotonic = time.monotonic()
        with STATE.lock:
            STATE.ros_ready = False
            STATE.ros_error = ""
        self.thread = threading.Thread(target=self._run, name="ros-worker", daemon=True)
        self.thread.start()

    def stop(self, timeout: float = 5.0) -> bool:
        self.stop_flag.set()
        thread = self.thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=max(0.1, timeout))
        alive = bool(thread and thread.is_alive())
        if not alive:
            self.connected.clear()
        if alive:
            push_log("warn", f"ROS worker 在 {timeout:.1f}s 内未完成退出")
        return not alive

    def call_service(self, name: str, desc: str, cb) -> None:
        STATE.cmdq.put(("svc", name, desc, cb))

    def request_initialpose(self, x: float, y: float, yaw: float, frame: str) -> None:
        with STATE.lock:
            STATE.pending_initialpose = {"x": x, "y": y, "yaw": yaw, "frame": frame}
        push_log("info", f"排队发布 /initialpose  x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}° frame={frame}")

    def request_goal(self, x: float, y: float, yaw: float, frame: str = "map") -> None:
        with STATE.lock:
            STATE.pending_goal = {"x": x, "y": y, "yaw": yaw, "frame": frame}
            STATE.plan_pending = True
            STATE.plan_goal_ns = time.time_ns()
            STATE.goal_seq += 1
            STATE.plan_success_ns = 0
            STATE.plan_success_stamp_ns = 0
            STATE.goal_target = (float(x), float(y))
            STATE.goal_failed = False
            STATE.plan_goal_distance = None
            STATE.global_pending = True
            STATE.global_success_ns = 0
            STATE.global_goal_distance = None
            if STATE.global_pts:
                STATE.global_active = False
                STATE.global_stale = True
                STATE.global_seq += 1
            # The old path is not the new goal's result. Keep it for context,
            # but never let it count as a successful new plan.
            if STATE.plan_pts:
                STATE.plan_active = False
                STATE.plan_stale = True
                STATE.plan_seq += 1
        push_log("info", f"排队发布 /move_base_simple/goal  x={x:.2f} y={y:.2f} "
                         f"yaw={math.degrees(yaw):.1f}° frame={frame}；"
                         "等待该目标的新路径")

    # ---- 线程主体 ----
    def _run(self) -> None:
        """Top-level worker guard: never leave the UI in an endless starting state."""
        try:
            self._run_impl()
        except BaseException as exc:  # noqa: BLE001 - worker boundary must report all failures
            msg = f"ROS Worker 异常退出：{type(exc).__name__}: {exc}"
            self.connected.clear()
            with STATE.lock:
                STATE.ros_ready = False
                STATE.ros_error = msg
            push_log("err", msg)
            # Best-effort cleanup for failures during node/executor initialization or spin.
            try:
                if self.executor is not None:
                    if self.node is not None:
                        self.executor.remove_node(self.node)
                    self.executor.shutdown(timeout_sec=1.0)
            except Exception as cleanup_exc:  # noqa: BLE001
                push_log("warn", f"ROS executor 清理异常：{cleanup_exc}")
            try:
                if self.node is not None:
                    self.node.destroy_node()
            except Exception as cleanup_exc:  # noqa: BLE001
                push_log("warn", f"ROS node 清理异常：{cleanup_exc}")
            finally:
                self.node = None
                self.executor = None
                if self._owns_context:
                    try:
                        if rclpy.ok():
                            rclpy.shutdown()
                    except Exception as cleanup_exc:  # noqa: BLE001
                        push_log("warn", f"rclpy Context 清理异常：{cleanup_exc}")
                    self._owns_context = False
        finally:
            self.connected.clear()
            # A normal return without ROS readiness is also surfaced explicitly.
            with STATE.lock:
                if not STATE.ros_ready and not STATE.ros_error:
                    STATE.ros_error = "ROS Worker 已退出，但未建立有效 ROS 连接；请查看运行日志。"

    @staticmethod
    def _can_write_dir(d: str) -> bool:
        try:
            os.makedirs(d, exist_ok=True)
            probe = os.path.join(d, f".wtest_{os.getpid()}_{time.monotonic_ns()}")
            with open(probe, "w") as fh:
                fh.write("x")
            os.unlink(probe)
            return True
        except OSError:
            return False

    def _ensure_log_dir_writable(self) -> None:
        """rclpy.init 会在 $ROS_LOG_DIR（默认 ~/.ros/log）建日志文件。
        只读根分区 / 权限不足时 init 会直接抛 'Read-only file system'，
        GUI 就显示“ROS 未连接”。这里提前检测并切到 GUI 目录下的可写位置。"""
        current = os.environ.get("ROS_LOG_DIR") or os.path.expanduser("~/.ros/log")
        if self._can_write_dir(current):
            return
        fallback = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log", "ros")
        if not self._can_write_dir(fallback):
            push_log("warn", f"ROS 日志目录不可写：{current}，备选也失败：{fallback}")
            return
        os.environ["ROS_LOG_DIR"] = fallback
        push_log("warn", f"ROS 日志目录 {current} 不可写，已改用 {fallback}")

    def _init_rclpy_idempotent(self) -> str | None:
        """Initialize or safely share the process default Context.

        A pre-initialized default context can be used by this worker's dedicated
        executor. The worker only shuts it down if this worker initialized it.
        """
        if rclpy.ok():
            self._owns_context = False
            push_log("info", "检测到进程内已有有效 rclpy Context；使用专属 executor，不重复 init")
            return None
        try:
            rclpy.init(args=None)
            self._owns_context = True
            return None
        except Exception as exc:  # noqa: BLE001
            text = f"{type(exc).__name__}: {exc}"
            if "must only be called once" in text and rclpy.ok():
                self._owns_context = False
                push_log("info", "rclpy Context 已初始化；使用专属 executor")
                return None
            return f"rclpy.init 失败：{text}"

    def _run_impl(self) -> None:
        push_log("info", f"ROS worker 启动 pid={os.getpid()} thread={threading.get_ident()}")
        if not _ENV_OK:
            msg = (f"lite 消息类型不可用（{_ENV_REASON}）。"
                   f"请先 source {ROSK}/setup_runtime.sh 再启动本 GUI")
            with STATE.lock:
                STATE.ros_error = msg
            push_log("err", msg)
            return
        self._ensure_log_dir_writable()
        err = self._init_rclpy_idempotent()
        if err:
            with STATE.lock:
                STATE.ros_error = err
            push_log("err", err)
            return
        with STATE.lock:
            STATE.ros_error = ""
        # 节点名带线程标识：同一进程若残留旧 worker，两者不会撞名（rosout 重复注册警告）
        node = Node(f"n01_slam_lite_gui_{os.getpid() % 10000}_{threading.get_ident() % 1000}")
        self.node = node
        STATE.ros_ready = False
        q_cloud = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        q_small = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        q_static = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        q_status = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        q_path_global = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)

        for tool, name in (("loc", LOC_NODE), ("map", MAP_NODE)):
            for leaf in (LOC_SRV_DESC if tool == "loc" else MAP_SRV_DESC):
                self.srv_clients[leaf] = node.create_client(Trigger, f"{name}/{leaf}")
            self.param_clients[tool] = node.create_client(GetParameters, f"{name}/get_parameters")

        self._sub(node, "/loc_status", _LOC_MSG, q_small, self._on_loc_status)
        self._sub(node, "/mapping_status", _MAP_MSG, q_small, self._on_mapping_status)
        self._sub(node, "/robot_pose", Odometry, q_small, self._on_pose)
        self._sub(node, "/body_pose", Odometry, q_small, self._on_body_pose)
        self._sub(node, "/localization_map", PointCloud2, q_cloud, self._on_loc_map)
        self._sub(node, "/cloud_map", PointCloud2, q_cloud, self._on_cloud)
        self._sub(node, "/n01/fast_lio/map", PointCloud2, q_cloud, self._on_cloud)
        self._sub(node, "/n01/fast_lio/cloud_registered", PointCloud2, q_cloud,
                  self._on_loc_map)
        self._sub(node, "/livox/lidar/geo_filtered", PointCloud2, q_cloud, self._on_input_cloud)
        # Data-chain liveness topics (read-only; no lifecycle side effects).
        self._sub(node, "/livox/lidar", PointCloud2, q_cloud,
                  self._chain_cb("livox_lidar"))
        self._sub(node, "/livox/imu", Imu, q_small, self._chain_cb("livox_imu"))
        self._sub(node, "/imu/data", Imu, q_small, self._chain_cb("imu_data"))
        self._sub(node, "/imu/rpy/filtered", Vector3Stamped, q_small,
                  self._chain_cb("imu_rpy"))
        self._sub(node, "/odom", Odometry, q_small, self._chain_cb("odom"))
        self._sub(node, "/n01/localization/pose", Odometry, q_small,
                  self._chain_cb("n01_pose"))
        self._sub(node, "/n01/localization/status", DiagnosticArray, q_small,
                  self._chain_cb("n01_status"))
        self._sub(node, "/cloud", PointCloud2, q_cloud,
                  self._chain_cb("bridge_cloud"))
        self._sub(node, "/n01/live_navigation/readiness", String, q_small,
                  self._on_readiness)
        self._sub(node, "/n01/live_navigation/slow_motion/status", String, q_small,
                  self._on_slow_status)
        self._sub(node, "/grid_map/static_occupancy", PointCloud2, q_static, self._on_nav_static)
        self._sub(node, "/grid_map/occupancy", PointCloud2, q_cloud, self._on_nav_dynamic)
        self._sub(node, "/n01/live_navigation/navigation_ready", Bool, q_status, self._on_nav_ready)
        self._sub(node, "/n01/live_navigation/goal_gate_status", String, q_status, self._on_goal_status)
        self._sub(node, "/n01/live_navigation/system_health", String, q_status, self._on_nav_health)
        self._sub(node, GLOBAL_PATH_TOPIC, RosPath, q_path_global,
                  self._on_global_path)
        self._sub(node, LOCAL_PATH_TOPIC, RosPath,
                  q_status, self._on_plan_path)
        self._sub(node, "/n01/live_navigation/hive2_feasibility", String,
                  q_status, self._on_feasibility)
        self.pose_pub = node.create_publisher(PoseWithCovarianceStamped, INITIAL_POSE_TOPIC, 3)
        self.goal_pub = node.create_publisher(PoseStamped, GOAL_TOPIC, 3)

        push_log("info", f"ROS 节点 {node.get_name()} 已启动，开始订阅")

        # 用本线程专属 executor：rclpy.spin_once() 的默认全局 executor 是进程级单例，
        # 两个 worker 线程同时 spin 会报 "generator already executing"。
        executor = SingleThreadedExecutor()
        self.executor = executor
        executor.add_node(node)
        # Successful add_node is the GUI's ROS connection criterion.  It does
        # not depend on SLAM nodes/topics/services being present.
        self.connected.set()
        with STATE.lock:
            STATE.ros_ready = True
            STATE.ros_error = ""
        push_log("info", "ROS Worker 已连接：executor 已接管 GUI 节点")

        while not self.stop_flag.is_set() and rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.05)
            except Exception as exc:  # noqa: BLE001
                push_log("err", f"spin 异常: {exc}")
                time.sleep(0.2)
            self._drain_cmds()
            self._service_waiting()
            self._service_pending()
            self._publish_initialpose()
            self._publish_goal()
            self._poll_params()
            self._check_nodes()
            self._census()
        try:
            executor.remove_node(node)
            executor.shutdown()
        except Exception:  # noqa: BLE001
            pass
        self.executor = None
        self.connected.clear()

        try:
            node.destroy_node()
        finally:
            if self._owns_context and rclpy.ok():
                rclpy.shutdown()
            self._owns_context = False
            STATE.ros_ready = False
            push_log("info", "ROS 工作线程退出")

    def _sub(self, node, topic, msg_type, qos, cb) -> None:
        if msg_type is None:
            return
        try:
            node.create_subscription(msg_type, topic, cb, qos)
        except Exception as exc:  # noqa: BLE001
            push_log("warn", f"订阅 {topic} 失败: {exc}")

    def _chain_cb(self, key):
        def _cb(_msg):
            STATE.recv(key)
        return _cb

    # ---- 回调 ----
    def _on_loc_status(self, msg) -> None:
        with STATE.lock:
            STATE.loc["session"] = msg.session
            STATE.loc["map_loaded"] = bool(msg.map_loaded)
            STATE.loc["reloc_ok"] = bool(msg.reloc_ok)
            STATE.loc["reloc_fitness"] = float(msg.reloc_fitness)
        STATE.recv("loc_status")

    def _on_mapping_status(self, msg) -> None:
        with STATE.lock:
            STATE.mapping["session"] = msg.session
        STATE.recv("mapping_status")

    def _on_body_pose(self, msg: Odometry) -> None:
        with STATE.lock:
            STATE.body_pose_xyz = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z))
        STATE.recv("body_pose")

    def _on_slow_status(self, msg: String) -> None:
        with STATE.lock:
            STATE.slow_status = msg.data
        STATE.recv("slow_status")

    def _on_pose(self, msg: Odometry) -> None:
        p = msg.pose.pose
        yaw = walk(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        with STATE.lock:
            STATE.pose = (float(p.position.x), float(p.position.y), yaw)
            STATE.pose_z = float(p.position.z)
            STATE.pose_frame = msg.header.frame_id or ""
            STATE.trail.append((float(p.position.x), float(p.position.y), yaw))
        STATE.recv("pose")

    def _on_loc_map(self, msg: PointCloud2) -> None:
        xyz = pc2_to_xyz(msg)
        with STATE.lock:
            STATE.loc_pts = xyz
            STATE.loc_seq += 1
        STATE.recv("loc_map")

    def _on_cloud(self, msg: PointCloud2) -> None:
        xyz = pc2_to_xyz(msg)
        with STATE.lock:
            STATE.cloud_pts = xyz
            STATE.cloud_seq += 1
        STATE.recv("cloud_map")

    def _on_input_cloud(self, msg: PointCloud2) -> None:
        STATE.recv("input_cloud")

    def _on_nav_static(self, msg: PointCloud2) -> None:
        xyz = pc2_to_xyz(msg)
        with STATE.lock:
            STATE.nav_static_pts = xyz
            STATE.nav_map_seq += 1
        STATE.recv("nav_static")

    def _on_nav_dynamic(self, msg: PointCloud2) -> None:
        xyz = pc2_to_xyz(msg)
        with STATE.lock:
            STATE.nav_dyn_pts = xyz
            STATE.nav_map_seq += 1
        STATE.recv("nav_dynamic")

    def _on_nav_ready(self, msg) -> None:
        with STATE.lock:
            STATE.nav_ready = bool(msg.data)
        STATE.recv("nav_ready")

    @staticmethod
    def _json_or_empty(text: str) -> dict:
        try:
            value = json.loads(text)
            return value if isinstance(value, dict) else {}
        except (ValueError, TypeError):
            return {}

    def _on_goal_status(self, msg: String) -> None:
        with STATE.lock:
            STATE.goal_status = self._json_or_empty(msg.data)
        STATE.recv("goal_status")

    def _on_nav_health(self, msg: String) -> None:
        with STATE.lock:
            STATE.nav_health = self._json_or_empty(msg.data)
        STATE.recv("nav_health")

    def _on_readiness(self, msg: String) -> None:
        # 桥接的 0.5 s 诊断：保存下来，供数据链说明 /cloud、/body_pose 红色根因。
        with STATE.lock:
            STATE.readiness = self._json_or_empty(msg.data)
        STATE.recv("readiness")

    def _on_feasibility(self, msg: String) -> None:
        with STATE.lock:
            STATE.feasibility = self._json_or_empty(msg.data)
        STATE.recv("feasibility")

    def _on_global_path(self, msg: RosPath) -> None:
        points = [(float(p.pose.position.x), float(p.pose.position.y))
                  for p in msg.poses]
        stamp_ns = (int(msg.header.stamp.sec) * 1000000000
                    + int(msg.header.stamp.nanosec))
        with STATE.lock:
            if not points:
                # 空 Path 是“清空”信号（当前 scan_planner 实际不会发，属防御）：
                # 只清活动态，不再提前 return，保证 recv 与状态一致。
                if STATE.global_pts:
                    STATE.global_active = False
                    STATE.global_stale = True
                    STATE.global_seq += 1
            else:
                pending = STATE.global_pending
                goal_ns = STATE.plan_goal_ns
                old_replay = bool(
                    pending and goal_ns and stamp_ns
                    and stamp_ns + 1_000_000_000 < goal_ns)
                if old_replay:
                    STATE.global_stale = bool(STATE.global_pts)
                    STATE.global_seq += 1
                else:
                    STATE.global_xy = points
                    STATE.global_pts = len(points)
                    STATE.global_frame = msg.header.frame_id or ""
                    STATE.global_active = True
                    STATE.global_stale = False
                    STATE.global_pending = False
                    STATE.global_success_ns = time.time_ns()
                    target = STATE.goal_target
                    STATE.global_goal_distance = (
                        math.hypot(points[-1][0] - target[0],
                                   points[-1][1] - target[1])
                        if target is not None else None)
                    STATE.global_seq += 1
        STATE.recv("global_path")

    def _on_plan_path(self, msg: RosPath) -> None:
        points = [(float(p.pose.position.x), float(p.pose.position.y))
                  for p in msg.poses]
        path_stamp_ns = (int(msg.header.stamp.sec) * 1000000000
                         + int(msg.header.stamp.nanosec))
        with STATE.lock:
            if points:
                pending = STATE.plan_pending
                goal_ns = STATE.plan_goal_ns
                # The trajectory monitor replays the last active path. A replay
                # of the previous goal must not satisfy the new goal. Compare
                # the Path header stamp with the goal send time (1 s clock
                # tolerance for DDS/clock granularity).
                old_replay = bool(
                    pending and goal_ns and path_stamp_ns
                    and path_stamp_ns + 1_000_000_000 < goal_ns)
                if old_replay:
                    STATE.plan_pending = True
                    STATE.plan_active = False
                    STATE.plan_stale = bool(STATE.plan_pts)
                    STATE.plan_seq += 1
                else:
                    STATE.plan_xy = points
                    STATE.plan_pts = len(points)
                    STATE.plan_frame = msg.header.frame_id or ""
                    STATE.plan_active = True
                    STATE.plan_stale = False
                    STATE.plan_pending = False
                    STATE.plan_success_ns = time.time_ns()
                    STATE.plan_success_stamp_ns = path_stamp_ns
                    target = STATE.goal_target
                    if target is not None:
                        STATE.plan_goal_distance = math.hypot(
                            points[-1][0] - target[0], points[-1][1] - target[1])
                    else:
                        STATE.plan_goal_distance = None
                    STATE.plan_seq += 1
            elif STATE.plan_pts:
                # An empty Path clears the active window. Keep the last
                # non-empty path for display until the next point/plan.
                STATE.plan_active = False
                STATE.plan_stale = True
                STATE.plan_seq += 1
        STATE.recv("plan_path")

    # ---- 服务调用 ----
    def _drain_cmds(self) -> None:
        while True:
            try:
                kind, name, desc, cb = STATE.cmdq.get_nowait()
            except queue.Empty:
                return
            client = self.srv_clients.get(name)
            if client is None:
                push_log("err", f"{desc}: 未知服务 {name}")
                cb(False, "未知服务")
                continue
            self.svc_wait.append((name, client, desc, cb, time.monotonic()))

    def _service_waiting(self) -> None:
        keep = deque()
        for name, client, desc, cb, t0 in self.svc_wait:
            if time.monotonic() - t0 > 5.0:
                push_log("err", f"{desc}: 服务 /{name} 5s 内不可用（节点未启动？）")
                cb(False, "服务不可用")
                continue
            if not client.service_is_ready():
                keep.append((name, client, desc, cb, t0))
                continue
            try:
                fut = client.call_async(Trigger.Request())
            except Exception as exc:  # noqa: BLE001
                push_log("err", f"{desc}: 调用异常 {exc}")
                cb(False, str(exc))
                continue
            self.pending_calls.append((name, fut, cb, desc, time.monotonic()))
            push_log("info", f"已发送服务请求 {name}")
            cb(None, "已发送")   # 立即反馈“已发送”，结果稍后到
        self.svc_wait = keep

    def _service_pending(self) -> None:
        keep = []
        for name, fut, cb, desc, t0 in self.pending_calls:
            if not fut.done():
                if time.monotonic() - t0 > 10.0:
                    push_log("err", f"{desc}: 10s 无响应，放弃等待")
                    cb(False, "超时")
                    continue
                keep.append((name, fut, cb, desc, t0))
                continue
            try:
                resp = fut.result()
                ok = bool(resp.success)
                text = str(resp.message or "")
                push_log("ok" if ok else "err", f"{desc}: {'成功' if ok else '失败'} {text}")
                cb(ok, text)
            except Exception as exc:  # noqa: BLE001
                push_log("err", f"{desc}: 结果异常 {exc}")
                cb(False, str(exc))
        self.pending_calls = keep

    def _publish_initialpose(self) -> None:
        payload = None
        with STATE.lock:
            if STATE.pending_initialpose is not None:
                payload = STATE.pending_initialpose
                STATE.pending_initialpose = None
        if payload is None or self.pose_pub is None:
            return
        yaw = payload["yaw"]
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = payload["frame"]
        msg.pose.pose.position.x = payload["x"]
        msg.pose.pose.position.y = payload["y"]
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = cov[35] = 0.01
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)
        push_log("info", f"/initialpose 已发布（{payload['frame']}，仅重定位会话接受）")

    def _publish_goal(self) -> None:
        payload = None
        with STATE.lock:
            if STATE.pending_goal is not None:
                payload = STATE.pending_goal
                STATE.pending_goal = None
        if payload is None or self.goal_pub is None:
            return
        yaw = float(payload["yaw"])
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = str(payload["frame"])
        msg.pose.position.x = float(payload["x"])
        msg.pose.position.y = float(payload["y"])
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(msg)
        push_log("info", f"已发布导航目标 map ({msg.pose.position.x:.2f}, "
                         f"{msg.pose.position.y:.2f}) yaw={math.degrees(yaw):.1f}°")

    def _poll_params(self) -> None:
        now = time.monotonic()
        if now < self.param_next:
            return
        self.param_next = now + 10.0
        jobs = (("loc", ["map_path"]), ("map", ["save_dir"]))
        for tool, names in jobs:
            client = self.param_clients.get(tool)
            if client is None or not client.service_is_ready():
                continue
            try:
                req = GetParameters.Request()
                req.names = names
                fut = client.call_async(req)
                fut.add_done_callback(lambda f, t=tool: self._on_param_result(t, f))
            except Exception:  # noqa: BLE001
                pass

    def _check_nodes(self) -> None:
        """用节点参数服务是否就绪推断 loc/map 节点在位。1 Hz。"""
        now = time.monotonic()
        if now - self.last_node_check < 1.0:
            return
        self.last_node_check = now
        with STATE.lock:
            STATE.loc_node_up = bool(
                self.param_clients.get("loc") and self.param_clients["loc"].service_is_ready())
            STATE.map_node_up = bool(
                self.param_clients.get("map") and self.param_clients["map"].service_is_ready())

    def force_census(self) -> None:
        """让下一次 spin 循环立刻重做普查（页面“立即刷新”按钮）。"""
        self.last_census = 0.0

    def _census(self) -> None:
        """全图节点普查 + 关键话题发布者计数。2 Hz 上限，全部只读。

        数据源：rclpy 图 API（DDS 发现结果），不走 ros2 CLI，
        因此不需要 daemon、直接读取当前 ROS_DOMAIN_ID 的 DDS 发现结果。
        """
        now = time.monotonic()
        if now - self.last_census < 2.0 or self.node is None:
            return
        self.last_census = now
        try:
            pairs = self.node.get_node_names_and_namespaces()
        except Exception as exc:  # noqa: BLE001
            push_log("warn", f"节点普查失败: {exc}")
            return

        counts: dict[str, int] = {}
        for name, ns in pairs:
            key = _abs_name(name, ns)
            counts[key] = counts.get(key, 0) + 1
        dups = sorted(n for n, c in counts.items() if c > 1)

        pubs: dict[str, list[str]] = {}
        for topic in CENSUS_TOPICS:
            try:
                infos = self.node.get_publishers_info_by_topic(topic)
            except Exception:  # noqa: BLE001
                infos = []
            pubs[topic] = sorted({_abs_name(i.node_name, i.node_namespace) for i in infos})

        snap = _ps_snapshot()          # 每次普查只取一次宿主进程快照
        rows = []
        for name in sorted(counts):
            base = name.lstrip("/")
            supported = _node_hints(name)[1] is not None
            plan = node_kill_plan(base, snap) if supported else None
            protected = bool(plan and plan["protected"] and not plan["killable"] and not plan.get("unmanaged"))
            killable = plan["killable"] if plan else []
            unmanaged = plan.get("unmanaged", []) if plan else []
            # PPID=1 仅作为“父进程已退出”的提示；不能据此判断归属或自动停止。
            orphan_pids = [p["pid"] for p in (killable + unmanaged)
                           if p.get("ppid") == 1]
            if name.startswith("/n01_slam_lite_gui"):
                owner = "本 GUI 进程（线程）"
            elif plan and (plan["killable"] or unmanaged):
                visible = plan["killable"] + unmanaged
                owner = "、".join(_fmt_proc(p) for p in visible[:2])
                if len(visible) > 2:
                    owner += f" +{len(visible) - 2}"
                if plan["killable"]:
                    owner += "（本 GUI 受管，可停止）"
                elif unmanaged:
                    owner += "（外部/未登记进程，只展示）"
                if orphan_pids:
                    owner += "（PPID=1，父进程已退出）"
                if plan["protected"]:
                    owner += "（另有受保护 " + "、".join(
                        f"pid {p['pid']}" for p in plan["protected"][:2]) + "）"
            elif plan and plan["protected"]:
                owner = "受保护 " + "、".join(f"pid {p['pid']}"
                                             for p in plan["protected"][:2])
            elif supported:
                owner = "未找到宿主进程"
            else:
                # 白名单外（例如 N01 导航栈）：宽松匹配只为定位“谁占了同名节点”，
                # 本 GUI 不会停止这些进程。
                avail, _known = host_procs(base, snap)
                if avail:
                    owner = "、".join(_fmt_proc(p) for p in avail[:2])
                    if len(avail) > 2:
                        owner += f" +{len(avail) - 2}"
                    owner += "（仅展示，不在可停止白名单）"
                else:
                    owner = "—"
            if protected:
                state = "重复·受保护" if counts[name] > 1 else "受保护"
            elif counts[name] > 1:
                state = "重复·孤儿" if orphan_pids else "重复"
            elif orphan_pids:
                state = "孤儿（残留）"
            else:
                state = "正常"
            rows.append({
                "name": name,
                "count": counts[name],
                "owner": owner,
                "state": state,
                "killable": bool(plan and plan["killable"]),
            })

        with STATE.lock:
            STATE.node_rows = rows
            STATE.dup_groups = dups
            STATE.topic_pubs = pubs
            STATE.census_at = time.monotonic()
            detail_target = getattr(STATE, "node_detail_target", None)
        if detail_target:
            namespace, _, short_name = detail_target.rpartition("/")
            try:
                sections = []
                for title, getter in (
                        ("发布", self.node.get_publisher_names_and_types_by_node),
                        ("订阅", self.node.get_subscriber_names_and_types_by_node)):
                    entries = getter(short_name, namespace or "/")
                    sections.append(title + "：\n" + ("\n".join(
                        f"  {topic}  [{', '.join(types)}]" for topic, types in sorted(entries)) or "  —"))
                detail = detail_target + "\n" + "\n\n".join(sections)
            except Exception as exc:
                detail = f"{detail_target}：接口读取失败（节点可能已退出）：{exc}"
            with STATE.lock:
                if getattr(STATE, "node_detail_target", None) == detail_target:
                    STATE.node_detail = detail
        # 只在重名/孤儿集合变化时记日志：否则每 2 秒重复一条，把运行日志刷满
        sig = tuple(dups)
        if sig != self.last_dup_sig:
            if dups:
                push_log("warn", f"节点普查：发现 {len(dups)} 组重名节点 {dups}")
            elif self.last_dup_sig:
                push_log("ok", "节点普查：重名已消失")
            self.last_dup_sig = sig
        osig = tuple(r["name"] for r in rows if "孤儿" in r["state"])
        if osig != self.last_orphan_sig:
            if osig:
                push_log("warn", f"节点普查：未托管残留 {list(osig)}（ppid=1，父 launch "
                                  f"已死；所有权未确认，仅展示不自动清理）")
            elif self.last_orphan_sig:
                push_log("ok", "节点普查：未托管残留已清零")
            self.last_orphan_sig = osig

    def _on_param_result(self, tool: str, fut) -> None:
        try:
            vals = fut.result().values
            text = vals[0].string_value if vals else ""
        except Exception:  # noqa: BLE001
            return
        with STATE.lock:
            if tool == "loc":
                STATE.loc["map_path"] = text or "(空)"
            else:
                STATE.mapping["save_dir"] = text or "(空)"


# Keep one worker object per interpreter even if this module is imported/reloaded
# through an embedding server. A new OS process gets a new registry automatically.
_registry = getattr(builtins, "_N01_SLAM_GUI_ROS_WORKER_REGISTRY", None)
if not isinstance(_registry, dict) or _registry.get("pid") != os.getpid():
    _registry = {"pid": os.getpid(), "worker": RosWorker()}
    setattr(builtins, "_N01_SLAM_GUI_ROS_WORKER_REGISTRY", _registry)
WORKER = _registry["worker"]


# ---------------------------------------------------------------------------
# 可选节点管理（默认关闭）
# ---------------------------------------------------------------------------
class ProcManager:
    """管理 desk、RViz、Foxglove bridge；子进程继承已 source 的环境。"""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.procs: dict[str, dict] = {}
        self.desk_mode = ""
        self.bag_path = ""

    def running(self, key: str) -> bool:
        with self.lock:
            item = self.procs.get(key)
            return bool(item and item["proc"].poll() is None)

    def status_text(self) -> str:
        with self.lock:
            items = []
            for key, item in self.procs.items():
                if item["proc"].poll() is None:
                    items.append(item["desc"])
        return "、".join(items) if items else "无"

    def _spawn(self, key: str, cmd: list[str], desc: str, env=None) -> bool:
        with self.lock:
            old = self.procs.get(key)
            if old and old["proc"].poll() is None:
                push_log("warn", f"{desc} 已在运行")
                return False
            if not os.path.isfile(GUARD_SCRIPT):
                push_log("err", f"生命周期看门狗缺失：{GUARD_SCRIPT}；拒绝启动 {desc}")
                return False
            gui_start_ticks = _proc_start_ticks(os.getpid())
            if gui_start_ticks is None:
                push_log("err", "无法读取 GUI 进程身份（/proc starttime），拒绝启动受管进程")
                return False
            # 受管子进程必须经看门狗启动；没有降级为裸 launch 的路径。
            cmd = [sys.executable, GUARD_SCRIPT, str(os.getpid()),
                   str(gui_start_ticks), *cmd]
            # 读端另留一个 dup 给看门狗：GUI 退出后若有进程被保留（PCD 保存
            # 重试），保留组写日志时管道仍需有读者，否则首个 write 就 EPIPE/被
            # SIGPIPE——要保留的节点自己先死掉。
            out_r, out_w = os.pipe()
            os.set_inheritable(out_r, True)
            guard_env = dict(os.environ if env is None else env)
            guard_env["N01_GUARD_READ_FD"] = str(out_r)
            try:
                proc = subprocess.Popen(
                    cmd, stdout=out_w, stderr=subprocess.STDOUT,
                    start_new_session=True, env=guard_env, pass_fds=(out_r,))
            except Exception as exc:  # noqa: BLE001
                push_log("err", f"启动 {desc} 失败: {exc}")
                for _fd in (out_r, out_w):
                    try:
                        os.close(_fd)
                    except OSError:
                        pass
                return False
            finally:
                try:
                    os.close(out_w)
                except OSError:
                    pass
            # start_new_session=True → 子进程自成会话组，pgid == pid。
            # 记下 pgid：父进程先死时仍能清理整组残余。
            try:
                pgid = os.getpgid(proc.pid)
            except ProcessLookupError:
                pgid = proc.pid
            self.procs[key] = {"proc": proc, "desc": desc, "pgid": pgid}
        push_log("info", f"启动 {desc}: {' '.join(cmd)}")
        reader = os.fdopen(out_r, "r", encoding="utf-8", errors="replace")
        threading.Thread(target=self._reader, args=(key, proc, reader),
                         daemon=True).start()
        return True

    def _reader(self, key: str, proc: subprocess.Popen, reader) -> None:
        try:
            for line in reader:
                line = line.rstrip()
                if line:
                    STATE.log_q.put(f"[{time.strftime('%H:%M:%S')}] {key}| {line}")
        except Exception:  # noqa: BLE001
            pass
        try:
            reader.close()
        except OSError:
            pass
        rc = proc.poll()
        push_log("info" if rc == 0 else "warn", f"{key} 进程退出 rc={rc}")
        if key == "desk":
            was_mapping = self.desk_mode in (
                "mapping", "fast_lio_mapping", "fast_lio_realtime")
            self.desk_mode = ""
            if was_mapping and self.running("bag"):
                push_log("warn", "建图 desk 已退出；同步停止 bag 录制，避免孤儿录包")
                self.stop("bag")

    def start_bag(self) -> bool:
        """Start one ros2 bag recording for the mapping session."""
        if self.running("bag"):
            push_log("warn", "已有 bag 录制在运行，拒绝重复启动")
            return False
        try:
            os.makedirs(MAPPING_BAG_DIR, exist_ok=True)
        except OSError as exc:
            push_log("err", f"创建 bag 目录失败：{MAPPING_BAG_DIR}: {exc}")
            return False
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(MAPPING_BAG_DIR, f"bag_{stamp}")
        if os.path.exists(path):
            path = f"{path}_{os.getpid()}"
        cmd = ["ros2", "bag", "record", "-o", path, *MAPPING_BAG_TOPICS]
        ok = self._spawn("bag", cmd, f"bag 录制 {path}")
        if not ok:
            push_log("err", "建图 bag 录制启动失败；不会继续启动建图 desk")
            return False
        # Catch an immediate rosbag2 startup failure before starting the desk.
        time.sleep(0.4)
        with self.lock:
            item = self.procs.get("bag")
            early_exit = bool(item and item["proc"].poll() is not None)
        if early_exit:
            push_log("err", "bag 进程启动后立即退出；不会继续启动建图 desk")
            self.stop("bag")
            return False
        self.bag_path = path
        push_log("ok", f"建图 bag 录制已启动：{path}")
        push_log("info", "录制话题：" + " ".join(MAPPING_BAG_TOPICS))
        return True

    def deactivate_mapping(self, timeout: float = 60.0) -> tuple[bool, str]:
        """Ask mapping_lite_node to save the PCD, synchronously (UI thread blocks)."""
        cmd = ["ros2", "service", "call",
               f"{MAP_NODE}/deactivate_mapping", "std_srvs/srv/Trigger", "{}"]
        try:
            cp = subprocess.run(cmd, capture_output=True, text=True,
                                timeout=timeout, check=False,
                                env=os.environ.copy())
        except subprocess.TimeoutExpired:
            return False, f"deactivate_mapping 超时 {timeout:.0f}s"
        except Exception as exc:  # noqa: BLE001
            return False, f"deactivate_mapping 异常：{type(exc).__name__}: {exc}"
        text = (cp.stdout or "") + (cp.stderr or "")
        ok = cp.returncode == 0 and (
            "success=True" in text or "success: true" in text.lower())
        return ok, text

    def start_desk(self, mode: str, skip_rsp: bool = False) -> bool:
        # 节点图检查优先于本 GUI 自己的进程表，覆盖“外部终端已经启动”的情况。
        # 普查结果过期/不存在时拒绝：否则“图里有旧 desk”看不见，会叠出第二套。
        with STATE.lock:
            loc_up, map_up = STATE.loc_node_up, STATE.map_node_up
            census_at = STATE.census_at
        if not census_at or time.monotonic() - census_at > 15.0:
            push_log("err", "节点普查结果不可用或已过期，拒绝启动 desk"
                              "（无法确认前置链是否已在运行）")
            return False
        if mode == "loc" and map_up:
            push_log("err", "互斥阻止：mapping_lite_node 已在运行，不能同时启动定位")
            return False
        if mode == "mapping" and loc_up:
            push_log("err", "互斥阻止：localization_lite_node 已在运行，不能同时启动建图")
            return False
        with self.lock:
            old = self.procs.get("desk")
            if old and old["proc"].poll() is None:
                push_log("warn", f"已有 {old['desc']} 在运行，请先停止")
                return False
        # desk 前置链（互补滤波 / cloud_preprocessor / static TF / RSP）若已在图里，
        # 新 desk 会再起一套 → /imu/data、/livox/lidar/geo_filtered、/tf_static 双源。
        # 一律拦截：无论那是外部终端故意起的还是上次残留，再叠一套都只会制造
        # 双权威源；要起新的，先明确停掉旧的那一套（体检页单停 / 未托管残留检查）。
        with STATE.lock:
            rows = list(STATE.node_rows)
        fast_lio_residue = [r["name"] for r in rows
                            if r["name"] in FAST_LIO_NODES
                            and r.get("count", 0) > 0]
        if fast_lio_residue:
            push_log("err", f"拒绝启动甲方 desk：FAST-LIO2 后端仍在运行 "
                            f"{fast_lio_residue}；请从原启动入口停止后再切换")
            return False
        residue = [r["name"] for r in rows
                   if r["name"] in DESK_CHAIN_NODES and r.get("count", 0) > 0
                   # 已选择由地盘 RSP 提供 TF 时，它的 robot_state_publisher 是预期存在的
                   and not (skip_rsp and r["name"] == "/robot_state_publisher")]
        orphans = [r["name"] for r in rows
                   if r["name"] in DESK_CHAIN_NODES and "孤儿" in r.get("state", "")]
        if orphans:
            # 父 launch 已死 = 确定的残留，再起一套必然双源，直接拒绝
            push_log("err", f"拒绝启动 desk：检测到孤儿前置链残留 {orphans}"
                              f"（ppid=1，父 launch 已死）。"
                              f"先到“节点体检”页查看进程详情，确认来源后回到原启动终端 Ctrl+C；"
                              f"不会自动终止未托管进程")
            return False
        if residue:
            rsp_only = (residue == ["/robot_state_publisher"] and not skip_rsp)
            hint = ("图里的 robot_state_publisher 是地盘 hive2_driver 的。"
                    "请勾选“不启动 SLAM 的 RSP”，或先停掉地盘 RSP 再起 SLAM 自带 RSP。"
                    if rsp_only else
                    "先核对进程归属，并从原启动终端正常停止后再启动 desk。")
            push_log("err", f"拒绝启动 desk：前置链节点已在位 {residue}，"
                              f"再起一套会让 /imu/data、/tf_static 出现双源。{hint}")
            return False
        launch = "localization_lite_desk.launch.py" if mode == "loc" else "mapping_lite_desk.launch.py"
        if mode == "loc":
            # One click starts desk + adapter + SCAN bridge so /body_pose and
            # /cloud (the navigation inputs) exist before navigation starts.
            # The helper sources ROS + vendor SLAM + this workspace; desk_guard
            # alone only has the vendor AMENT_PREFIX_PATH.
            cmd = ["bash", os.path.join(NAV_REPO, "scripts", "start_localization_stack.sh")]
        else:
            cmd = ["stdbuf", "-oL", "ros2", "launch",
                   "hivecore_slam3d_lite_console", launch]
        if os.path.isfile(STATIC_TF_PARAMS):
            cmd.append(f"static_tf_params_file:={STATIC_TF_PARAMS}")
            push_log("info", f"静态 TF 使用 N-01 参数：{STATIC_TF_PARAMS}")
        else:
            push_log("warn", f"缺少静态 TF 覆盖文件：{STATIC_TF_PARAMS}；"
                              "desk 将使用厂商默认 auto-calibration")
        params_file = LOCALIZATION_PARAMS if mode == "loc" else MAPPING_PARAMS
        if os.path.isfile(params_file):
            key = "localization_params_file" if mode == "loc" else "mapping_params_file"
            cmd.append(f"{key}:={params_file}")
            push_log("info", f"SLAM 参数使用统一 0.90 链：{params_file}")
        else:
            push_log("warn", f"缺少统一 SLAM 参数文件：{params_file}")
        if skip_rsp:
            # 甲方契约：传感器帧唯一定义在 quad_leg_description，
            # 由 hive2_driver 的 RSP 发布（z=0.90）；SLAM 自带的 3-link URDF 是 z=0.30。
            cmd.append("launch_robot_state_publisher:=false")
            push_log("info", "已关闭 desk 自带 RSP：base_footprint→base_link 由地盘 RSP 提供")
        desc = "定位 desk" if mode == "loc" else "建图 desk"
        if skip_rsp:
            desc += "（无自带 RSP）"
        if mode == "loc" and self.running("bag"):
            push_log("err", "建图 bag 仍在录制；请先结束建图，再启动定位")
            return False
        bag_started = False
        if mode == "mapping":
            if not self.start_bag():
                return False
            bag_started = True
        # Set the intent before spawn so an immediate desk exit is still seen as
        # a mapping session by _reader and tears down the bag.
        self.desk_mode = mode
        ok = self._spawn("desk", cmd, desc)
        if not ok:
            self.desk_mode = ""
            if bag_started:
                push_log("warn", "建图 desk 启动失败，停止同时启动的 bag 录制")
                self.stop("bag")
                self.bag_path = ""
        return ok

    def start_fast_lio(self, mode: str, saved_map_path: str,
                       fast_lio_map_path: str = DEFAULT_FAST_LIO_MAP) -> bool:
        """Start the repository-owned FAST-LIO2 backend through the same guard."""
        if mode not in ("loc", "mapping", "realtime"):
            push_log("err", f"未知 FAST-LIO2 模式：{mode}")
            return False
        if self.running("desk"):
            push_log("warn", "已有建图定位后端在运行，请先停止后再切换")
            return False
        with STATE.lock:
            names = {r["name"] for r in STATE.node_rows}
            census_at = STATE.census_at
        if not census_at or time.monotonic() - census_at > 15.0:
            push_log("err", "节点普查结果不可用或已过期，拒绝启动 FAST-LIO2")
            return False
        conflicts = sorted(names & {
            LOC_NODE, MAP_NODE, "/n01/fast_lio/mapping",
            "/n01_fast_lio_relocalizer", "/n01_fast_lio_pose_adapter",
            "/n01_localization_adapter", "/n01_live_scan_bridge",
            "/scan_planner_node",
        })
        if conflicts:
            push_log("err", f"拒绝启动 FAST-LIO2：发现其它建图定位后端 {conflicts}")
            return False
        map_out = os.path.abspath(os.path.expanduser(fast_lio_map_path))
        helper = os.path.join(NAV_REPO, "scripts", "start_fast_lio_gui_backend.sh")
        if not os.path.isfile(helper):
            push_log("err", f"FAST-LIO2 GUI 启动器缺失：{helper}")
            return False
        if mode == "loc":
            saved = os.path.abspath(os.path.expanduser(saved_map_path))
            if not os.path.isfile(saved):
                push_log("err", f"FAST-LIO2 定位参考 PCD 不存在：{saved}")
                return False
            cmd = ["bash", helper, "localization",
                   f"saved_map_path:={saved}", f"fastlio_map_path:={map_out}"]
            desc = "FAST-LIO2 定位 + 适配器 + 导航桥接"
        elif mode == "mapping":
            cmd = ["bash", helper, "mapping", f"map_file_path:={map_out}",
                   "rviz:=false"]
            desc = "FAST-LIO2 建图"
            if not self.start_bag():
                return False
        else:
            cmd = ["bash", helper, "realtime", f"map_file_path:={map_out}"]
            desc = "FAST-LIO2 实时建图 + 定位适配 + 导航桥接"
            if not self.start_bag():
                return False
        self.desk_mode = f"fast_lio_{mode}"
        ok = self._spawn("desk", cmd, desc)
        if not ok:
            self.desk_mode = ""
            if mode in ("mapping", "realtime"):
                self.stop("bag")
        return ok

    def start_rviz(self, mode: str) -> bool:
        script = (f"{ROSK}/run_vis_world_localization.sh" if mode == "loc"
                  else f"{ROSK}/run_vis_world_mapping.sh")
        if not os.path.isfile(script):
            push_log("err", f"找不到 RViz 启动脚本: {script}")
            return False
        return self._spawn("rviz", ["bash", script],
                           "RViz 定位视图" if mode == "loc" else "RViz 建图视图")

    def start_fast_lio_rviz(self, realtime: bool = False) -> bool:
        config = os.path.join(NAV_REPO, "src", "n01_fast_lio", "rviz",
                              "fast_lio_realtime.rviz" if realtime else
                              "fast_lio.rviz")
        if not os.path.isfile(config):
            push_log("err", f"找不到 FAST-LIO2 RViz 配置：{config}")
            return False
        return self._spawn(
            "rviz", ["rviz2", "-d", config],
            "FAST-LIO2 实时建图 RViz" if realtime else "FAST-LIO2 重定位 RViz")

    def start_slow_motion(self, speed: float = 0.05) -> bool:
        """Start the opt-in very-slow /cmd_vel path executor.

        path_source=auto  + control_mode=auto：重规划窗口内沿用最后验证前缀
        (grace 1s)，然后回退新鲜全局路径；4WIS 几何校验失败时自动转差速
        缩放的 Twist，避免低速大曲率触发 ±90° 转向界死锁。
        """
        speed = max(0.01, min(0.08, float(speed)))
        cmd = ["bash", os.path.join(NAV_REPO, "scripts", "start_slow_motion.sh"),
               "--ros-args", "-p", "enabled:=true",
               "-p", f"max_speed_mps:={speed:.3f}",
               "-p", "max_yaw_rate_radps:=0.12",
               "-p", "max_accel_mps2:=0.04",
               "-p", "path_source:=auto",
               "-p", "control_mode:=auto"]
        return self._spawn("slow_motion", cmd,
                           f"慢速运行（max {speed:.3f} m/s，/cmd_vel，auto/auto）")

    def start_nav_rviz(self) -> bool:
        if not os.path.isfile(NAV_RVIZ_CONFIG):
            push_log("err", f"找不到导航 RViz 配置: {NAV_RVIZ_CONFIG}")
            return False
        return self._spawn("nav_rviz", ["rviz2", "-d", NAV_RVIZ_CONFIG],
                           "导航 RViz")

    def start_bridge(self) -> bool:
        return self._spawn("foxglove", ["ros2", "launch", "foxglove_bridge",
                                         "foxglove_bridge_launch.xml"],
                           "Foxglove Bridge")

    @staticmethod
    def _group_members(pgid: int) -> list[int]:
        members = []
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            pid = int(entry)
            try:
                with open(f"/proc/{pid}/stat", "rb") as fh:
                    raw = fh.read().decode("ascii", "replace")
                fields = raw.rsplit(") ", 1)[1].split()
                state, process_group = fields[0], int(fields[2])
                if process_group == pgid and state != "Z":
                    members.append(pid)
            except (OSError, IndexError, ValueError):
                continue
        return members

    @classmethod
    def _group_alive(cls, pgid: int) -> bool:
        return bool(cls._group_members(pgid))

    @staticmethod
    def _signal_group(pgid: int, sig) -> None:
        # 绝不向本 GUI 自己的进程组发信号
        try:
            if pgid == os.getpgid(0):
                return
        except ProcessLookupError:
            return
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            pass

    def stop(self, key: str) -> None:
        """停止 GUI 自己拉起的子进程 —— 保证整组清空，而不是只看父进程。

        旧实现的残留路径：launch 父进程先退出时 `return` 直接跳过清理，
        组内子节点（例如互补滤波、cloud_preprocessor）会继续跑，下次再起 desk
        就叠出第二套 /imu/data、/livox/lidar/geo_filtered、/tf_static。
        """
        with self.lock:
            item = self.procs.get(key)
            if not item:
                push_log("info", f"{key} 未在运行")
                return
            proc = item["proc"]
            pgid = item.get("pgid") or proc.pid
            self.procs.pop(key, None)
        desc = item["desc"]
        if proc.poll() is None:
            push_log("info", f"发送 SIGINT 停止 {desc}（进程组 {pgid}）…")
            self._signal_group(pgid, signal.SIGINT)
            try:
                proc.wait(timeout=12)   # 看门狗内部最多等 8s 再 SIGKILL
            except subprocess.TimeoutExpired:
                push_log("err", f"{desc} 12s 未退出，升级 SIGKILL")
        else:
            push_log("warn", f"{desc} 父进程已退出（rc={proc.returncode}），"
                              f"检查进程组 {pgid} 是否有残留子节点")
        # 无论父进程在不在，都要确认整组确实空了
        if self._group_alive(pgid):
            push_log("warn", f"{desc}: 进程组仍有活动成员，升级 SIGKILL")
            self._signal_group(pgid, signal.SIGKILL)
            deadline = time.monotonic() + 2.0
            while self._group_alive(pgid) and time.monotonic() < deadline:
                time.sleep(0.1)
        if self._group_alive(pgid):
            push_log("err", f"{desc}: 进程组 {pgid} 仍有残留，请看“节点体检”页按节点停止")
        else:
            push_log("ok", f"{desc}: 进程组 {pgid} 已清空")

    def stop_all(self) -> None:
        # Stop actuation first, then finalize rosbag2 before its backend exits.
        # All targets are GUI-owned process groups.
        for key in ("slow_motion", "bag", "desk", "rviz", "nav_rviz", "foxglove"):
            self.stop(key)


PROCS = ProcManager()


# ---------------------------------------------------------------------------
# N-01 导航生命周期：委托给导航仓库自己的进程组管理器
# ---------------------------------------------------------------------------
NAV_REPO = os.path.abspath(os.path.expanduser(
    os.environ.get("N01_NAV_REPO", str(REPO_ROOT))))
NAV_ENTRY = os.path.join(NAV_REPO, "scripts", "n01_navigation.sh")
# Vendor static TF publisher: use the N-01 override so lidar_link->imu_link->
# livox_frame is published from YAML at desk startup, without waiting for the
# optional live IMU calibration stream.
STATIC_TF_PARAMS = os.path.join(NAV_REPO, "config", "n01_static_tf_params.yaml")
GUI_CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config")
LOCALIZATION_PARAMS = os.path.join(GUI_CONFIG_DIR, "n01_localization_lite.yaml")
MAPPING_PARAMS = os.path.join(GUI_CONFIG_DIR, "n01_mapping_lite.yaml")
# Keep the page default aligned with the active custom localization YAML.
DEFAULT_MAP_PATH = "/home/demo/lyy/slam_data/maps/mapping_lite_20261109_140536_087.pcd"
NAV_RVIZ_CONFIG = os.path.join(NAV_REPO, "rviz", "n01_full_pipeline.rviz")
_NAV_ACTION_LOCK = threading.Lock()

def run_navigation_lifecycle(action: str, timeout: float | None = None,
                              env_extra: dict | None = None) -> tuple[bool, str]:
    """通过统一 helper 调用导航仓库生命周期入口；不按 ROS 节点名全局 kill。"""
    helper = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "tools", "navigation_lifecycle.py")
    if not os.path.isfile(helper):
        return False, f"导航生命周期管理器缺失：{helper}"
    if timeout is None:
        # helper 的 start 是 stop(45)+status(20)+start(90) 三段串行，常规最坏 155 s，
        # 170 s 覆盖它；若某段真挂住，helper 自己的兜底再 +100 s（极端 255 s），
        # 此时外层杀掉 helper 后由下方 TimeoutExpired 分支再发一次全新 stop——
        # 与 stop 相同的二级升级，不会把 launch 留成孤儿。
        # stop：helper 自己的兜底路径最坏 45+100=145 s，若会话真挂住，
        # 外层在 75 s 杀掉 helper 、再由下方 TimeoutExpired 分支对同一仓库再发
        # 一次全新 stop（PID 文件幂等）即是设计内的二级升级，最终能停干净；
        # 所以 stop 预算刻意低于 145 s——让升级尽早发生，而不是把 GUI 卡到 145 s。
        timeout = {"start": 170.0, "stop": 75.0, "status": 30.0}.get(action, 60.0)
    if not _NAV_ACTION_LOCK.acquire(blocking=False):
        return False, "已有导航生命周期操作正在执行，请等待完成。"
    try:
        env = os.environ.copy()
        if env_extra:
            env.update({str(k): str(v) for k, v in env_extra.items()})
        cp = subprocess.run([sys.executable, helper, action, "--repo", NAV_REPO],
                            cwd=os.path.dirname(os.path.abspath(__file__)),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, timeout=timeout, check=False,
                            env=env)
        output = cp.stdout[-24000:] + f"\nexit={cp.returncode}"
        try:
            log_dir = Path(os.environ.get("N01_RUNTIME_ROOT", str(Path(NAV_REPO).parent / "n01_runtime"))) / "logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"gui_navigation_{action}_{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns()}.log"
            log_path.write_text(cp.stdout + f"\nexit={cp.returncode}\n", encoding="utf-8")
            output += f"\n完整日志：{log_path}"
        except OSError as exc:
            output += f"\n日志落盘失败：{exc}"
        return cp.returncode == 0, output
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or ""
        if isinstance(output, bytes):
            output = output.decode("utf-8", "replace")
        # 超时只杀掉了 helper 这个直接子进程；它下面可能在跑 ros2 launch。
        # 用仓库自己的 stop（经 PID 文件清理 launch 进程组）做兜底，
        # 而不是 killpg——nohup setsid 的 launch 在独立会话里，killpg 打不到。
        note = ""
        try:
            stop_cp = subprocess.run(
                [sys.executable, helper, "stop", "--repo", NAV_REPO],
                cwd=os.path.dirname(os.path.abspath(__file__)),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=100.0, check=False, env=env)
            note = f"\n超时后已尝试 stop 清理 (exit={stop_cp.returncode})"
        except subprocess.TimeoutExpired:
            note = ("\n超时后 stop 清理本身超时；请用诊断页“查看停止计划/执行停止”"
                    "确认无残留后再重试")
        except Exception as stop_exc:  # noqa: BLE001
            note = f"\n超时后 stop 清理异常：{type(stop_exc).__name__}: {stop_exc}"
        return False, (str(output) + note
                       + f"\n导航 {action} 超时；请在终端执行 "
                         f"python3 {helper} status --repo {NAV_REPO}")
    except Exception as exc:  # noqa: BLE001
        return False, f"导航 {action} 异常：{type(exc).__name__}: {exc}"
    finally:
        _NAV_ACTION_LOCK.release()


class GlobalLifecycleManager:
    """Single orchestration point for GUI-owned SLAM visualization and N-01 navigation."""

    def __init__(self, proc_manager: ProcManager) -> None:
        self.proc_manager = proc_manager
        self._lock = threading.RLock()
        self._navigation_started_by_gui = False

    def navigation(self, action: str, timeout: float | None = None,
                   env_extra: dict | None = None) -> tuple[bool, str]:
        ok, output = run_navigation_lifecycle(action, timeout=timeout,
                                              env_extra=env_extra)
        with self._lock:
            if action == "start" and ok:
                self._navigation_started_by_gui = True
            elif action == "stop" and ok:
                self._navigation_started_by_gui = False
        return ok, output

    def stop_all(self) -> tuple[bool, str]:
        # Explicit operator action: stop N-01 through its own manager, then GUI-owned SLAM desk/viz.
        ok, output = self.navigation("stop", timeout=60.0)
        self.proc_manager.stop_all()
        return ok, output

    def shutdown(self) -> tuple[bool, str]:
        # GUI shutdown only stops navigation if this GUI started it. An independently
        # launched navigation stack is left untouched unless the operator explicitly asks.
        with self._lock:
            owned_navigation = self._navigation_started_by_gui
        if owned_navigation:
            ok, output = self.navigation("stop", timeout=75.0)
        else:
            ok, output = True, "当前 GUI 未启动导航；独立导航实例保持不变。"
        self.proc_manager.stop_all()
        return ok, output

    def shutdown_navigation_only(self) -> tuple[bool, str]:
        """退出清理时只停导航栈，不动 desk/bag（PCD 保存失败时要保留写盘）。"""
        with self._lock:
            owned_navigation = self._navigation_started_by_gui
        if not owned_navigation:
            return True, "当前 GUI 未启动导航；独立导航实例保持不变。"
        return self.navigation("stop", timeout=75.0)


LIFECYCLE = GlobalLifecycleManager(PROCS)

# 兜底：即使 nicegui 的 on_shutdown 没跑到（解释器正常收尾），
# 也在这里把 GUI 拉起的 desk / RViz / Foxglove 整组停掉。
# 例外：退出时 deactivate_mapping 保存 PCD 失败/超时，服务端可能仍在写盘；
# 此时 desk 与 bag 必须保留（否则整张地图连同录包一起丢），由 _shutdown_app
# 置位 _PRESERVE_SHUTDOWN_SAVE_FAIL 来禁止本次兜底清理。
_PRESERVE_SHUTDOWN_SAVE_FAIL = False


def _fallback_stop_all() -> None:
    if _PRESERVE_SHUTDOWN_SAVE_FAIL:
        push_log("warn", "建图 PCD 保存失败：desk/bag 已保留。请回持久终端执行 "
                         "`ros2 service call /mapping_lite_node/deactivate_mapping "
                         "std_srvs/srv/Trigger \"{}\"` 重试保存，确认 PCD 大小合理后"
                         "再手动停止 desk 与 bag")
        return
    PROCS.stop_all()


atexit.register(_fallback_stop_all)


# ---------------------------------------------------------------------------
# 图表数据
# ---------------------------------------------------------------------------
_chart_cache: dict = {"key": None, "pts": [], "option": None}


def _view_points() -> tuple[list, str]:
    """按视图选择取点，返回 ([ [x,y], ... ], 描述)。"""
    src = UI["src"].value if UI.get("src") else "定位地图"
    want_loc = src in ("定位地图", "两者")
    want_cloud = src in ("建图点云", "两者")
    with STATE.lock:
        loc_t, cloud_t = STATE.loc_pts, STATE.cloud_pts
        loc_seq, cloud_seq = STATE.loc_seq, STATE.cloud_seq
        pose = STATE.pose
        trail = list(STATE.trail)

    key = (want_loc, want_cloud, loc_seq if want_loc else 0,
           cloud_seq if want_cloud else 0)
    if key == _chart_cache["key"]:
        return _chart_cache["pts"], src

    arr = None
    if want_loc and want_cloud:
        parts = [a for a in (loc_t, cloud_t) if a is not None and len(a)]
        arr = np.vstack(parts) if parts else None
    elif want_loc:
        arr = loc_t
    elif want_cloud:
        arr = cloud_t
    if arr is not None and len(arr):
        n = min(len(arr), MAX_PTS_VIEW)
        if n < len(arr):
            idx = np.linspace(0, len(arr) - 1, n).astype(int)
            arr = arr[idx]
        pts = arr[:, :2].round(2).tolist()
    else:
        pts = []
    _chart_cache.update(key=key, pts=pts)
    return pts, src


def _chart_option(pts: list, trail: list, pose) -> dict:
    if pts:
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
        half = max((max(xs) - min(xs)) / 2.0, (max(ys) - min(ys)) / 2.0, 8.0)
    elif pose:
        cx, cy, half = pose[0], pose[1], 8.0
    else:
        cx, cy, half = 0.0, 0.0, 8.0

    mode = UI["range"].value if UI.get("range") else "自动"
    fixed = {"20m": 10.0, "40m": 20.0, "100m": 50.0}.get(mode)
    if fixed:
        half = fixed
        if pose:
            cx, cy = pose[0], pose[1]
    half = min(half, 400.0)

    arrow = []
    if pose:
        x, y, yaw = pose
        arrow = [[x - 0.45 * math.cos(yaw), y - 0.45 * math.sin(yaw)],
                 [x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw)]]

    series = [
        {"name": "点云", "type": "scatter", "data": pts, "symbolSize": 2.2,
         "itemStyle": {"color": "#5598e7", "opacity": 0.55},
         "large": True, "largeThreshold": 2000, "progressive": 3000},
        {"name": "轨迹", "type": "line", "data": [[p[0], p[1]] for p in trail],
         "symbol": "none", "lineStyle": {"width": 1.4, "color": TRAIL_C, "opacity": 0.6}},
        {"name": "机身", "type": "line", "data": arrow, "symbol": "none",
         "lineStyle": {"width": 3, "color": ACCENT}},
        {"name": "位置", "type": "scatter",
         "data": [[pose[0], pose[1]]] if pose else [],
         "symbolSize": 7, "itemStyle": {"color": ACCENT}, "z": 10},
    ]
    return {
        "backgroundColor": SURFACE,
        "animation": False,
        "legend": {"top": 2, "textStyle": {"color": TEXT}, "itemWidth": 12, "itemHeight": 8},
        "grid": {"left": 58, "right": 18, "top": 34, "bottom": 42},
        "xAxis": {"type": "value", "name": "x [m]", "min": cx - half, "max": cx + half,
                  "axisLabel": {"color": MUTED}, "nameTextStyle": {"color": MUTED},
                  "axisLine": {"lineStyle": {"color": MUTED}},
                  "splitLine": {"lineStyle": {"color": GRID}}},
        "yAxis": {"type": "value", "name": "y [m]", "min": cy - half, "max": cy + half,
                  "axisLabel": {"color": MUTED}, "nameTextStyle": {"color": MUTED},
                  "axisLine": {"lineStyle": {"color": MUTED}},
                  "splitLine": {"lineStyle": {"color": GRID}}},
        "series": series,
    }


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------
from nicegui import app, ui  # noqa: E402

UI: dict = {}


def _lbl(text: str, cls: str = "text-sm text-slate-300"):
    return ui.label(text).classes(cls)


def _kv(key: str, initial: str = "—"):
    """在当前 grid 里放一格“键 + 值”，返回值标签。"""
    _lbl(key, "text-xs text-slate-500")
    return ui.label(initial).classes("font-semibold text-sm text-slate-100 break-words")


_SET_CACHE: dict[int, tuple[str, str]] = {}


def _set(el, text: str, color: str | None = None) -> None:
    """只有文本/颜色实际变化时才写 DOM。

    多条 0.5 s 刷新路径每 tick 都会调用 _set；内容没变时跳过 set_text +
    style(replace=...)，避免 NiceGUI 每个周期都替换相同文本/样式（闪烁、CPU）。
    """
    c = color or TEXT
    key = id(el)
    cached = _SET_CACHE.get(key)
    if cached == (text, c):
        return
    if len(_SET_CACHE) > 4096:
        _SET_CACHE.clear()
    _SET_CACHE[key] = (text, c)
    el.set_text(text)
    el.style(replace=f"color:{c}")


def _session_cn(s: str) -> str:
    return {"idle": "空闲", "relocalization": "重定位", "tracking": "追踪",
            "mapping": "建图中"}.get((s or "").lower(), s or "—")


def _session_color(s: str) -> str:
    return {"idle": COL_IDLE, "relocalization": COL_RELOC, "tracking": COL_TRACK,
            "mapping": COL_TRACK}.get((s or "").lower(), TEXT)


def _card(title: str):
    card = ui.card().classes("w-full q-pa-md gap-2")
    with card:
        _lbl(title, "text-base font-semibold text-slate-100")
    return card


def build_status_tab() -> None:
    with ui.card().classes("w-full q-pa-md gap-1"):
        _lbl("ROS 连接", "text-base font-semibold text-slate-100")
        UI["ros_reason"] = ui.label("ROS 工作线程正在启动…").classes(
            "w-full text-xs font-mono break-all")
    with ui.card().classes("w-full q-pa-md gap-1"):
        _lbl("节点普查（重名 / 多发布者）", "text-base font-semibold text-slate-100")
        UI["dup_banner"] = ui.label("等待 ROS 图…").classes(
            "w-full text-xs font-mono break-all")
    with ui.row().classes("w-full gap-4 items-start no-wrap"):
        # ---- 左列状态 ----
        with ui.column().classes("w-80 shrink-0 gap-3"):
            with _card("定位 /localization_lite_node"):
                with ui.grid(columns=2).classes("gap-x-4 gap-y-1"):
                    UI["loc_session"] = _kv("会话")
                    UI["loc_map_loaded"] = _kv("地图已加载")
                    UI["loc_reloc"] = _kv("重定位收敛")
                    UI["loc_fitness"] = _kv("fitness")
                    UI["loc_pose"] = _kv("位姿 x / y / yaw")
                    UI["loc_pose_age"] = _kv("位姿新鲜度")
                    UI["loc_map_pts"] = _kv("定位地图点数")
                ui.separator()
                _lbl("map_path", "text-xs text-slate-400")
                UI["loc_map_path"] = ui.label("—").classes("text-xs break-all text-slate-300")

            with _card("建图 /mapping_lite_node"):
                with ui.grid(columns=2).classes("gap-x-4 gap-y-1"):
                    UI["map_session"] = _kv("会话")
                    UI["map_cloud_pts"] = _kv("建图点云点数")
                ui.separator()
                _lbl("save_dir", "text-xs text-slate-400")
                UI["map_save_dir"] = ui.label("—").classes("text-xs break-all text-slate-300")

            with _card("上游输入"):
                with ui.grid(columns=2).classes("gap-x-4 gap-y-1"):
                    UI["in_cloud_age"] = _kv("/livox/lidar/geo_filtered")
                _lbl("输入云是 SLAM 的源头；无数据时定位/建图都不会有输出", "text-xs text-slate-400")

        # ---- 右侧地图 ----
        with ui.column().classes("flex-grow min-w-0 gap-2"):
            with ui.card().classes("w-full q-pa-md gap-2"):
                with ui.row().classes("items-center gap-3"):
                    _lbl("地图 / 位姿", "text-base font-semibold text-slate-100")
                    UI["src"] = ui.radio(["定位地图", "建图点云", "两者"], value="定位地图",
                                        on_change=lambda _: draw_chart(force=True)) \
                        .props("inline dense")
                    UI["range"] = ui.select(["自动", "20m", "40m", "100m"], value="自动",
                                           label="视野", on_change=lambda _: draw_chart(force=True)) \
                        .classes("w-28").props("dense")
                    UI["trail_on"] = ui.checkbox("轨迹", value=True)
                    UI["chart_info"] = ui.label("等待点云…").classes("text-xs text-slate-400")
                UI["chart"] = ui.echart(_chart_option([], [], None)).classes("w-full h-[580px]")


def build_ctrl_tab() -> None:
    with ui.row().classes("w-full gap-4 items-start"):
        # 定位控制
        with ui.column().classes("flex-1 min-w-72 gap-3"):
            with _card("定位控制"):
                for name, desc in LOC_SRV_DESC.items():
                    with ui.row().classes("items-center gap-3 w-full no-wrap"):
                        ui.button(name, on_click=functools.partial(srv_click, name)) \
                            .props("outline color=primary").classes("w-64 text-left")
                        _lbl(desc, "text-xs text-slate-400")
            with _card("建图控制"):
                for name, desc in MAP_SRV_DESC.items():
                    with ui.row().classes("items-center gap-3 w-full no-wrap"):
                        ui.button(name, on_click=functools.partial(srv_click, name)) \
                            .props("outline color=secondary").classes("w-64 text-left")
                        _lbl(desc, "text-xs text-slate-400")

        # 初始位姿 + 节点管理
        with ui.column().classes("flex-1 min-w-72 gap-3"):
            with _card("初始位姿 /initialpose"):
                _lbl("仅重定位会话接受；追踪 / 空闲会被忽略。", "text-xs text-slate-400")
                with ui.row().classes("gap-2 items-end"):
                    UI["ip_x"] = ui.number("x [m]", value=0.0, format="%.2f").classes("w-24").props("dense")
                    UI["ip_y"] = ui.number("y [m]", value=0.0, format="%.2f").classes("w-24").props("dense")
                    UI["ip_yaw"] = ui.number("yaw [deg]", value=0.0, format="%.1f").classes("w-24").props("dense")
                with ui.row().classes("gap-2 items-center"):
                    UI["ip_frame"] = ui.select(["map", "base_footprint"], value="map",
                                              label="frame").classes("w-48").props("dense")
                    ui.button("发布", on_click=send_initialpose).props("color=primary")
                UI["ip_result"] = ui.label("").classes("text-xs text-slate-400")

            with _card("节点管理（可选项，默认关闭）"):
                UI["mg_enable"] = ui.checkbox(
                    "启用节点管理：由本 GUI 启动 / 停止 lite desk 节点", value=False)
                with ui.row().classes("gap-2 items-center"):
                    UI["mg_mode"] = ui.select(["定位 desk", "建图 desk"], value="定位 desk") \
                        .classes("w-36").props("dense")
                    ui.button("启动", on_click=start_desk).props("color=positive")
                    ui.button("停止", on_click=stop_desk).props("color=negative")
                UI["mg_skip_rsp"] = ui.checkbox(
                    "不启动 SLAM 的 RSP（由地盘 hive2_driver 提供 "
                    "base_footprint→base_link→lidar_link）", value=True)
                _lbl("勾选后传 launch_robot_state_publisher:=false。"
                     "desk 自带的 3-link URDF 是 z=0.30，地盘的整机 URDF 是 z=0.90；"
                     "两者同时发同一 joint 会交替抖动。仅当地盘 RSP 已在运行时才可勾选。",
                     "text-xs text-slate-400")
                _lbl("命令：ros2 launch hivecore_slam3d_lite_console "
                     "{localization|mapping}_lite_desk.launch.py", "text-xs text-slate-400")
                _lbl("desk 会同时起 IMU 互补滤波 / RSP / cloud_preprocessor，"
                     "若这些已在跑请勿重复启动。", "text-xs text-amber-400")
                ui.separator()
                _lbl("定位与建图互斥：两者都订阅同一输入云、都发布 /robot_pose 与 "
                     "map→base_footprint TF；同一时刻只能运行一个。", "text-xs text-amber-400")
                with ui.grid(columns=2).classes("gap-x-4 gap-y-1"):
                    UI["proc_nodes"] = _kv("节点在位")
                    UI["proc_procs"] = _kv("本 GUI 进程")


def build_lifecycle_tab() -> None:
    with ui.column().classes("w-full gap-3"):
        with _card("统一生命周期总览"):
            _lbl("SLAM 定位与建图互斥；导航仅在定位 tracking、地图已加载且位姿持续更新时允许启动。",
                 "text-sm text-amber-300")
            with ui.grid(columns=3).classes("w-full gap-3"):
                UI["life_loc"] = _kv("Lite 定位")
                UI["life_map"] = _kv("Lite 建图")
                UI["life_nav"] = _kv("N-01 导航")
            with ui.row().classes("w-full items-center gap-2"):
                ui.button("刷新状态", on_click=lambda: navigation_action("status")) \
                    .props("outline color=primary")
                ui.button("停止全部 GUI 托管模块", on_click=stop_managed_all) \
                    .props("color=negative")
        with ui.row().classes("w-full items-start gap-4"):
            with ui.column().classes("flex-1 gap-3"):
                with _card("SLAM 生命周期"):
                    _lbl("启动前执行节点普查。发现定位/建图或 desk 前置链残留时拒绝叠加；不会误杀 Livox、IMU、底盘或外部 SLAM。",
                         "text-xs text-slate-400")
                    with ui.row().classes("items-center gap-2"):
                        ui.button("启动定位", on_click=lambda: lifecycle_start_slam("loc")) \
                            .props("color=positive")
                        ui.button("启动建图 + 录包", on_click=lambda: lifecycle_start_slam("mapping")) \
                            .props("color=positive")
                        ui.button("结束建图 / 停止 desk", on_click=stop_desk) \
                            .props("color=negative")
                    UI["life_slam_note"] = ui.label("SLAM desk 由本 GUI 的独立进程组管理。") \
                        .classes("text-xs text-slate-400")
            with ui.column().classes("flex-1 gap-3"):
                with _card("N-01 导航生命周期"):
                    _lbl(f"导航仓库：{NAV_REPO}", "text-xs font-mono text-slate-400 break-all")
                    _lbl("启动前先停止上一套受管导航进程组，再执行状态复查。只清理 N-01 导航自有进程，不会全局 kill ROS 节点。",
                         "text-xs text-slate-400")
                    with ui.row().classes("items-center gap-2"):
                        ui.button("启动导航", on_click=lambda: navigation_action("start")) \
                            .props("color=positive")
                        ui.button("停止导航", on_click=stop_navigation_module) \
                            .props("color=negative")
                        ui.button("导航状态", on_click=lambda: navigation_action("status")) \
                            .props("outline color=primary")
                    UI["nav_result"] = ui.label("尚未执行导航生命周期命令。") \
                        .classes("w-full text-xs font-mono whitespace-pre-wrap break-all")
        with _card("安全边界"):
            ui.markdown(
                "- **允许自动清理：** 当前 GUI 登记的 SLAM desk / RViz / Foxglove 进程组；"
                "N-01 导航仓库生命周期入口明确管理的导航进程组。\n"
                "- **禁止自动清理：** Livox、IMU、底盘驱动、腿控、控制器、外部独立 SLAM、"
                "外部定位适配器。\n"
                "- 如果发现无法证明归属的 ROS 残留，系统会阻止重复启动并报告 PID/节点，"
                "而不是对所有 ROS 进程执行 killall。"
            ).classes("text-sm")


async def lifecycle_start_slam(mode: str) -> None:
    # 统一生命周期页是显式的节点管理入口，因此在此处启用原有安全检查。
    UI["mg_enable"].value = True
    UI["mg_mode"].value = "定位 desk" if mode == "loc" else "建图 desk"
    backend = (UI["slam_backend"].value
               if UI.get("slam_backend") else BACKEND_VENDOR)
    if backend == BACKEND_FAST_LIO:
        fast_mode = (UI["fast_lio_mode"].value
                     if UI.get("fast_lio_mode") else FAST_MODE_OFFLINE)
        realtime = fast_mode == FAST_MODE_REALTIME
        saved_map = str(UI["map_path"].value or "").strip()
        fast_map = str(UI["fast_lio_map"].value or DEFAULT_FAST_LIO_MAP).strip()
        if realtime:
            mode = "realtime"
            fast_map = str(UI["fast_lio_map"].value or DEFAULT_FAST_LIO_MAP).strip()
            with STATE.lock:
                STATE.loc["map_path"] = "(FAST-LIO2 实时地图)"
        elif mode == "mapping":
            # The compact GUI treats this field as an output PCD for FAST-LIO2,
            # while the vendor backend keeps its original save-directory meaning.
            if not fast_map.lower().endswith(".pcd"):
                fast_map = DEFAULT_FAST_LIO_MAP
        else:
            fast_map = DEFAULT_FAST_LIO_MAP
            with STATE.lock:
                STATE.loc["map_path"] = saved_map
        label = ("实时建图定位 + bag" if mode == "realtime" else
                 ("定位" if mode == "loc" else "离线建图 + bag"))
        detail = (f"参考地图：{saved_map}\n" if mode == "loc" else "")
        if not await _confirm(
                f"将启动 {BACKEND_FAST_LIO} · {label}。\n{detail}"
                "输出经 n01_fast_lio 适配到统一定位/导航接口；"
                "不会与甲方 SLAM 同时运行。\n\n确认？"):
            return
        ok = await asyncio.to_thread(
            PROCS.start_fast_lio, mode, saved_map, fast_map)
        ui.notify("FAST-LIO2 已启动" if ok else "FAST-LIO2 启动失败，见运行日志",
                  type="positive" if ok else "negative")
        return
    await start_desk()


async def start_selected_slam_task() -> None:
    task = UI["slam_task"].value if UI.get("slam_task") else "定位"
    await lifecycle_start_slam("mapping" if task == "建图" else "loc")


async def start_selected_rviz() -> None:
    task = UI["slam_task"].value if UI.get("slam_task") else "定位"
    await start_slam_rviz("mapping" if task == "建图" else "loc")


def _slow_notice(msg: str, sev: str = "warn") -> None:
    """慢速行驶结果写进 STATE，由 refresh_panels 统一显示（单一写者，避免 0.5 s 被覆盖）。"""
    with STATE.lock:
        STATE.slow_status = msg
    push_log(sev, msg)


async def start_slow_motion() -> None:
    with STATE.lock:
        loc = dict(STATE.loc)
    ready = _nav_ready_state()
    if str(loc.get("session", "")).lower() != "tracking":
        _slow_notice("拒绝慢速运行：定位尚未 tracking。", "err"); return
    body_age = STATE.age_of("body_pose")
    path_age = STATE.age_of("plan_path")
    # 轨迹被清空时 monitor 会发一条空 Path，recv() 会把 age 重置到约 0，
    # 只看 age 会把"已清空"误判为"新鲜"。必须同时要求存在活动路径。
    path_ok = bool(STATE.plan_active) and path_age is not None and path_age <= 2.0
    if body_age is None or body_age > 0.5 or not path_ok:
        _slow_notice(f"拒绝慢速运行：/body_pose age={body_age}, "
                     f"local path age={path_age}, 活动路径={STATE.plan_active}；"
                     f"需要新鲜输入。", "err"); return
    if ready is not True:
        _slow_notice("拒绝慢速运行：导航 runtime 尚未 READY。", "err"); return
    try:
        speed = float(UI["slow_speed"].value if UI.get("slow_speed") else 0.05)
    except (TypeError, ValueError):
        speed = 0.05
    speed = max(0.01, min(0.08, speed))
    ok = await asyncio.to_thread(PROCS.start_slow_motion, speed)
    _slow_notice(f"慢速运行启动，最大 {speed:.3f} m/s" if ok else "慢速运行启动失败；查看日志",
                 "ok" if ok else "err")


async def stop_slow_motion() -> None:
    await asyncio.to_thread(PROCS.stop, "slow_motion")
    _slow_notice("慢速运行已停止（已发零速）", "warn")


async def navigation_action(action: str) -> None:
    env_extra = None
    if action == "start":
        realtime = bool(
            PROCS.desk_mode == "fast_lio_realtime" and PROCS.running("desk"))
        with STATE.lock:
            loc = dict(STATE.loc)
        age = STATE.age_of("pose")
        if str(loc.get("session", "")).lower() != "tracking":
            msg = "拒绝启动导航：当前建图定位后端尚未处于 tracking。"
            UI["nav_result"].set_text(msg); push_log("err", msg); return
        if loc.get("map_loaded") is not True or loc.get("reloc_ok") is not True:
            msg = "拒绝启动导航：地图未加载或 reloc_ok 未确认。"
            UI["nav_result"].set_text(msg); push_log("err", msg); return
        if age is None or age > float(os.environ.get("N01_NAV_MAX_POSE_AGE", "2.0")):
            msg = f"拒绝启动导航：位姿过期（age={age}，阈值={os.environ.get('N01_NAV_MAX_POSE_AGE', '2.0')}s）。"
            UI["nav_result"].set_text(msg); push_log("err", msg); return
        body_age = STATE.age_of("body_pose")
        if body_age is None or body_age > float(os.environ.get("N01_NAV_MAX_BODY_AGE", "1.0")):
            msg = ("拒绝启动导航：/body_pose 未就绪或已过期"
                   f"（age={body_age}）。请确认定位适配层/导航桥接已启动。")
            UI["nav_result"].set_text(msg); push_log("err", msg); return
        pcd = str(UI["nav_pcd"].value or "").strip() if UI.get("nav_pcd") else ""
        if not realtime and not pcd:
            # 与定位当前地图保持一致是首选；其次才用默认 PCD。显示层和门控不再脱节。
            with STATE.lock:
                pcd = str(STATE.loc.get("map_path", "") or "").strip()
        if not realtime and not pcd and os.path.isfile(DEFAULT_MAP_PATH):
            pcd = DEFAULT_MAP_PATH
        if not realtime and not pcd:
            msg = "拒绝启动导航：导航 PCD 为空（先加载定位地图，或点“同步 PCD”）。"
            UI["nav_result"].set_text(msg); push_log("err", msg); return
        if not realtime and UI.get("nav_pcd"):
            UI["nav_pcd"].value = pcd
        if not realtime and not os.path.isfile(pcd):
            msg = f"拒绝启动导航：导航 PCD 不存在：{pcd}"
            UI["nav_result"].set_text(msg); push_log("err", msg); return
        try:
            ground = float(UI["nav_ground_z"].value if UI.get("nav_ground_z") else 0.139)
        except (TypeError, ValueError):
            msg = "拒绝启动导航：ground_z_map_m 无效。"
            UI["nav_result"].set_text(msg); push_log("err", msg); return
        env_extra = {
            "N01_NAV_GROUND_Z": f"{ground:.3f}",
            "N01_NAV_MAP_MODE": "realtime" if realtime else "offline",
            # Headless GUI must not let launch start a dead rviz2. Use the
            # navigation module's standalone RViz button when DISPLAY exists.
            "N01_NAV_RVIZ": "false",
        }
        if not realtime:
            env_extra["N01_NAV_PCD"] = pcd
        map_desc = "FAST-LIO2 实时地图" if realtime else f"PCD={pcd}"
        push_log("info", f"导航启动参数：{map_desc} ground={ground:.3f} rviz=false")
    UI["nav_result"].set_text(f"正在执行导航 {action} …")
    ok, output = await asyncio.to_thread(LIFECYCLE.navigation, action,
                                         env_extra=env_extra)
    UI["nav_result"].set_text(("完成" if ok else "失败") + "\n" + output)
    errors = [line for line in output.splitlines()
              if any(token in line.lower() for token in ("error", "refusing", "timeout", "错误", "失败"))]
    summary = (f"导航 {action} 完成" if ok else
               f"导航 {action} 失败：" + ("；".join(errors[-2:]) or output[-600:]))
    if UI.get("nav_start_summary"):
        _set(UI["nav_start_summary"], summary, COL_OK if ok else COL_ERR)
    if not ok:
        push_log("err", summary)
    push_log("ok" if ok else "err", f"导航 {action} {'完成' if ok else '失败'}；详情见生命周期面板")


async def stop_managed_all() -> None:
    await asyncio.to_thread(PROCS.stop, "slow_motion")
    if not await _close_mapping_session():
        UI["nav_result"].set_text("建图保存失败：已保留 bag 和 desk，停止全部已取消。")
        ui.notify("建图保存失败，未执行停止全部", type="negative")
        return
    ok, output = await asyncio.to_thread(LIFECYCLE.stop_all)
    UI["nav_result"].set_text(("导航停止完成" if ok else "导航停止有告警") + "\n" + output)
    push_log("ok" if ok else "warn",
             "统一生命周期停止完成" if ok else "统一生命周期停止需检查：" + output[-1200:])
    # “停止全部”后所有 desk 都已关闭，隐藏模式复位，避免残留“建图 desk”。
    if UI.get("mg_mode"):
        UI["mg_mode"].value = "定位 desk"

def _nav_ready_state() -> bool | None:
    """navigation_ready 是 nav runtime 的 4 Hz 心跳；心跳停下后缓存值不能再当 READY。"""
    with STATE.lock:
        value = STATE.nav_ready
    age = STATE.age_of("nav_ready")
    if age is None or age > 2.0:
        return None
    return value


def refresh_lifecycle() -> None:
    if not UI.get("life_loc"):
        return
    with STATE.lock:
        loc = dict(STATE.loc); mapping = dict(STATE.mapping)
    loc_running = STATE.loc_node_up or (
        PROCS.desk_mode in ("fast_lio_loc", "fast_lio_realtime")
        and PROCS.running("desk"))
    map_running = STATE.map_node_up or (
        PROCS.desk_mode in ("fast_lio_mapping", "fast_lio_realtime")
        and PROCS.running("desk"))
    _set(UI["life_loc"], f"{'运行' if loc_running else '停止'} / {loc.get('session', '—')}",
         COL_OK if loc_running else COL_IDLE)
    _set(UI["life_map"], f"{'运行' if map_running else '停止'} / {mapping.get('session', '—')}",
         COL_OK if map_running else COL_IDLE)
    nav_ready = _nav_ready_state()
    _set(UI["life_nav"],
         "READY" if nav_ready is True else ("BLOCKED" if nav_ready is False else "—"),
         COL_OK if nav_ready is True else (COL_ERR if nav_ready is False else MUTED))


def build_ds_tab() -> None:
    with ui.row().classes("w-full gap-4 items-start"):
        with ui.column().classes("flex-1 min-w-0 gap-3"):
            with _card("订阅 / 发布话题"):
                cols = [
                    {"name": "topic", "label": "话题", "field": "topic", "align": "left"},
                    {"name": "type", "label": "类型", "field": "type", "align": "left"},
                    {"name": "dir", "label": "方向", "field": "dir"},
                    {"name": "age", "label": "最近数据", "field": "age"},
                    {"name": "note", "label": "用途", "field": "note", "align": "left"},
                ]
                rows = [{"topic": t, "type": ty, "dir": d, "note": n, "age": "—"}
                        for t, ty, d, n in TOPICS_DEF]
                UI["ds_table"] = ui.table(columns=cols, rows=rows, row_key="topic") \
                    .classes("w-full text-xs").props("dense wrap-cells")
            with _card("服务 std_srvs/srv/Trigger"):
                for svc, ty in SERVICES_DEF:
                    ui.label(f"{svc}   ({ty})").classes("text-xs font-mono text-slate-300")
        with ui.column().classes("w-96 shrink-0 gap-3"):
            with _card("环境与数据源铁律"):
                ament = os.environ.get("AMENT_PREFIX_PATH", "")
                new_tree = _has_rosk_prefix()
                ui.markdown(
                    f"**运行环境**\n\n"
                    f"必须先 `source {ROSK}/setup_runtime.sh`，"
                    f"否则无法加载 SLAM3D Lite 消息类型。"
                    f"每个新终端 source 一次即可；本 GUI 由 run 脚本启动时已自动 source，"
                    f"它拉起的 desk / RViz / bridge 子进程继承同一环境。\n\n"
                    f"当前：lite 消息 `{'可用' if _ENV_OK else '不可用'}`，"
                    f"新树在 `AMENT_PREFIX_PATH` 中 `{'是' if new_tree else '否'}`；"
                    f"`ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}`，"
                    f"`RMW={os.environ.get('RMW_IMPLEMENTATION', '默认')}`，"
                    f"`ROS_LOG_DIR={os.environ.get('ROS_LOG_DIR', '~/.ros/log')}`。\n\n"
                    f"若页面顶部显示“ROS 未连接”，页底日志会给出具体原因"
                    f"（消息类型不可用 / rclpy.init 失败）；最常见的是没 source "
                    f"或日志目录不可写（本 GUI 会自动改用自身目录下的 `log/ros`）。\n\n"
                    f"**互斥**\n\n"
                    f"- `localization_lite_node` 与 `mapping_lite_node` **不能同时运行**："
                    f"两者订阅同一 `/livox/lidar/geo_filtered`、都写 `/robot_pose` 与 "
                    f"`map→base_footprint` TF。\n"
                    f"- 建图产出的 PCD 是定位的输入，二者是串行工作流。\n\n"
                    f"**数据来源**\n\n"
                    f"- 话题 / TF / frame 全部来自甲方发布端，本 GUI 不生成。\n"
                    f"- 订阅统一用 BEST_EFFORT/VOLATILE，兼容发布端 QoS。\n\n"
                    f"**注意**\n\n"
                    f"- `deactivate_mapping` 会自动保存 PCD 并清空内存。\n"
                    f"- `enter_tracking` 前先确认 `reloc_ok=true` 且 fitness ≤ {FITNESS_THRESHOLD}。"
                )
            with _card("N-01 适配说明"):
                ui.markdown(
                    "导航链路使用仓库内 "
                    "`n01_live_navigation/config/slam3d_lite_adapter.yaml`"
                    "（`/loc_status` → `/n01/localization/status`）。"
                    "本 GUI 面向人工可视化操作，不替代该适配，也不写入其话题。"
                )


def _x_display_works(display: str):
    """Actually probe an X display; DISPLAY being set is not enough."""
    try:
        cp = subprocess.run(["xdpyinfo", "-display", display],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            timeout=2.0, check=False)
        return cp.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return None


def _display_info() -> tuple[str, bool, bool]:
    """返回 (说明, 是否可尝试启动, 是否使用 :0 兜底)。

    The GUI log showed rviz2 dying with xcb "could not connect to display"
    while DISPLAY was unset/invalid. Never spawn a native Qt window based only
    on the presence of /tmp/.X11-unix; probe it first.
    """
    disp = os.environ.get("DISPLAY", "").strip()
    if disp:
        works = _x_display_works(disp)
        if works is True:
            return f"DISPLAY={disp}（X 连接正常）", True, False
        if works is False:
            return (f"DISPLAY={disp} 但 X 服务器不可连接（检查 Xauthority/SSH 转发）；"
                    "RViz 不会启动"), False, False
        return (f"DISPLAY={disp}，但系统没有 xdpyinfo，无法验证 X；"
                "为避免起死进程，不启动 RViz"), False, False
    wayland = os.environ.get("WAYLAND_DISPLAY", "").strip()
    xdg = os.environ.get("XDG_RUNTIME_DIR", "").strip()
    if wayland and xdg and os.path.exists(os.path.join(xdg, wayland)):
        return f"WAYLAND_DISPLAY={wayland}", True, False
    if os.path.exists("/tmp/.X11-unix/X0"):
        works = _x_display_works(":0")
        if works is True:
            os.environ["DISPLAY"] = ":0"
            return "DISPLAY 未设置；检测到 X0 并已设置 DISPLAY=:0", True, True
        if works is False:
            return ("检测到 X0 socket，但连接 :0 失败（Xauthority/权限问题）；"
                    "RViz 不会启动"), False, False
    return ("无可用 X/Wayland 显示，RViz2 原生窗口无法弹出；"
            "请在桌面终端设置 DISPLAY/XAUTHORITY，或使用 Foxglove"), False, False


def build_viz_tab() -> None:
    with ui.row().classes("w-full gap-4 items-start"):
        with ui.column().classes("flex-1 min-w-72 gap-3"):
            with _card("RViz2（原生窗口）"):
                _lbl("使用 /opt/slam3d_release 自带的 vis_world_*.launch.py 与 .rviz 配置；"
                     "脚本内部会 source setup_runtime.sh。", "text-xs text-slate-400")
                with ui.row().classes("gap-2 items-center"):
                    UI["viz_mode"] = ui.select(["定位视图", "建图视图"], value="定位视图") \
                        .classes("w-36").props("dense")
                    ui.button("启动 RViz2", on_click=start_rviz).props("color=positive")
                    ui.button("停止", on_click=stop_rviz).props("color=negative")
                _text, _ok, _fallback = _display_info()
                _lbl(_text, "text-xs " + ("text-amber-400" if _fallback else "text-slate-400"))
                _lbl("rviz2 是 X11 原生窗口，不能嵌进网页；远程请用 X11 转发 / 桌面共享。",
                     "text-xs text-slate-400")
                UI["viz_rviz_status"] = ui.label("未运行").classes("text-xs text-slate-300")

            with _card("Foxglove Bridge（浏览器 3D）"):
                _lbl("启动后浏览器打开 Foxglove Studio（网页版或桌面版），"
                     "连接下方 WebSocket 地址即可看 3D 点云 / TF / 全部话题。",
                     "text-xs text-slate-400")
                with ui.row().classes("gap-2 items-center"):
                    ui.button("启动 Bridge", on_click=start_bridge).props("color=positive")
                    ui.button("停止", on_click=stop_bridge).props("color=negative")
                host = os.environ.get("ROS_HOSTNAME") or os.environ.get("ROS_IP") or socket.gethostname()
                UI["viz_ws"] = ui.label(f"ws://{host}:8765").classes("font-mono text-sm text-sky-300")
                _lbl(f"远程时先转发：ssh -L 8765:127.0.0.1:8765 demo@{host} → 连 ws://127.0.0.1:8765",
                     "text-xs text-slate-400")
                _lbl(f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0(默认)')}"
                     f" · ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY', '0')}"
                     "（bridge 的 8765 端口是普通 TCP，不受 ROS_LOCALHOST_ONLY 影响）",
                     "text-xs text-slate-500")
                UI["viz_bridge_status"] = ui.label("未运行").classes("text-xs text-slate-300")
        with ui.column().classes("w-96 shrink-0 gap-3"):
            with _card("网页内置 2D 视图"):
                ui.markdown(
                    "“状态与地图”页已经内置浏览器二维俯视图（定位地图 / 建图点云 + 机身朝向 + 轨迹），"
                    "不依赖 X 显示，远程最稳。\n\n"
                    "需要完整 RViz 交互（3D 旋转、图层开关）时用 RViz2 或 Foxglove。"
                )
            with _card("互斥提醒"):
                ui.markdown(
                    "RViz 只是看数据，可以和定位或建图并存；\n"
                    "但 **定位 desk 与建图 desk 不能同时运行**。\n\n"
                    "RViz 视图选择应与当前运行的 desk 一致：\n"
                    "- 定位 desk → 定位视图（`/localization_map`）\n"
                    "- 建图 desk → 建图视图（`/cloud_map`）"
                )


# ---------------------------------------------------------------------------
# 节点体检页
# ---------------------------------------------------------------------------
KILL_OPTIONS = {f"/{k}": k for k in sorted(NODE_EXEC_HINTS)}


def build_health_tab() -> None:
    with ui.row().classes("w-full gap-4 items-start no-wrap"):
        with ui.column().classes("flex-grow min-w-0 gap-3"):
            with _card("全图节点（按绝对名统计）"):
                with ui.row().classes("items-center gap-4"):
                    UI["health_count"] = _lbl("等待 ROS…", "text-sm font-semibold")
                    UI["health_dup"] = _lbl("", "text-sm font-semibold")
                    ui.button("立即刷新", on_click=refresh_census_now).props("flat dense")
                cols = [
                    {"name": "name", "label": "节点", "field": "name", "align": "left"},
                    {"name": "count", "label": "次数", "field": "count"},
                    {"name": "state", "label": "判定", "field": "state"},
                    {"name": "owner", "label": "宿主进程", "field": "owner", "align": "left"},
                ]
                UI["health_nodes"] = ui.table(columns=cols, rows=[], row_key="name") \
                    .classes("w-full text-xs").props("dense wrap-cells")
                _lbl("次数 >1 = 同一节点名被多个进程/线程占用（ros2 会警告 "
                     "“share an exact name”）。宿主进程列显示 pid/pgid/启动时长："
                     "同 pgid = 一次 launch 起了两份，不同 pgid = 两次独立启动。"
                     "白名单外（如导航栈）只展示 pid，本 GUI 不停止它们。",
                     "text-xs text-slate-400")

            with _card("关键话题发布者"):
                cols = [
                    {"name": "topic", "label": "话题", "field": "topic", "align": "left"},
                    {"name": "n", "label": "发布者数", "field": "n"},
                    {"name": "state", "label": "判定", "field": "state"},
                    {"name": "pubs", "label": "发布者", "field": "pubs", "align": "left"},
                ]
                UI["health_topics"] = ui.table(columns=cols, rows=[], row_key="topic") \
                    .classes("w-full text-xs").props("dense wrap-cells")
                _lbl("/imu/data 应只有互补滤波一个源；/livox/lidar/geo_filtered 应只有 "
                     "cloud_preprocessor 一个源；/loc_status、/robot_pose 应只有当前 "
                     "desk 的 lite 节点。>1 说明旧 desk 没停干净。", "text-xs text-slate-400")

        with ui.column().classes("w-96 shrink-0 gap-3"):
            with _card("停止节点（按宿主进程）"):
                _lbl("只对 SLAM desk 链生效。甲方底盘 / 腿控 / 手柄 / Livox 驱动 / lpms "
                     "即使同名也永远不会出现在这里。", "text-xs text-amber-400")
                UI["kill_target"] = ui.select(
                    list(KILL_OPTIONS), value=None, label="选择节点") \
                    .classes("w-full").props("dense")
                with ui.row().classes("gap-2 items-center"):
                    ui.button("查看进程明细", on_click=show_kill_plan).props("flat")
                    ui.button("停止该节点", on_click=kill_selected) \
                        .props("color=negative")
                _lbl("停止方式：先 SIGINT 所在 launch（comp 滤波是 respawn=True，只杀叶子会被"
                     "拉起），3 秒后对残余 SIGKILL。每个节点都要单独确认。",
                     "text-xs text-slate-400")

                ui.separator()
                _lbl("未托管残留（只读）", "text-sm font-semibold text-slate-100")
                _lbl("PPID=1 只表示父进程已退出，不能证明进程属于本 GUI。"
                     "这里只列出疑似残留，不会自动发送信号。请核对 PGID、命令行和启动来源，"
                     "优先回到原启动终端正常 Ctrl+C。GUI 自己启动的专属进程组由生命周期管理器负责。",
                     "text-xs text-amber-400")
                with ui.row().classes("gap-2 items-center"):
                    ui.button("列出未托管残留", on_click=show_orphans).props("flat")
                UI["orphan_result"] = ui.label("").classes("text-xs text-slate-300")
                UI["kill_result"] = ui.label("").classes("text-xs text-slate-300")


# ---------------------------------------------------------------------------
# 动作
# ---------------------------------------------------------------------------
async def _confirm(message: str) -> bool:
    with ui.dialog() as dlg, ui.card().classes("w-96 gap-3"):
        ui.label(message).classes("text-sm")
        with ui.row().classes("w-full justify-end gap-2"):
            ui.button("取消", on_click=lambda: dlg.submit(False)).props("flat")
            ui.button("确定", on_click=lambda: dlg.submit(True)).props("color=negative")
    dlg.open()
    return bool(await dlg)


UI_EVENTS: queue.Queue = queue.Queue()


def _status_cb(ok, text) -> None:
    """工作线程回调：只入队，不直接碰 UI 元素。"""
    if ok is None:
        UI_EVENTS.put(str(text))
    else:
        UI_EVENTS.put(f"{'成功' if ok else '失败'}: {text}")


async def srv_click(name: str) -> None:
    if not STATE.ros_ready:
        ui.notify("ROS 未连接", type="negative")
        return
    if name == "deactivate_mapping":
        # Mapping close is a coordinated lifecycle: save PCD, stop bag, stop desk.
        await stop_desk()
        return
    if name == "activate_mapping":
        if not PROCS.running("desk") or PROCS.desk_mode != "mapping":
            ui.notify("请先用“启动建图 + 录包”启动建图会话", type="warning")
            return
        if not PROCS.running("bag") and not PROCS.start_bag():
            ui.notify("bag 录制启动失败，未激活建图", type="negative")
            return
    desc = LOC_SRV_DESC.get(name) or MAP_SRV_DESC.get(name) or name
    if name == "deactivate_localization":
        extra = "（会停止位姿输出）" if name == "deactivate_localization" \
            else "（自动保存 PCD 后清空内存）"
        if not await _confirm(f"确认调用 {name} {extra}？"):
            return
    WORKER.call_service(name, desc, _status_cb)


def _service_button(name: str, desc: str) -> None:
    """Visible feedback for service buttons; never silently no-op."""
    leaf = name.rsplit("/", 1)[-1]
    push_log("info", f"请求服务：{desc}（{leaf}）")
    if UI.get("loc_svc_result"):
        _set(UI["loc_svc_result"], f"已请求：{desc}", COL_WARN)
    WORKER.call_service(leaf, desc, _status_cb)


def request_enter_tracking() -> None:
    """Refuse locally while relocalization is not matched, with a visible reason."""
    with STATE.lock:
        loc = dict(STATE.loc)
    ok = loc.get("reloc_ok")
    fit = loc.get("reloc_fitness")
    if ok is not True or fit is None or float(fit) > 0.045:
        msg = (f"进入跟踪被拒绝：reloc_ok={ok}, fitness="
               f"{'—' if fit is None else f'{float(fit):.5f}'}，阈值 0.045。"
               "请先修正初始位姿/地图并等待重定位匹配。")
        if UI.get("loc_track_result"):
            _set(UI["loc_track_result"], msg, COL_ERR)
        push_log("warn", msg)
        ui.notify(msg, type="warning")
        return
    if UI.get("loc_track_result"):
        _set(UI["loc_track_result"], "重定位已通过，正在进入跟踪…", COL_OK)
    backend = (UI["slam_backend"].value
               if UI.get("slam_backend") else BACKEND_VENDOR)
    if backend == BACKEND_FAST_LIO:
        # The FAST-LIO adapter publishes tracking immediately after ICP succeeds;
        # it deliberately has no vendor enter_tracking service.
        realtime = bool(UI.get("fast_lio_mode")
                        and UI["fast_lio_mode"].value == FAST_MODE_REALTIME)
        ui.notify("FAST-LIO2 实时适配器已自动进入 tracking" if realtime else
                  "FAST-LIO2 ICP 已通过，适配器已自动进入 tracking",
                  type="positive")
        return
    WORKER.call_service("enter_tracking", "进入跟踪", _status_cb)


def send_initialpose() -> None:
    if ((PROCS.desk_mode == "fast_lio_realtime" and PROCS.running("desk"))
            or (UI.get("slam_backend") and UI["slam_backend"].value == BACKEND_FAST_LIO
            and UI.get("fast_lio_mode")
            and UI["fast_lio_mode"].value == FAST_MODE_REALTIME)):
        ui.notify("FAST-LIO2 实时模式不需要初始位姿", type="info")
        return
    if not STATE.ros_ready:
        ui.notify("ROS 未连接", type="negative")
        return
    x = UI["ip_x"].value or 0.0
    y = UI["ip_y"].value or 0.0
    yaw = math.radians(UI["ip_yaw"].value or 0.0)
    WORKER.request_initialpose(float(x), float(y), float(yaw), UI["ip_frame"].value)
    UI["ip_result"].set_text("已排队发布")


def begin_relocalization() -> None:
    """Expose one operator action while keeping backend-specific services hidden."""
    backend = (UI["slam_backend"].value
               if UI.get("slam_backend") else BACKEND_VENDOR)
    if backend == BACKEND_FAST_LIO:
        if ((PROCS.desk_mode == "fast_lio_realtime" and PROCS.running("desk"))
                or (UI.get("fast_lio_mode")
                    and UI["fast_lio_mode"].value == FAST_MODE_REALTIME)):
            ui.notify("实时模式以本次启动点为 map 原点，不需要重定位",
                      type="info")
            return
        ui.notify("FAST-LIO2 在启动时已进入重定位；请发送初始位姿触发 ICP",
                  type="info")
        return
    WORKER.call_service("activate_localization", "开始重定位", _status_cb)


async def start_desk() -> None:
    if not UI["mg_enable"].value:
        ui.notify("请先勾选“启用节点管理”", type="warning")
        return
    mode = "loc" if UI["mg_mode"].value == "定位 desk" else "mapping"
    with STATE.lock:
        loc_up, map_up = STATE.loc_node_up, STATE.map_node_up
    # 互斥：目标节点已在跑，或另一模式的节点在跑
    if mode == "loc" and map_up:
        push_log("err", "互斥阻止：mapping_lite_node 在运行，请先停建图")
        ui.notify("定位与建图互斥：建图节点正在运行，请先停止", type="negative")
        return
    if mode == "mapping" and loc_up:
        push_log("err", "互斥阻止：localization_lite_node 在运行，请先停定位")
        ui.notify("定位与建图互斥：定位节点正在运行，请先停止", type="negative")
        return
    if (mode == "loc" and loc_up) or (mode == "mapping" and map_up):
        push_log("warn", "目标节点已在运行（可能由外部终端启动）")
        ui.notify("该节点已在运行，无需重复启动", type="warning")
        return
    skip_rsp = bool(UI["mg_skip_rsp"].value)
    with STATE.lock:
        rows = list(STATE.node_rows)
        census_at = STATE.census_at
    if not census_at or time.monotonic() - census_at > 15.0:
        push_log("err", "启动 desk 前节点普查不可用/已过期：请稍等再试")
        ui.notify("节点普查未完成或已过期，无法确认前置链没有残留，请稍等片刻再启动。",
                  type="negative", timeout=8000)
        return
    residue = [r["name"] for r in rows
               if r["name"] in DESK_CHAIN_NODES and r.get("count", 0) > 0
               and not (skip_rsp and r["name"] == "/robot_state_publisher")]
    if residue:
        push_log("err", f"拒绝启动：前置链节点已在位 {residue}，"
                        f"再起一套会让 /imu/data、/tf_static 出现双源")
        if residue == ["/robot_state_publisher"] and not skip_rsp:
            msg = ("图里的 robot_state_publisher 是地盘 hive2_driver 的（z=0.90）。"
                   "请勾选“不启动 SLAM 的 RSP”，或先停掉地盘 RSP。")
        else:
            msg = (f"{', '.join(residue)} 已在运行（可能是上次残留）。先到“节点体检”页"
                   f"停止；未托管残留请回到原启动终端正常 Ctrl+C，再启动。")
        ui.notify(msg, type="negative", timeout=8000)
        return
    if skip_rsp:
        # 关闭 SLAM RSP 的前提：图里已有另一份 robot_state_publisher（地盘 hive2_driver）。
        # 否则 base_footprint→base_link 会整条缺失，宁可拦住。
        with STATE.lock:
            rsp_count = next((r["count"] for r in STATE.node_rows
                              if r["name"] == "/robot_state_publisher"), 0)
        if rsp_count < 1:
            push_log("err", "已勾选“不启动 SLAM RSP”，但图里没有其它 "
                            "robot_state_publisher（地盘 hive2_driver 未运行）→ "
                            "关闭会让 base_footprint→base_link 缺失，已阻止启动")
            ui.notify("未检测到地盘 RSP，不能关闭 SLAM RSP。"
                      "请取消勾选，或先启动 hive2_driver。",
                      type="negative", timeout=8000)
            return
    label = "定位 desk（localization_lite_node）" if mode == "loc" else "建图 desk（mapping_lite_node）"
    rsp_note = ("\n\nRSP：不启动 SLAM 自带 RSP，使用地盘 hive2_driver 的 "
                "base_footprint→base_link（z=0.90）。" if skip_rsp else
                "\n\nRSP：启动 desk 自带 RSP（3-link，z=0.30），可能与地盘 RSP 冲突。")
    msg = (f"将启动 {label}，同时会拉起 LiDAR 驱动、IMU 互补滤波、cloud_preprocessor。"
           f"{rsp_note}\n\n定位模式还会拉起 n01_localization_adapter 和 "
           "n01_live_scan_bridge，使 /body_pose 直接可供导航。"
           "\n\n定位与建图互斥，同一时刻只能运行一个。")
    if mode == "mapping":
        msg += ("\n\n同时开始 ros2 bag 录制；结束建图会先保存 PCD，"
                "再停止录包和 desk。")
    msg += "\n\n确认？"
    if await _confirm(msg):
        if PROCS.start_desk(mode, skip_rsp=skip_rsp):
            ui.notify("已启动，" + label, type="positive")
        else:
            ui.notify("启动失败，见运行日志", type="negative")


async def _close_mapping_session() -> bool:
    """Save mapping PCD first, then stop the bag and report success."""
    with STATE.lock:
        mode = PROCS.desk_mode
        session = str(STATE.mapping.get("session", "")).lower()
    if mode == "mapping" and session == "mapping":
        if UI.get("svc_result"):
            UI["svc_result"].set_text("正在结束建图：保存 PCD（可能需要几十秒）…")
        ok, text = await asyncio.to_thread(PROCS.deactivate_mapping)
        if UI.get("svc_result"):
            UI["svc_result"].set_text(
                ("建图保存成功" if ok else "建图保存失败") + "\n" + text[-3000:])
        push_log("ok" if ok else "err",
                 "建图 PCD 已保存" if ok else "建图 PCD 保存失败，保留 bag/desk 以便重试")
        if not ok:
            return False
    elif mode in ("fast_lio_mapping", "fast_lio_realtime"):
        if UI.get("svc_result"):
            UI["svc_result"].set_text(
                "正在停止 FAST-LIO2；pcd_save 已启用，退出时写入输出 PCD…")
        push_log("info", "FAST-LIO2 建图将通过受管进程 SIGINT 完成 PCD 落盘")
    # rosbag2 must receive SIGINT before its parent desk goes away.
    await asyncio.to_thread(PROCS.stop, "bag")
    return True


async def stop_desk() -> None:
    if not UI["mg_enable"].value:
        ui.notify("请先勾选“启用节点管理”", type="warning")
        return
    with STATE.lock:
        mode = PROCS.desk_mode
    if mode in ("mapping", "fast_lio_mapping", "fast_lio_realtime"):
        is_fast_lio = mode in ("fast_lio_mapping", "fast_lio_realtime")
        if not await _confirm(
                "结束建图将依次执行：\n"
                + ("1) FAST-LIO2 收到 SIGINT 后保存 PCD；\n" if is_fast_lio else
                   "1) deactivate_mapping 保存全分辨率 PCD；\n")
                + "2) 停止 bag 录制并写入 metadata；\n"
                + "3) 停止建图后端。\n\n确认？"):
            return
        if not await _close_mapping_session():
            ui.notify("建图保存失败：bag 和 desk 仍保留，请检查后重试", type="negative")
            return
        await asyncio.to_thread(PROCS.stop, "desk")
        await asyncio.to_thread(PROCS.stop, "rviz")
        # 复位隐藏的模式控件：否则后续任何仍直接调用 start_desk() 的入口
        # 都会沿用“建图 desk”这个残留值（曾经导致“① 启动定位节点”误起建图栈）。
        if UI.get("mg_mode"):
            UI["mg_mode"].value = "定位 desk"
        ui.notify("建图已保存，bag 已停止，desk/RViz 已停止", type="positive")
        return
    nav_live = _nav_ready_state() is True or _node_present("/scan_planner_node")
    # 最常见状态是“导航已停、bridge 仍随定位栈存活”：此时 nav_ready 心跳已停、
    # scan_planner_node 已被停止脚本清掉，只看 nav_live 会漏掉 bridge 探测。
    bridge_up = _node_present("/n01_live_scan_bridge")
    if nav_live or bridge_up:
        if nav_live:
            warn = ("导航栈仍在运行。定位 desk 进程组同时承载导航桥接"
                    "（/cloud、/body_pose），\n"
                    "停止定位会让导航立即 BLOCKED 并留下依赖残留。\n\n"
                    "建议先“停止导航”。仍要停止定位吗？")
        else:
            warn = ("导航已停止，但 n01_live_scan_bridge 仍由定位 desk 承载"
                    "（/cloud、/body_pose 由它提供）。\n"
                    "停止定位会连同它启动的 Livox 驱动一起移除此桥接。\n\n"
                    "之后要单独启动导航时需先重新启动定位 desk。仍要停止定位吗？")
        if not await _confirm(warn):
            return
    stop_msg = "确认停止当前 desk 进程？（会中断定位/建图输出"
    if bridge_up:
        stop_msg += "，并移除 Livox 驱动与 n01_live_scan_bridge 桥接"
    stop_msg += "）"
    if await _confirm(stop_msg):
        await asyncio.to_thread(PROCS.stop, "desk")
        await asyncio.to_thread(PROCS.stop, "rviz")
        # 停止后复位隐藏模式，避免 mg_mode 残留影响后续启动入口。
        if UI.get("mg_mode"):
            UI["mg_mode"].value = "定位 desk"


async def start_rviz() -> None:
    _text, available, _fallback = _display_info()
    if not available:
        push_log("err", f"RViz2 无法启动：{_text}")
        ui.notify(_text, type="negative")
        return
    mode = "loc" if UI["viz_mode"].value == "定位视图" else "mapping"
    if await _confirm("将启动 RViz2 原生窗口（走 vendor 的 vis_world 启动脚本，内部自动 source）。确认？"):
        PROCS.start_rviz(mode)


async def stop_rviz() -> None:
    threading.Thread(target=PROCS.stop, args=("rviz",), daemon=True).start()


async def start_bridge() -> None:
    if await _confirm("将启动 Foxglove Bridge（0.0.0.0:8765），"
                      "浏览器用 Foxglove Studio 连接。确认？"):
        PROCS.start_bridge()


async def stop_bridge() -> None:
    threading.Thread(target=PROCS.stop, args=("foxglove",), daemon=True).start()


# ---- 节点体检动作 ----
KILL_EVENTS: queue.Queue = queue.Queue()


def _plan_lines(plan: dict) -> list[str]:
    out = []
    for r in plan.get("launch") or []:
        out.append(f"launch  pid {r['pid']:<6} {r['cmd'][:110]}")
    for r in plan.get("killable") or []:
        out.append(f"进程    pid {r['pid']:<6} {r['cmd'][:110]}")
    for r in plan.get("unmanaged") or []:
        out.append(f"外部进程 pid {r['pid']:<6} {r['cmd'][:110]}  ← 只展示，不会停止")
    for r in plan.get("protected") or []:
        out.append(f"受保护  pid {r['pid']:<6} {r['cmd'][:110]}  ← 不会停止")
    return out


async def _confirm_detail(title: str, lines: list[str]) -> bool:
    with ui.dialog() as dlg, ui.card().classes("w-[42rem] gap-3"):
        ui.label(title).classes("text-sm font-semibold")
        ui.label("\n".join(lines)).classes(
            "text-xs font-mono whitespace-pre-wrap break-all max-h-72 overflow-auto")
        with ui.row().classes("w-full justify-end gap-2"):
            ui.button("取消", on_click=lambda: dlg.submit(False)).props("flat")
            ui.button("确认停止", on_click=lambda: dlg.submit(True)).props("color=negative")
    dlg.open()
    return bool(await dlg)


def refresh_census_now() -> None:
    if not STATE.ros_ready:
        ui.notify("ROS 未连接", type="negative")
        return
    WORKER.force_census()
    ui.notify("已请求重新普查，约 2 秒内刷新")


async def show_kill_plan() -> None:
    name = UI["kill_target"].value
    if not name:
        ui.notify("先选一个节点", type="warning")
        return
    plan = await asyncio.to_thread(node_kill_plan, name)
    if not plan["supported"]:
        ui.notify("该节点不在可停止白名单内", type="warning")
        return
    lines = _plan_lines(plan) or ["（没有找到对应的宿主进程）"]
    with ui.dialog() as dlg, ui.card().classes("w-[42rem] gap-3"):
        ui.label(f"{name} 的宿主进程").classes("text-sm font-semibold")
        ui.label("\n".join(lines)).classes(
            "text-xs font-mono whitespace-pre-wrap break-all max-h-72 overflow-auto")
        ui.button("关闭", on_click=dlg.close).props("flat")
    dlg.open()


async def kill_selected() -> None:
    name = UI["kill_target"].value
    if not name:
        ui.notify("先选一个节点", type="warning")
        return
    if not STATE.ros_ready:
        ui.notify("ROS 未连接", type="negative")
        return
    plan = await asyncio.to_thread(node_kill_plan, name)
    if not plan["supported"]:
        ui.notify("该节点不在可停止白名单内", type="warning")
        return
    if not plan["killable"]:
        msg = ("检测到外部管理的进程；GUI 只展示，不会越权停止。请回到原启动终端 Ctrl+C。"
               if plan["unmanaged"] else
               ("没有找到可停止的宿主进程（节点可能已退出，或进程属于其他用户）"
                if plan["procs"] else "按进程名没有匹配到该节点"))
        push_log("warn", f"{name}: {msg}")
        ui.notify(msg, type="warning")
        return
    lines = _plan_lines(plan)
    lines.append("")
    lines.append("仅停止由当前 GUI 登记的专属进程组；外部进程只展示，不会发送信号。")
    lines.append("停止顺序：SIGINT，等待 3 秒，仍存活时对确认归属的 PID 升级 SIGKILL。")
    lines.append("这会中断该节点当前的定位/建图输出。")
    if not await _confirm_detail(f"停止 {name}？", lines):
        return
    UI["kill_result"].set_text(f"{time.strftime('%H:%M:%S')} 停止 {name} 中…")
    threading.Thread(target=_kill_worker, args=(name,), daemon=True).start()


def _kill_worker(name: str) -> None:
    try:
        execute_node_kill(name)
    except Exception as exc:  # noqa: BLE001
        push_log("err", f"{name}: 停止异常 {exc}")
    finally:
        WORKER.force_census()
        KILL_EVENTS.put(f"{time.strftime('%H:%M:%S')} {name} 停止流程结束（看页底日志）")


def _orphan_lines(targets: list[dict]) -> list[str]:
    return [f"{_fmt_proc(t):<32} {t['cmd'][:100]}" for t in targets]


async def show_orphans() -> None:
    targets = await asyncio.to_thread(collect_unmanaged_residue)
    if not targets:
        UI["orphan_result"].set_text("没有发现疑似未托管残留")
        ui.notify("没有发现疑似未托管残留", type="positive")
        return
    lines = _orphan_lines(targets)
    lines.append("")
    lines.append("安全提示：这些进程未被当前 GUI 生命周期管理器登记。")
    lines.append("本 GUI 不会自动终止它们；请从原启动终端 Ctrl+C，或由对应服务管理器停止。")
    with ui.dialog() as dlg, ui.card().classes("w-[42rem] gap-3"):
        ui.label(f"疑似未托管残留 {len(targets)} 个（只读检查）").classes("text-sm font-semibold")
        ui.label("\n".join(lines)).classes(
            "text-xs font-mono whitespace-pre-wrap break-all max-h-72 overflow-auto")
        ui.button("关闭", on_click=dlg.close).props("flat")
    dlg.open()


# ---------------------------------------------------------------------------
# 刷新
# ---------------------------------------------------------------------------
def _age_text(key: str) -> str:
    age = STATE.age_of(key)
    if age is None:
        return "无数据"
    return f"{age * 1000:.0f} ms" if age < 0.5 else f"{age:.1f} s"


NODE_CATEGORIES = ["底盘与执行", "传感器", "建图定位", "适配器", "导航规划",
                   "TF与机器人模型", "可视化与GUI", "其他"]


def node_category(name: str) -> tuple[str, str]:
    """Classify observed nodes; category never grants permission to stop them."""
    short = name.rsplit("/", 1)[-1].lower()
    if short in ("n01_localization_adapter", "n01_live_scan_bridge",
                 "n01_fast_lio_pose_adapter", "n01_fast_lio_odometry_adapter"):
        return "适配器", "位姿/状态归一或导航点云桥接"
    if short in ("robot_state_publisher", "static_tf_publisher_node",
                 "static_transform_publisher", "joint_state_publisher") or "transform_publisher" in short:
        return "TF与机器人模型", "机体/传感器坐标关系"
    if any(token in short for token in ("rviz", "foxglove", "gui", "console")):
        return "可视化与GUI", "操作与可视化"
    if "traj_tracker" in short or "slow_motion" in short:
        return "底盘与执行", "轨迹执行（涉及运动输出）"
    if (short.startswith("n01_live_") or short in
            ("scan_planner_node", "n01_navigation_path", "n01_static_map", "n01_hive2_trajectory_monitor",
             "n01_shadow_path_follower")):
        return "导航规划", "静态地图/规划/门控/轨迹评估"
    if any(token in short for token in ("localization", "mapping", "fastlio", "relocaliz", "slam3d")):
        return "建图定位", "地图构建或地图定位"
    if any(token in short for token in ("livox", "imu", "lpms", "lidar", "complementary", "hipnuc")):
        return "传感器", "雷达/IMU驱动与预处理"
    if any(token in short for token in ("controller", "chassis", "wheel", "steer", "ethercat",
                                       "joy", "teleop", "battery", "traj_tracker", "slow_motion")):
        return "底盘与执行", "运控/执行/遥控/底盘状态"
    return "其他", "未识别，按完整节点名核对来源"


def node_management_rows(rows, pubs):
    category = UI["node_category"].value if UI.get("node_category") else "全部"
    query = str(UI["node_search"].value or "").strip().lower() if UI.get("node_search") else ""
    result = []
    for row in rows:
        group, role = node_category(row["name"])
        item = dict(row, category=group, role=role)
        item["topics"] = " · ".join(t for t, names in pubs.items() if row["name"] in names) or "—"
        item["management"] = ("GUI受管停止计划" if row.get("killable") else
                              "导航统一入口" if group == "导航规划" else "外部/受保护，原入口管理")
        if category != "全部" and category != group:
            continue
        if query and query not in (item["name"] + item["owner"] + item["topics"]).lower():
            continue
        result.append(item)
    return result


def select_node_detail(event) -> None:
    name = event.value
    with STATE.lock:
        STATE.node_detail_target = name
        STATE.node_detail = "正在读取所选节点的话题接口…" if name else "选择节点查看发布/订阅接口。"
    if name and UI.get("kill_target"):
        UI["kill_target"].value = name
    WORKER.force_census()


def refresh_health() -> None:
    """节点普查结果 → 状态页横幅 + 体检页表格。1 Hz，纯 UI 更新。"""
    if UI.get("kill_result"):
        try:
            while True:
                UI["kill_result"].set_text(KILL_EVENTS.get_nowait())
        except queue.Empty:
            pass

    with STATE.lock:
        rows = [dict(r) for r in STATE.node_rows]
        dups = list(STATE.dup_groups)
        pubs = {k: list(v) for k, v in STATE.topic_pubs.items()}
        at = STATE.census_at
        ready = STATE.ros_ready
    when = time.strftime("%H:%M:%S", time.localtime(
        time.time() - max(0.0, time.monotonic() - at))) if at else "—"

    orphans = [r["name"] for r in rows if "孤儿" in r["state"]]
    if UI.get("dup_banner"):
        if not ready:
            txt, color = "ROS 未连接，无法普查", COL_ERR
        elif not at:
            txt, color = "普查中…（首次约 2 秒）", COL_WARN
        elif dups or orphans:
            bits = []
            if dups:
                bits.append(f"⚠ 发现 {len(dups)} 组重名节点：{', '.join(dups)}")
            if orphans:
                bits.append(f"未托管残留（父进程已死）：{', '.join(orphans)}")
            txt = ("；".join(bits)
                   + f"（{when}，共 {len(rows)} 个节点）→ 在节点管理中核对归属")
            color = COL_ERR if dups else COL_WARN
        else:
            txt = f"✓ {len(rows)} 个节点，无重名（{when}）"
            color = COL_OK
        if UI["dup_banner"].text != txt:
            UI["dup_banner"].set_text(txt)
            UI["dup_banner"].style(replace=f"color:{color}")

    node_tbl = UI.get("health_nodes")
    if node_tbl is not None:
        # 控制台“诊断”页放的是标签，旧体检页放的是表格；两者都兼容。
        if hasattr(node_tbl, "rows"):
            visible_rows = node_management_rows(rows, pubs)
            if node_tbl.rows != visible_rows:
                node_tbl.rows = visible_rows
                node_tbl.update()
        else:
            _set(node_tbl, f"{len(rows)} 个节点", TEXT)
    if UI.get("node_category_summary"):
        totals = Counter(node_category(r["name"])[0] for r in rows)
        _set(UI["node_category_summary"], " · ".join(
            f"{category} {totals.get(category, 0)}" for category in NODE_CATEGORIES))
    if UI.get("node_detail_select"):
        options = sorted(r["name"] for r in rows)
        selector = UI["node_detail_select"]
        if selector.options != options:
            selector.set_options(options, value=selector.value if selector.value in options else None)
    if UI.get("node_detail"):
        with STATE.lock:
            detail = getattr(STATE, "node_detail", None)
        if detail:
            _set(UI["node_detail"], detail)
    if UI.get("health_count"):
        _set(UI["health_count"], f"节点 {len(rows)} 个", TEXT)
    if UI.get("health_dup"):
        _set(UI["health_dup"],
             f"重名 {len(dups)} 组" + (f"：{', '.join(dups)}" if dups else "")
             + (f" / 未托管残留 {len(orphans)} 个" if orphans else ""),
             COL_ERR if dups else (COL_WARN if orphans else COL_OK))

    ttable = UI.get("health_topics")
    if ttable is not None and pubs:
        trows = []
        for topic in CENSUS_TOPICS:
            names = pubs.get(topic, [])
            if not names:
                state = "× 无发布者"
            elif len(names) > 1:
                state = "TF多发布者，需核对边" if topic in ("/tf", "/tf_static") else "⚠ 多源"
            else:
                state = "✓ 正常"
            trows.append({"topic": topic, "n": len(names), "state": state,
                          "pubs": ", ".join(names) or "—"})
        if hasattr(ttable, "rows"):
            if ttable.rows != trows:
                ttable.rows = trows
                ttable.update()
        else:
            # 诊断页这里是 label（没有 .rows），label 分支必须自己把“多源”
            # 说出来，否则 /imu/data 双发布者这类事故在 GUI 里完全不可见。
            n_ok = sum(1 for names in pubs.values() if names)
            multi = [t for t in CENSUS_TOPICS if len(pubs.get(t, [])) > 1
                     and t not in ("/tf", "/tf_static")]
            text = f"{n_ok}/{len(CENSUS_TOPICS)} 有发布者"
            if multi:
                detail = ", ".join(f"{t}({len(pubs[t])})" for t in multi[:4])
                if len(multi) > 4:
                    detail += " …"
                _set(ttable, f"{text}；⚠ 多源 {len(multi)} 个：{detail}", COL_ERR)
            else:
                _set(ttable, text, COL_OK if n_ok else MUTED)

    # 进程 / 可视化 / 数据源缺口（原先由已删除的 refresh_status 写入）
    if UI.get("proc_nodes"):
        with STATE.lock:
            loc_up, map_up = STATE.loc_node_up, STATE.map_node_up
        if loc_up and map_up:
            _set(UI["proc_nodes"], "定位 ✓ / 建图 ✓（互斥冲突!）", COL_ERR)
        elif loc_up:
            _set(UI["proc_nodes"], "定位 ✓ / 建图 ×", COL_OK)
        elif map_up:
            _set(UI["proc_nodes"], "定位 × / 建图 ✓", COL_OK)
        else:
            _set(UI["proc_nodes"], "定位 × / 建图 ×", COL_IDLE)
    if UI.get("proc_procs"):
        _set(UI["proc_procs"], PROCS.status_text())
    if UI.get("viz_rviz_status") is not None:
        rv = PROCS.running("rviz")
        _set(UI["viz_rviz_status"], f"RViz: {'运行中' if rv else '未运行'}",
             COL_OK if rv else COL_IDLE)
    if UI.get("viz_bridge_status") is not None:
        br = PROCS.running("foxglove")
        _set(UI["viz_bridge_status"], f"Bridge: {'运行中' if br else '未运行'}",
             COL_OK if br else COL_IDLE)
    if UI.get("ds_table") is not None and not hasattr(UI["ds_table"], "rows") and pubs:
        missing = [t for t in CENSUS_TOPICS if not pubs.get(t)]
        multi = [t for t in CENSUS_TOPICS if len(pubs.get(t, [])) > 1
                 and t not in ("/tf", "/tf_static")]
        text = f"无发布者 {len(missing)}/{len(CENSUS_TOPICS)}"
        if missing:
            text += "：" + ", ".join(missing[:4]) + (" …" if len(missing) > 4 else "")
        if multi:
            text += "；⚠ 多源: " + ", ".join(multi[:4]) + (" …" if len(multi) > 4 else "")
        # CENSUS_TOPICS 混有建图专属（/mapping_status、/cloud_map）与导航专属话题，
        # 定位/建图本就互斥（见 proc_nodes 的 OK 判定），单模式运行时缺席是预期：
        # 只有“多源”才是必须排查的故障，缺席只做黄色提示。
        _set(UI["ds_table"], text,
             COL_ERR if multi else (COL_WARN if missing else COL_OK))


def draw_chart(force: bool = False) -> None:
    if not UI.get("chart") or not STATE.ros_ready:
        return
    pts, src = _view_points()
    with STATE.lock:
        pose = STATE.pose
        trail = list(STATE.trail)
    if not UI["trail_on"].value:
        trail = []
    opt = _chart_option(pts, trail, pose)
    key = (src, _chart_cache.get("key"), len(trail), bool(pose),
           round(pose[0], 1) if pose else None, round(pose[1], 1) if pose else None,
           round(pose[2], 2) if pose else None, UI["range"].value)
    if not force and key == _chart_cache.get("okey"):
        return
    _chart_cache["okey"] = key
    # nicegui 3.x 的 EChart.options 是只读 property，内部真值在 _props["options"]
    UI["chart"]._props["options"] = opt  # noqa: SLF001
    UI["chart"].update()
    UI["chart_info"].set_text(
        f"{src}: {len(pts):,} 点 · {time.strftime('%H:%M:%S')}"
        if pts else f"{src}: 等待点云（检查上游是否发布）")


def drain_ui_events() -> None:
    """把工作线程/服务回调的事件搬到 UI 线程执行。"""
    while True:
        try:
            text = UI_EVENTS.get_nowait()
        except queue.Empty:
            break
        if UI.get("svc_result"):
            UI["svc_result"].set_text(text)
        elif UI.get("ip_result"):
            UI["ip_result"].set_text(text)


def drain_logs() -> None:
    lines = []
    while True:
        try:
            lines.append(STATE.log_q.get_nowait())
        except queue.Empty:
            break
    if lines and UI.get("log"):
        UI["log"].push("\n".join(lines))
    drain_ui_events()


# ---------------------------------------------------------------------------
# 页面
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# Single-page console: projected occupancy grid + manual SLAM/nav control
# ---------------------------------------------------------------------------
_GRID_VIEW: dict = {}
_GRID_CACHE: dict = {"key": None}
_BLANK_PNG = (
    "data:image/png;base64,"
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII="
)
GRID_COLORS = {
    "unknown": (15, 18, 20),
    "free": (27, 33, 35),
    "occupied": (112, 166, 224),
    "dynamic": (225, 142, 64),
    "grid": (43, 48, 50),
    "trail": (150, 150, 140),
    "robot": (217, 89, 38),
    "goal": (30, 200, 140),
    "goal_failed": (225, 70, 70),
    "global_plan": (55, 125, 255),
    "global_plan_stale": (65, 85, 120),
    "plan": (35, 220, 220),
    "plan_stale": (225, 150, 60),
}


def _card2(title: str):
    card = ui.card().classes("w-full q-pa-sm gap-1 compact-card")
    with card:
        _lbl(title, "text-base font-semibold text-slate-100")
    return card


def _grid_source():
    src = str(UI["src"].value) if UI.get("src") else "自动"
    with STATE.lock:
        loc = STATE.loc_pts
        cloud = STATE.cloud_pts
        nav_static = STATE.nav_static_pts
        nav_dyn = STATE.nav_dyn_pts
        loc_seq = STATE.loc_seq
        cloud_seq = STATE.cloud_seq
        nav_seq = STATE.nav_map_seq
        pose = STATE.pose
        pose_z = STATE.pose_z
        trail = list(STATE.trail)
        loc_session = str(STATE.loc.get("session", "")).lower()
        map_session = str(STATE.mapping.get("session", "")).lower()

    def stack(parts):
        valid = [p for p in parts if p is not None and len(p)]
        return np.vstack(valid) if valid else None

    nav_pts = stack([nav_static, nav_dyn])
    if src == "定位地图":
        return loc, "定位 PCD 投影", loc_seq, None, pose, pose_z, trail
    if src == "建图":
        return cloud, "建图点云投影", cloud_seq, None, pose, pose_z, trail
    if src == "导航栅格":
        return nav_pts, "导航占用栅格投影", nav_seq, nav_dyn, pose, pose_z, trail
    if loc_session in ("tracking", "relocalization") and loc is not None:
        return loc, "定位 PCD 投影", loc_seq, None, pose, pose_z, trail
    if map_session == "mapping" and cloud is not None:
        return cloud, "建图点云投影", cloud_seq, None, pose, pose_z, trail
    if nav_pts is not None:
        return nav_pts, "导航占用栅格投影", nav_seq, nav_dyn, pose, pose_z, trail
    if loc is not None:
        return loc, "定位 PCD 投影", loc_seq, None, pose, pose_z, trail
    return cloud, "建图点云投影", cloud_seq, None, pose, pose_z, trail


def _grid_ground():
    try:
        value = float(UI["ground_z"].value) if UI.get("ground_z") else 0.139
    except (TypeError, ValueError):
        value = 0.139
    return value if math.isfinite(value) else 0.139


def _map_xy_to_pixel(x, y, view):
    px = int(round((x - view["x0"]) / view["size_m"] * GRID_IMAGE_PX))
    py = int(round((1.0 - (y - view["y0"]) / view["size_m"]) * GRID_IMAGE_PX))
    return px, py


def _rasterize(points, view, ground, dynamic_points):
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    zmin, zmax = ground + 0.08, ground + 1.60
    valid &= (z >= zmin) & (z <= zmax)
    if not valid.any():
        return None, zmin, zmax
    x, y = x[valid], y[valid]
    size_m = view["size_m"]
    n = max(60, min(600, int(round(size_m / GRID_RES))))
    res = size_m / n
    ix = np.floor((x - view["x0"]) / res).astype(np.int32)
    iy = np.floor((y - view["y0"]) / res).astype(np.int32)
    inside = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
    occ = np.zeros((n, n), dtype=np.uint8)
    occ[iy[inside], ix[inside]] = 1
    dyn = np.zeros_like(occ)
    if dynamic_points is not None and len(dynamic_points):
        dx, dy, dz = (dynamic_points[:, 0], dynamic_points[:, 1],
                      dynamic_points[:, 2])
        dmask = (np.isfinite(dx) & np.isfinite(dy) & np.isfinite(dz)
                 & (dz >= zmin) & (dz <= zmax))
        dix = np.floor((dx[dmask] - view["x0"]) / res).astype(np.int32)
        diy = np.floor((dy[dmask] - view["y0"]) / res).astype(np.int32)
        din = (dix >= 0) & (dix < n) & (diy >= 0) & (diy < n)
        dyn[diy[din], dix[din]] = 1
    rgb = np.zeros((n, n, 3), dtype=np.uint8)
    rgb[:] = GRID_COLORS["unknown"]
    xmin, xmax = float(np.min(x)), float(np.max(x))
    ymin, ymax = float(np.min(y)), float(np.max(y))
    ix0 = int(max(0, math.floor((xmin - view["x0"]) / res)))
    ix1 = int(min(n, math.ceil((xmax - view["x0"]) / res)))
    iy0 = int(max(0, math.floor((ymin - view["y0"]) / res)))
    iy1 = int(min(n, math.ceil((ymax - view["y0"]) / res)))
    if ix1 > ix0 and iy1 > iy0:
        rgb[iy0:iy1, ix0:ix1] = GRID_COLORS["free"]
    rgb[occ > 0] = GRID_COLORS["occupied"]
    rgb[dyn > 0] = GRID_COLORS["dynamic"]
    return rgb, zmin, zmax


def _grid_overlay(img, view, pose, trail, goal_xy, plan_xy=None,
                  plan_active=True, goal_failed=False, global_xy=None,
                  global_active=True):
    draw = ImageDraw.Draw(img)
    for gx in range(int(math.ceil(view["x0"])),
                    int(math.floor(view["x0"] + view["size_m"])) + 1):
        px, _ = _map_xy_to_pixel(gx, view["y0"], view)
        draw.line([(px, 0), (px, GRID_IMAGE_PX)], fill=GRID_COLORS["grid"], width=1)
    for gy in range(int(math.ceil(view["y0"])),
                    int(math.floor(view["y0"] + view["size_m"])) + 1):
        _, py = _map_xy_to_pixel(view["x0"], gy, view)
        draw.line([(0, py), (GRID_IMAGE_PX, py)], fill=GRID_COLORS["grid"], width=1)
    trail_on = bool(UI["trail_on"].value) if UI.get("trail_on") else True
    if trail_on and trail:
        pts = [_map_xy_to_pixel(t[0], t[1], view) for t in trail]
        if len(pts) >= 2:
            draw.line(pts, fill=GRID_COLORS["trail"], width=2)
    if pose is not None:
        x, y, yaw = pose
        px, py = _map_xy_to_pixel(x, y, view)
        r = 7
        draw.polygon([
            (px + r * math.cos(yaw), py - r * math.sin(yaw)),
            (px + r * math.cos(yaw + 2.5), py - r * math.sin(yaw + 2.5)),
            (px + r * math.cos(yaw - 2.5), py - r * math.sin(yaw - 2.5)),
        ], fill=GRID_COLORS["robot"])
    if global_xy:
        global_px = [_map_xy_to_pixel(x, y, view) for x, y in global_xy]
        if len(global_px) >= 2:
            draw.line(global_px,
                      fill=GRID_COLORS["global_plan" if global_active
                                       else "global_plan_stale"],
                      width=4 if global_active else 2)
    if plan_xy:
        plan_px = [_map_xy_to_pixel(x, y, view) for x, y in plan_xy]
        if len(plan_px) >= 2:
            draw.line(plan_px,
                      fill=GRID_COLORS["plan" if plan_active else "plan_stale"],
                      width=3 if plan_active else 2)
    if goal_xy is not None:
        px, py = _map_xy_to_pixel(goal_xy[0], goal_xy[1], view)
        goal_color = GRID_COLORS["goal_failed" if goal_failed else "goal"]
        draw.ellipse([px - 6, py - 6, px + 6, py + 6],
                     outline=goal_color, width=2)
        draw.line([(px - 9, py), (px + 9, py)], fill=goal_color, width=1)
        draw.line([(px, py - 9), (px, py + 9)], fill=goal_color, width=1)
    return img


def draw_grid(force: bool = False) -> None:
    if not UI.get("map_image"):
        return
    points, label, seq, dynamic_points, pose, _pose_z, trail = _grid_source()
    with STATE.lock:
        plan_xy = list(STATE.plan_xy)
        plan_seq = STATE.plan_seq
        plan_active = STATE.plan_active
        goal_failed = STATE.goal_failed
        global_xy = list(STATE.global_xy)
        global_seq = STATE.global_seq
        global_active = STATE.global_active
    # 发布者中途消失时，最后一条路径不能继续按“活动”着色。
    # 与 _chain_state / refresh_console 共用同一判据 _nav_paths_live()，
    # 避免“地图灰了导航页还绿”的三套定义漂移。
    nav_path_live, plan_path_live = _nav_paths_live()
    if not plan_path_live:
        plan_active = False
    # 与 _chain_state / refresh_console 完全一致的谓词：全局路径“任务中”
    # 必须 锁存位 + nav 心跳 + 局部路径回放 同时成立。只按 nav 心跳放行，
    # 会在“任务结束但导航栈未停”时把已结束的路径继续画成活动蓝线。
    # plan_active 需一并纳入：轨迹清空时它立即为 False，而 plan_path_live
    # 还要等 ≤2 s 回放窗口过期——不查它就出现“地图仍绿、数据链已灰”的窗口。
    if not (plan_active and nav_path_live and plan_path_live):
        global_active = False
    if points is None or not len(points):
        UI["map_image"].set_source(_BLANK_PNG)
        if UI.get("grid_info"):
            UI["grid_info"].set_text("等待地图数据…（定位 PCD / 建图点云 / 导航占用栅格）")
        return
    try:
        half = float(str(UI["range"].value).replace("m", "")) / 2.0
    except (TypeError, ValueError):
        half = 10.0
    half = max(3.0, min(50.0, half))
    size_m = 2.0 * half
    center_x = pose[0] if pose is not None else float(np.median(points[:, 0]))
    center_y = pose[1] if pose is not None else float(np.median(points[:, 1]))
    view = {"x0": center_x - half, "y0": center_y - half, "size_m": size_m,
            "center": (center_x, center_y), "half": half}
    ground = _grid_ground()
    goal_xy = None
    try:
        if UI.get("goal_x") and UI.get("goal_y"):
            goal_xy = (float(UI["goal_x"].value), float(UI["goal_y"].value))
    except (TypeError, ValueError):
        goal_xy = None
    key = (str(UI["src"].value), str(UI["range"].value), round(ground, 3),
           seq, None if pose is None else tuple(round(v, 2) for v in pose),
           len(trail), goal_xy, plan_seq, plan_active, goal_failed,
           global_seq, global_active,
           bool(UI["trail_on"].value) if UI.get("trail_on") else True)
    if not force and key == _GRID_CACHE.get("key"):
        return
    rgb, zmin, zmax = _rasterize(points, view, ground, dynamic_points)
    if rgb is None:
        UI["map_image"].set_source(_BLANK_PNG)
        if UI.get("grid_info"):
            UI["grid_info"].set_text(
                f"{label} 在 z [{ground + 0.08:.2f}, {ground + 1.60:.2f}] 内无点；"
                "请调整地面 z")
        return
    # Raster row 0 is map y0 (bottom); flip so image top is map ymax, matching
    # the robot/trail overlay conversion.
    # rgb is already uint8 HxWx3; let Pillow infer RGB. Passing mode= is
    # deprecated in Pillow 11 and removed in Pillow 13.
    img = Image.fromarray(rgb[::-1]).resize(
        (GRID_IMAGE_PX, GRID_IMAGE_PX), Image.NEAREST)
    img = _grid_overlay(img, view, pose, trail, goal_xy, [], plan_active,
                        goal_failed, global_xy, global_active)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    UI["map_image"].set_source(
        "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode())
    _GRID_VIEW.clear()
    _GRID_VIEW.update(view)
    _GRID_CACHE["key"] = key
    if UI.get("grid_info"):
        UI["grid_info"].set_text(
            f"{label} · {len(points)} pts · z [{zmin:.2f}, {zmax:.2f}] · "
            f"ground {ground:.2f} · {size_m:.0f} m 视野")


def on_map_click(e) -> None:
    if not _GRID_VIEW:
        return
    x = _GRID_VIEW["x0"] + (e.image_x / GRID_IMAGE_PX) * _GRID_VIEW["size_m"]
    y = _GRID_VIEW["y0"] + (1.0 - e.image_y / GRID_IMAGE_PX) * _GRID_VIEW["size_m"]
    if UI.get("goal_x"):
        UI["goal_x"].value = round(x, 2)
    if UI.get("goal_y"):
        UI["goal_y"].value = round(y, 2)
    if UI.get("goal_result"):
        UI["goal_result"].set_text(
            f"已从地图选点 map ({x:.2f}, {y:.2f})；确认后点“发送目标”")
    draw_grid(force=True)


def _goal_failure_reason() -> str:
    with STATE.lock:
        gate = dict(STATE.goal_status)
        feas = dict(STATE.feasibility)
    if gate.get("goal_received") and gate.get("last_goal_accepted") is False:
        return f"Goal Gate 拒绝：{gate.get('reason', '—')}"
    if feas:
        status = str(feas.get("status", "") or feas.get("reason", ""))
        reason = str(feas.get("reason", ""))
        if status in ("WAITING_FOR_BSPLINE", "STALE_BSPLINE"):
            return ("规划器没有生成 B 样条：scan_planner 可能无解，"
                    "或规划线程卡住/未响应（当前点判失败，建议重启导航）")
        if status:
            return (f"规划器反馈：{status}"
                    + (f" / {reason}" if reason and reason != status else ""))
    return "目标已接受，但 20 s 内没有新路径（规划器无输出）"


async def send_goal() -> None:
    if not STATE.ros_ready:
        ui.notify("ROS 未连接", type="negative")
        return
    try:
        x = float(UI["goal_x"].value)
        y = float(UI["goal_y"].value)
        yaw = math.radians(float(UI["goal_yaw"].value or 0.0))
    except (TypeError, ValueError):
        ui.notify("目标 x/y/yaw 无效", type="negative")
        return
    with STATE.lock:
        pending_goal = STATE.plan_pending or STATE.global_pending
    if pending_goal:
        UI["goal_result"].set_text(
            "上一个目标仍在规划；请等它成功或失败（会自动清除）后再发新点。")
        ui.notify("上一个目标仍在规划，请等待结果", type="warning")
        return
    if not await _confirm(
            f"发送导航目标 map ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.1f}°？\n"
            "导航是 planning-only，不会直接驱动底盘。"):
        return
    WORKER.request_goal(x, y, yaw, "map")
    with STATE.lock:
        token = STATE.goal_seq
        goal_ns = STATE.plan_goal_ns
    # A* itself is fast (~15 ms) but the planner may queue behind a previous
    # smoothing job; observed end-to-end path latency is ~18-21 s.
    global_timeout = float(os.environ.get("N01_GLOBAL_TIMEOUT_S", "35"))
    local_timeout = float(os.environ.get("N01_LOCAL_TIMEOUT_S", "40"))
    tol = float(os.environ.get("N01_PATH_GOAL_TOL_M", "0.30"))
    UI["goal_result"].set_text(
        f"目标已发布；全局 A* 阶段（≤{global_timeout:.0f} s，失败立即清除允许下一个点）…")
    push_log("info", f"导航目标已发布 ({x:.2f}, {y:.2f})；全局 A* 阶段开始")

    def fail(reason: str, phase: str, elapsed_ms: float, pts: int = 0) -> None:
        with STATE.lock:
            if STATE.goal_seq == token:
                STATE.plan_pending = False
                STATE.global_pending = False
                STATE.goal_failed = True
        UI["goal_result"].set_text(
            f"规划失败（{phase}）：{reason}；耗时 {elapsed_ms:.0f} ms；"
            "目标已清除，允许重新选点；旧路径/过期路径不计入成功。")
        push_log("err", f"目标 ({x:.2f}, {y:.2f}) {phase}失败：{reason}；耗时 {elapsed_ms:.0f} ms；已清除")
        ui.notify(f"{phase}规划失败：{reason}", type="negative", timeout=8000)

    # Phase 1: global A* path. Fail fast; do not keep the goal pending.
    global_ok = False
    global_ms = 0.0
    global_deadline = time.monotonic() + max(3.0, global_timeout)
    while time.monotonic() < global_deadline:
        await asyncio.sleep(0.1)
        with STATE.lock:
            seq = STATE.goal_seq
            g_success_ns = STATE.global_success_ns
            g_pts = STATE.global_pts
            g_distance = STATE.global_goal_distance
            gate = dict(STATE.goal_status)
        if seq != token:
            UI["goal_result"].set_text("目标已被更新的目标取代；本次等待结束。")
            return
        if g_success_ns and g_success_ns >= goal_ns:
            global_ms = max(0.0, (g_success_ns - goal_ns) / 1e6)
            if g_distance is not None and g_distance > tol:
                fail(f"全局路径终点偏差 {g_distance:.2f} m > {tol:.2f} m（{g_pts} 点）",
                     "全局 A*", global_ms, g_pts)
                return
            global_ok = True
            push_log("ok", f"全局 A* 成功：{g_pts} 点，耗时 {global_ms:.0f} ms")
            break
        if (gate.get("goal_received") and gate.get("last_goal_accepted") is False
                and gate.get("goal_event_age_s") is not None
                and float(gate.get("goal_event_age_s") or 99) <= 1.0):
            reason = str(gate.get("reason", "goal_rejected"))
            fail(f"Goal Gate 拒绝（{reason}）", "全局 A*", 0.0)
            return
    if not global_ok:
        fail(f"全局 A* 未在 {global_timeout:.0f} s 内生成到目标的路径",
             "全局 A*", global_timeout * 1000.0)
        return

    # Phase 2: local SCAN + CBF candidate path.
    UI["goal_result"].set_text(
        f"全局 A* 成功（{global_ms:.0f} ms）；局部 SCAN/CBF 阶段（≤{local_timeout:.0f} s）…")
    local_ok = False
    local_ms = 0.0
    local_deadline = time.monotonic() + max(3.0, local_timeout)
    while time.monotonic() < local_deadline:
        await asyncio.sleep(0.1)
        with STATE.lock:
            seq = STATE.goal_seq
            l_success_ns = STATE.plan_success_ns
            l_pts = STATE.plan_pts
            l_active = STATE.plan_active
            gate = dict(STATE.goal_status)
        if seq != token:
            UI["goal_result"].set_text("目标已被更新的目标取代；本次等待结束。")
            return
        if l_success_ns and l_success_ns >= goal_ns:
            local_ms = max(0.0, (l_success_ns - goal_ns) / 1e6)
            local_ok = True
            with STATE.lock:
                STATE.goal_failed = False
                STATE.plan_pending = False
                STATE.global_pending = False
            UI["goal_result"].set_text(
                f"规划成功：全局 {g_pts} 点（{global_ms:.0f} ms）+ 局部 {l_pts} 点（{local_ms:.0f} ms）"
                + ("（局部当前有效）" if l_active else "（局部已过期，但本次曾成功）"))
            push_log("ok", f"目标 ({x:.2f}, {y:.2f}) 规划成功：全局 {g_pts} 点/{global_ms:.0f} ms + 局部 {l_pts} 点/{local_ms:.0f} ms")
            ui.notify(f"规划成功：全局 {global_ms:.0f} ms + 局部 {local_ms:.0f} ms", type="positive")
            return
        if (gate.get("goal_received") and gate.get("last_goal_accepted") is False
                and gate.get("goal_event_age_s") is not None
                and float(gate.get("goal_event_age_s") or 99) <= 1.0):
            reason = str(gate.get("reason", "goal_rejected"))
            fail(f"Goal Gate 拒绝（{reason}）", "局部", 0.0)
            return
    fail(f"全局 A* 已成功，但局部 SCAN/CBF 未在 {local_timeout:.0f} s 内生成路径",
         "局部", local_timeout * 1000.0)

async def apply_map_path() -> None:
    path = str(UI["map_path"].value or "").strip()
    if not path or not os.path.isfile(path):
        ui.notify(f"地图文件不存在：{path}", type="negative")
        return
    backend = (UI["slam_backend"].value
               if UI.get("slam_backend") else BACKEND_VENDOR)
    if backend == BACKEND_FAST_LIO:
        if ((PROCS.desk_mode == "fast_lio_realtime" and PROCS.running("desk"))
                or (UI.get("fast_lio_mode")
                    and UI["fast_lio_mode"].value == FAST_MODE_REALTIME)):
            ui.notify("实时模式不加载历史 PCD；本次启动点就是 map 原点",
                      type="info")
            return
        if PROCS.running("desk"):
            ui.notify("FAST-LIO2 参考地图在启动时加载；请先停止后端再更换地图",
                      type="warning")
            return
        UI["nav_pcd"].value = path
        UI["svc_result"].set_text(
            f"FAST-LIO2 定位参考地图已选择：{path}\n启动定位时执行 ICP 加载。")
        ui.notify("FAST-LIO2 参考地图已选择", type="positive")
        return
    with STATE.lock:
        ros_ready = STATE.ros_ready
        loc_up = STATE.loc_node_up
    if not ros_ready:
        UI["svc_result"].set_text("ROS worker 尚未连接；先等顶部显示 ROS ✓ 再试。")
        ui.notify("ROS 未连接", type="negative")
        return
    # The localization desk may still be starting when the user clicks. Wait for
    # the census to see /localization_lite_node instead of echoing raw
    # "Node not found" from ros2 param set.
    for _ in range(5):
        if loc_up:
            break
        await asyncio.sleep(0.4)
        with STATE.lock:
            loc_up = STATE.loc_node_up
    if not loc_up:
        UI["svc_result"].set_text(
            "设置 map_path 失败：/localization_lite_node 尚未上线。\n"
            "请先点“启动定位”，等定位节点出现并进入 Relocalization，"
            "再点“设置并加载”。")
        ui.notify("定位节点未运行：请先启动定位 desk", type="negative")
        return
    UI["svc_result"].set_text(f"设置 map_path={path} …")
    try:
        cp = None
        text = ""
        for attempt in range(3):
            cp = await asyncio.to_thread(
                subprocess.run,
                ["ros2", "param", "set", LOC_NODE, "map_path", path],
                capture_output=True, text=True, timeout=15, check=False)
            text = (cp.stdout or "") + (cp.stderr or "")
            if cp.returncode == 0:
                break
            if "Node not found" not in text:
                break
            # ros2 CLI graph can lag a just-started node; retry a couple times.
            await asyncio.sleep(1.0)
        if cp is None or cp.returncode != 0:
            if "Node not found" in text:
                UI["svc_result"].set_text(
                    "设置 map_path 失败：/localization_lite_node 未发现。\n"
                    "请先启动定位 desk，等节点上线后再点“设置并加载”。\n\n"
                    + text[-1200:])
                ui.notify("定位节点未运行", type="negative")
            else:
                UI["svc_result"].set_text(f"设置 map_path 失败：{text[-2000:]}")
            return
        UI["svc_result"].set_text(text[-1000:] + "\n正在调用 load_map …")
        await srv_click("load_map")
    except Exception as exc:  # noqa: BLE001
        UI["svc_result"].set_text(f"设置 map_path 异常：{type(exc).__name__}: {exc}")


async def run_tf_check() -> None:
    script = os.path.join(NAV_REPO, "scripts", "n01_frame_contract_check.py")
    if not os.path.isfile(script):
        UI["tf_check"].set_text(f"找不到 TF 检查脚本：{script}")
        return
    UI["tf_check"].set_text("正在执行只读 TF 检查…")
    try:
        cp = await asyncio.to_thread(
            subprocess.run, [sys.executable, script], capture_output=True,
            text=True, timeout=30, check=False)
        text = (cp.stdout or "") + (cp.stderr or "")
        UI["tf_check"].set_text(text[-6000:] or f"exit={cp.returncode}")
    except Exception as exc:  # noqa: BLE001
        UI["tf_check"].set_text(f"TF 检查失败：{type(exc).__name__}: {exc}")


def _age_text_for(key: str) -> str:
    age = STATE.age_of(key)
    return "—" if age is None else (f"{age * 1000:.0f} ms" if age < 0.5 else f"{age:.1f} s")


# ---------------------------------------------------------------------------
# Modular console: overall lifecycle -> mapping -> localization -> navigation
# ---------------------------------------------------------------------------
SERVICE_LABELS = {
    "activate_localization": "激活定位",
    "deactivate_localization": "停止定位服务",
    "enter_tracking": "进入跟踪",
    "enter_relocalization": "进入重定位",
    "activate_mapping": "激活建图服务",
    "save_map": "保存地图快照",
    "deactivate_mapping": "结束建图",
    "load_map": "加载地图",
}


def _fresh_color(age):
    if age is None:
        return MUTED
    if age <= 0.5:
        return COL_OK
    if age <= 2.0:
        return COL_WARN
    return COL_ERR


def _fresh_text(age):
    if age is None:
        return "—"
    if age < 0.5:
        return f"{age * 1000:.0f} ms"
    return f"{age:.2f} s"


def _worst_age(*ages) -> float | None:
    """取多个年龄中最老的一个；全部无数据时返回 None。"""
    present = [a for a in ages if a is not None]
    return max(present) if present else None


def _update_fresh(labels) -> None:
    if labels is None:
        return
    # New modular page stores a single label here; older tab builders pass a
    # {topic_key: label} mapping. Accept both instead of crashing the refresh.
    if hasattr(labels, "items"):
        for key, label in labels.items():
            age = STATE.age_of(key)
            _set(label, _fresh_text(age), _fresh_color(age))
        return
    age = STATE.age_of("input_cloud")
    _set(labels, _fresh_text(age), _fresh_color(age))


async def start_full_stack() -> None:
    """Start the normal localization + navigation stack, not mapping."""
    with STATE.lock:
        map_up = STATE.map_node_up
        loc_up = STATE.loc_node_up
    if map_up:
        ui.notify("建图正在运行：建图与定位互斥，请先结束建图", type="warning")
        return
    if not loc_up:
        await lifecycle_start_slam("loc")
        for _ in range(40):
            with STATE.lock:
                loc_up = STATE.loc_node_up
            if loc_up:
                break
            await asyncio.sleep(0.5)
        if not loc_up:
            ui.notify("定位节点未上线，未继续启动导航；请查看顶部错误/日志",
                      type="negative")
            return
    await navigation_action("start")


async def start_slam_rviz(mode: str) -> None:
    """Start the RViz profile matching the selected localization backend."""
    if PROCS.running("rviz"):
        await asyncio.to_thread(PROCS.stop, "rviz")
    display_text, available, _fallback = _display_info()
    if not available:
        push_log("err", f"RViz2 无法启动：{display_text}")
        ui.notify(display_text, type="negative")
        return
    UI["viz_mode"].value = "定位视图" if mode == "loc" else "建图视图"
    backend = (UI["slam_backend"].value
               if UI.get("slam_backend") else BACKEND_VENDOR)
    realtime = bool(
        PROCS.desk_mode == "fast_lio_realtime" and PROCS.running("desk")) or bool(
        backend == BACKEND_FAST_LIO and UI.get("fast_lio_mode")
        and UI["fast_lio_mode"].value == FAST_MODE_REALTIME)
    label = ("FAST-LIO2 实时建图视图" if realtime else
             "FAST-LIO2 重定位视图") if backend == BACKEND_FAST_LIO else (
        "甲方定位视图" if mode == "loc" else "甲方建图视图")
    if not await _confirm(f"将启动 {label}，用于点云检查和手动重定位。确认？"):
        return
    ok = (PROCS.start_fast_lio_rviz(realtime=realtime)
          if backend == BACKEND_FAST_LIO
          else PROCS.start_rviz(mode))
    ui.notify("RViz 已启动" if ok else "RViz 启动失败，请查看运行日志",
              type="positive" if ok else "negative")


async def start_nav_visualization() -> None:
    with STATE.lock:
        names = {r["name"] for r in STATE.node_rows}
    if names & {"/n01_live_navigation_rviz", "/n01_navigation_rviz", "/n01_rviz"}:
        ui.notify("导航启动时已自带 RViz，无需重复启动", type="warning")
        return
    if PROCS.running("nav_rviz"):
        ui.notify("导航 RViz 已在运行", type="warning")
        return
    _text, available, _fallback = _display_info()
    if not available:
        push_log("err", f"导航 RViz 无法启动：{_text}")
        ui.notify(_text, type="negative")
        return
    if await _confirm("将启动独立导航 RViz（n01_full_pipeline.rviz）。确认？"):
        if not PROCS.start_nav_rviz():
            ui.notify("导航 RViz 启动失败；请查看运行日志", type="negative")
            return
        await asyncio.sleep(1.0)
        if not PROCS.running("nav_rviz"):
            msg = ("导航 RViz 启动后立即退出：DISPLAY/X 服务器不可用或无权限。"
                   "请在有桌面的终端设置 DISPLAY/XAUTHORITY，或使用 Foxglove。")
            push_log("err", msg)
            ui.notify(msg, type="negative", timeout=8000)


async def stop_nav_visualization() -> None:
    await asyncio.to_thread(PROCS.stop, "nav_rviz")


async def stop_navigation_module() -> None:
    # Stop the module's standalone RViz first, then the nav lifecycle.
    await asyncio.to_thread(PROCS.stop, "nav_rviz")
    await navigation_action("stop")


async def sync_nav_pcd() -> None:
    with STATE.lock:
        path = str(STATE.loc.get("map_path", "") or "")
    if not path or not os.path.isfile(path):
        ui.notify("定位当前没有可用的 map_path；先加载地图", type="negative")
        return
    if UI.get("nav_pcd"):
        UI["nav_pcd"].value = path
    ui.notify("已同步定位当前地图到导航 PCD；启动导航时生效", type="positive")


async def sync_nav_ground() -> None:
    with STATE.lock:
        pose_z = STATE.pose_z
    if pose_z is not None:
        ground = pose_z + GROUND_OFFSET_FROM_FOOTPRINT
    elif UI.get("ground_z"):
        ground = float(UI["ground_z"].value or 0.139)
    else:
        ui.notify("没有位姿，无法同步当前地面", type="negative")
        return
    if UI.get("nav_ground_z"):
        UI["nav_ground_z"].value = round(float(ground), 3)
    ui.notify(f"已同步当前地面 {ground:+.3f} m；启动导航时生效", type="positive")


def refresh_console() -> None:
    with STATE.lock:
        loc = dict(STATE.loc)
        mapping = dict(STATE.mapping)
        pose = STATE.pose
        pose_z = STATE.pose_z
        goal_status = dict(STATE.goal_status)
        nav_health = dict(STATE.nav_health)
        rows = list(STATE.node_rows)
        dups = list(STATE.dup_groups)
        loc_pts = None if STATE.loc_pts is None else len(STATE.loc_pts)
        cloud_pts = None if STATE.cloud_pts is None else len(STATE.cloud_pts)
        ros_error = STATE.ros_error
    nav_ready = _nav_ready_state()   # 心跳新鲜度门控；需在 STATE.lock 之外调用
    worker_connected = bool(
        WORKER.thread is not None and WORKER.thread.is_alive()
        and WORKER.connected is not None and WORKER.connected.is_set())
    # Worker 卡在 rclpy init / executor 登入超过 20 s 时报错，避免无限“连接中”。
    if (not worker_connected and WORKER.thread is not None
            and WORKER.thread.is_alive() and not STATE.ros_error):
        started = getattr(WORKER, "started_monotonic", 0.0)
        if started and time.monotonic() - started > 20.0:
            with STATE.lock:
                STATE.ros_error = (
                    "ROS Worker 初始化超过 20 秒仍未加入 Executor；"
                    "不会无限等待，也不会启动第二个 Worker。请查看运行日志。")
    if WORKER.thread is not None and not WORKER.thread.is_alive():
        with STATE.lock:
            if not STATE.ros_ready and not STATE.ros_error:
                STATE.ros_error = "ROS Worker 线程已退出，未建立 ROS 连接；请查看运行日志并重新启动 GUI。"
    if worker_connected:
        with STATE.lock:
            if STATE.ros_error:
                STATE.ros_error = ""
    if UI.get("ros_badge"):
        if worker_connected:
            _set(UI["ros_badge"], "ROS ✓", COL_OK)
        elif ros_error:
            _set(UI["ros_badge"], "ROS 未连接", COL_ERR)
        else:
            _set(UI["ros_badge"], "ROS 连接中…", COL_WARN)
    if UI.get("ros_reason"):
        if worker_connected:
            _set(UI["ros_reason"], "executor 已接管 GUI 节点", COL_OK)
        elif ros_error:
            _set(UI["ros_reason"], f"启动失败：{ros_error[:300]}", COL_ERR)
        else:
            _set(UI["ros_reason"], "ROS worker 启动中…", COL_WARN)

    loc_session = str(loc.get("session", "—"))
    map_session = str(mapping.get("session", "—"))
    nav_text = "就绪" if nav_ready is True else (
        "阻塞" if nav_ready is False else "未启动")
    nav_color = COL_OK if nav_ready is True else (
        COL_ERR if nav_ready is False else MUTED)
    loc_running = STATE.loc_node_up or (
        PROCS.desk_mode in ("fast_lio_loc", "fast_lio_realtime")
        and PROCS.running("desk"))
    map_running = STATE.map_node_up or (
        PROCS.desk_mode in ("fast_lio_mapping", "fast_lio_realtime")
        and PROCS.running("desk"))

    if UI.get("gl_loc"):
        _set(UI["gl_loc"], f"{'运行' if loc_running else '停止'} · {loc_session}",
             COL_OK if loc_running else COL_IDLE)
    if UI.get("gl_map"):
        _set(UI["gl_map"], f"{'运行' if map_running else '停止'} · {map_session}",
             COL_OK if map_running else COL_IDLE)
    if UI.get("gl_nav"):
        _set(UI["gl_nav"], nav_text, nav_color)
    if UI.get("gl_bag"):
        if PROCS.running("bag"):
            _set(UI["gl_bag"], f"录制中 · {PROCS.bag_path}", COL_OK)
        elif PROCS.bag_path:
            _set(UI["gl_bag"], f"已停止 · {PROCS.bag_path}", MUTED)
        else:
            _set(UI["gl_bag"], "—", MUTED)
    if UI.get("gl_rviz"):
        rv = []
        if PROCS.running("rviz"):
            rv.append("定位/建图 RViz")
        if PROCS.running("nav_rviz"):
            rv.append("导航 RViz")
        names = {r["name"] for r in rows}
        if names & {"/n01_live_navigation_rviz", "/n01_navigation_rviz", "/n01_rviz"}:
            rv.append("导航自带 RViz")
        _set(UI["gl_rviz"], " / ".join(rv) if rv else "未运行",
             COL_OK if rv else MUTED)

    if UI.get("map_desk"):
        _set(UI["map_desk"], "运行" if map_running else "停止",
             COL_OK if map_running else COL_IDLE)
    if UI.get("map_session"):
        _set(UI["map_session"], map_session)
    if UI.get("map_bag"):
        if PROCS.running("bag"):
            _set(UI["map_bag"], f"录制中 · {PROCS.bag_path}", COL_OK)
        elif PROCS.bag_path:
            _set(UI["map_bag"], f"已停止 · {PROCS.bag_path}", MUTED)
        else:
            _set(UI["map_bag"], "—", MUTED)
    if UI.get("map_dir"):
        _set(UI["map_dir"], str(mapping.get("save_dir", "—")))
    if UI.get("map_pts"):
        _set(UI["map_pts"], f"{cloud_pts or 0} 点")
    slam_rviz = PROCS.running("rviz")
    rviz_view = UI["viz_mode"].value if UI.get("viz_mode") else "—"
    if UI.get("map_rviz"):
        _set(UI["map_rviz"],
             f"运行 · {rviz_view}" if slam_rviz else "停止",
             COL_OK if slam_rviz else MUTED)

    reloc_ok = bool(loc.get("reloc_ok"))
    try:
        fitness = float(loc.get("reloc_fitness"))
    except (TypeError, ValueError):
        fitness = None
    if reloc_ok:
        match_text = "已收敛"
        match_color = COL_OK
    elif fitness is not None:
        match_text = f"未收敛 · fitness={fitness:.3f} / 阈值 0.045"
        match_color = COL_ERR if fitness > 0.045 else COL_WARN
    else:
        match_text = "未收敛"
        match_color = COL_WARN
    if UI.get("loc_match"):
        _set(UI["loc_match"], match_text, match_color)
    if UI.get("loc_pose"):
        if pose is None:
            _set(UI["loc_pose"], "—", MUTED)
        else:
            _set(UI["loc_pose"],
                 f"x={pose[0]:+.2f} y={pose[1]:+.2f} yaw={math.degrees(pose[2]):+.1f}°"
                 f" · bf_z={pose_z if pose_z is None else f'{pose_z:+.2f}'}")
    if UI.get("loc_rviz"):
        _set(UI["loc_rviz"],
             f"运行 · {rviz_view}" if slam_rviz else "停止",
             COL_OK if slam_rviz else MUTED)

    if UI.get("nav_ready"):
        _set(UI["nav_ready"], nav_text, nav_color)
    blockers = nav_health.get("blockers") if isinstance(nav_health, dict) else None
    blockers = blockers if isinstance(blockers, list) else []
    if UI.get("nav_blockers"):
        _set(UI["nav_blockers"],
             "、".join(str(x) for x in blockers[:4]) if blockers else "无",
             COL_ERR if blockers else COL_OK)
    if UI.get("nav_gate"):
        if goal_status and goal_status.get("goal_received", False):
            # Heartbeats always carry accepted=false; the real last result is
            # last_goal_accepted. Using heartbeat.accepted made an accepted goal
            # (reason="accepted") display as "拒绝 · accepted".
            accepted = bool(goal_status.get(
                "last_goal_accepted", goal_status.get("accepted")))
            reason = str(goal_status.get("reason", "—"))
            if accepted and reason in ("accepted", "goal_accepted"):
                reason = "已接受"
            _set(UI["nav_gate"],
                 f"{'接受' if accepted else '拒绝'} · {reason}",
                 COL_OK if accepted else COL_ERR)
        elif goal_status:
            _set(UI["nav_gate"], "等待目标", MUTED)
        else:
            _set(UI["nav_gate"], "—", MUTED)
    if UI.get("nav_pose"):
        if pose is None:
            _set(UI["nav_pose"], "—", MUTED)
        else:
            _set(UI["nav_pose"],
                 f"x={pose[0]:+.2f} y={pose[1]:+.2f} yaw={math.degrees(pose[2]):+.1f}°")
    if UI.get("nav_pcd_match"):
        realtime = bool(
            PROCS.desk_mode == "fast_lio_realtime" and PROCS.running("desk"))
        nav_pcd = str(UI["nav_pcd"].value or "") if UI.get("nav_pcd") else ""
        loc_map = str(loc.get("map_path", "") or "")
        if realtime:
            _set(UI["nav_pcd_match"], "实时地图（无静态 PCD）", COL_OK)
        elif nav_pcd and loc_map:
            try:
                same = os.path.realpath(nav_pcd) == os.path.realpath(loc_map)
            except OSError:
                same = nav_pcd == loc_map
            _set(UI["nav_pcd_match"], "一致" if same else "不一致",
                 COL_OK if same else COL_ERR)
        else:
            _set(UI["nav_pcd_match"], "—", MUTED)
    if UI.get("nav_ground_match"):
        try:
            nav_g = float(UI["nav_ground_z"].value) if UI.get("nav_ground_z") else 0.139
        except (TypeError, ValueError):
            nav_g = None
        loc_g = pose_z + GROUND_OFFSET_FROM_FOOTPRINT if pose_z is not None else None
        if nav_g is None or loc_g is None:
            _set(UI["nav_ground_match"], "—", MUTED)
        else:
            diff = abs(nav_g - loc_g)
            _set(UI["nav_ground_match"],
                 f"一致 ({diff * 1000:.0f} mm)" if diff <= 0.02
                 else f"偏差 {diff * 1000:.0f} mm",
                 COL_OK if diff <= 0.02 else COL_ERR)
    if UI.get("nav_global_plan"):
        with STATE.lock:
            global_pts = STATE.global_pts
            global_active = STATE.global_active
            global_stale = STATE.global_stale
            global_distance = STATE.global_goal_distance
            plan_active = STATE.plan_active
        # 与数据链/地图同一判据（四个条件全查）：global_active 是锁存位，
        # scan_replan_fsm 只发一次且从不清空，必须叠加 plan_active 与两条
        # 路径的存活，否则轨迹清空后 ≤2s 内这里仍会显示绿 N 点，
        # 而同页 nav_plan 已显示“已过期”。
        nav_live, plan_live = _nav_paths_live()
        effective_active = bool(global_active and plan_active and nav_live and plan_live)
        stale_now = bool(global_stale or not effective_active)
        tol = float(os.environ.get("N01_PATH_GOAL_TOL_M", "0.30"))
        endpoint = (f" · 终点偏差 {global_distance:.2f} m"
                    if global_distance is not None else "")
        if global_pts:
            color = COL_ERR if (
                global_distance is not None and global_distance > tol) else (
                COL_OK if effective_active else COL_WARN)
            _set(UI["nav_global_plan"],
                 f"{'最近 ' if stale_now else ''}{global_pts} 点"
                 f"{' · 已过期' if stale_now else ''}{endpoint}",
                 COL_IDLE if global_stale else color)
        else:
            _set(UI["nav_global_plan"], "无全局路径", MUTED)
    if UI.get("nav_plan"):
        with STATE.lock:
            plan_pts = STATE.plan_pts
            plan_active = STATE.plan_active
            plan_stale = STATE.plan_stale
            plan_pending = STATE.plan_pending
        plan_age = STATE.age_of("plan_path")
        if plan_active and plan_pts:
            _set(UI["nav_plan"],
                 f"{plan_pts} 点 · {_fresh_text(plan_age)}",
                 _fresh_color(plan_age))
        elif plan_pts:
            prefix = "等待新路径 · " if plan_pending else ""
            _set(UI["nav_plan"],
                 f"{prefix}最近 {plan_pts} 点 · 已过期" if plan_stale
                 else f"{prefix}最近 {plan_pts} 点",
                 COL_WARN)
        else:
            _set(UI["nav_plan"], "无活动候选", MUTED)
    if UI.get("nav_feasibility"):
        with STATE.lock:
            feas = dict(STATE.feasibility)
        if feas:
            status = str(feas.get("status", feas.get("reason", "—")))
            reason = str(feas.get("reason", ""))
            if status == "WAITING_FOR_BSPLINE":
                text = "等待 B 样条（规划器未生成轨迹）"
            elif status == "STALE_BSPLINE":
                text = "B 样条已过期（规划器不再更新）"
            else:
                text = status + (f" · {reason}" if reason and reason != status else "")
            _set(UI["nav_feasibility"], text,
                 COL_WARN if "WAIT" in status or "STALE" in status else COL_OK)
        else:
            _set(UI["nav_feasibility"], "—", MUTED)
    if UI.get("nav_rviz"):
        nav_rviz = PROCS.running("nav_rviz")
        names = {r["name"] for r in rows}
        nav_self = bool(names & {"/n01_live_navigation_rviz",
                                 "/n01_navigation_rviz", "/n01_rviz"})
        if nav_rviz:
            _set(UI["nav_rviz"], "独立运行", COL_OK)
        elif nav_self:
            _set(UI["nav_rviz"], "导航自带", COL_OK)
        else:
            _set(UI["nav_rviz"], "停止", MUTED)

    if UI.get("diag_topics"):
        pairs = [
            ("/robot_pose", "pose"), ("/loc_status", "loc_status"),
            ("/mapping_status", "mapping_status"),
            ("/localization_map", "loc_map"), ("/cloud_map", "cloud_map"),
            ("/livox/lidar/geo_filtered", "input_cloud"),
            ("/n01/live_navigation/navigation_ready", "nav_ready"),
            ("/n01/live_navigation/goal_gate_status", "goal_status"),
            ("/n01/live_navigation/system_health", "nav_health"),
        ]
        UI["diag_topics"].set_text(
            " · ".join(f"{name}: {_age_text_for(key)}" for name, key in pairs))
    if UI.get("diag_nodes"):
        orphans = [r["name"] for r in rows if "孤儿" in r.get("state", "")]
        UI["diag_nodes"].set_text(
            f"节点 {len(rows)} · 重名 {len(dups)} {dups[:3] if dups else ''}"
            f" · 孤儿 {len(orphans)} {orphans[:3] if orphans else ''}")



# ---------------------------------------------------------------------------
# 数据链路（GUI 只读监测）
# ---------------------------------------------------------------------------
CHAIN_COLUMNS = [
    ("传感器", [
        ("LiDAR", "/livox/lidar", "livox_lidar", "livox_lidar_publisher"),
        ("IMU", "/livox/imu", "livox_imu", "livox_lidar_publisher"),
        ("底盘里程计", "/odom", "odom", "chassis_controller"),
    ]),
    ("预处理 / TF", [
        ("点云预处理", "/livox/lidar/geo_filtered", "input_cloud",
         "lidar_cloud_preprocess_node"),
        ("IMU 互补滤波", "/imu/rpy/filtered", "imu_rpy",
         "complementary_filter_gain_node"),
        ("静态 TF", None, None, "static_tf_publisher_node"),
    ]),
    ("定位", [
        ("PCD 地图", None, "map_loaded", None),
        ("robot_pose", "/robot_pose", "pose", "localization_lite_node"),
        ("loc_status", "/loc_status", "loc_status", "localization_lite_node"),
    ]),
    ("定位适配层", [
        ("n01 pose", "/n01/localization/pose", "n01_pose",
         "n01_localization_adapter"),
        ("n01 status", "/n01/localization/status", "n01_status",
         "n01_localization_adapter"),
    ]),
    ("导航输入桥接", [
        ("cloud", "/cloud", "bridge_cloud", "n01_live_scan_bridge"),
        ("body_pose", "/body_pose", "body_pose", "n01_live_scan_bridge"),
        ("readiness", "/n01/live_navigation/readiness", "readiness",
         "n01_live_scan_bridge"),
        ("静态栅格", "/grid_map/static_occupancy", "nav_static", "n01_static_map"),
        ("动态栅格", "/grid_map/occupancy", "nav_dynamic", "scan_planner_node"),
    ]),
    ("规划", [
        ("全局路径", "/planning/global_path", "global_path", "scan_planner_node"),
        ("局部路径", "/n01/live_navigation/hive2_candidate_path", "plan_path",
         "scan_planner_node"),
    ]),
]


def _node_present(name: str) -> bool:
    if not name:
        return False
    # 普查行用绝对名（/livox_lidar_publisher），CHAIN_COLUMNS 里写的是短名；
    # 不归一化会导致“节点在但无数据”永远判不出来。
    target = name if name.startswith("/") else "/" + name
    with STATE.lock:
        rows = list(STATE.node_rows)
    return any(r.get("name") == target and int(r.get("count", 0)) > 0 for r in rows)


def _publisher_count(topic: str) -> int:
    """当前 DDS 图上该话题的发布者数量（0=无发布者，>1=多源）。"""
    if not topic:
        return 0
    with STATE.lock:
        pub = STATE.topic_pubs.get(topic)
    return 0 if pub is None else len(pub)


_READY_GATES = (
    ("localization_status_fresh", "loc_status 过期"),
    ("robot_pose_receive_fresh", "/robot_pose 过期"),
    ("odom_receive_fresh", "/odom 过期"),
    ("sensor_extrinsic_ready", "TF 外参未就绪"),
    ("cloud_output_fresh", "/cloud 过期"),
    ("body_pose_live_fresh", "/body_pose 过期"),
)


def _readiness_blockers() -> list[str]:
    """桥接 readiness（0.5 s 周期）里第一个失守的上游门控，用于解释桥接行为红色。"""
    age = STATE.age_of("readiness")
    if age is None or age > 2.0:
        return []
    with STATE.lock:
        data = dict(STATE.readiness or {})
    return [txt for k, txt in _READY_GATES if data.get(k) is False]


def _chain_limit(key: str) -> float:
    """每个链条节点的“新鲜”阈值。

    传感器/位姿/桥接是连续流 → 1 s；规划输出按“当前是否处于规划周期”
    自适应：运动中局部路径要求 2 s、全局路径 10 s，空闲时放宽到 300 s，
    避免把一条已就绪的一次性规划结果误标成红色。
    """
    if key == "plan_path":
        return 2.0 if STATE.plan_active else 300.0
    if key == "global_path":
        # /planning/global_path 是一次性锁存结果：scan_replan_fsm 只在
        # plan/replan 事件发布一次（transient_local），执行期间不刷新，
        # 也从不发空 Path。按接收 age 判过期会让它在下发目标约 10 s 后
        # 永久变红。“任务进行中/空闲”交给 _chain_state 用局部路径活动判定。
        return float("inf")
    if key == "readiness":
        return 2.0          # 桥接诊断 0.5 s 周期
    return 1.0


def _nav_paths_live() -> tuple[bool, bool]:
    """“规划路径仍在任务中”的唯一判据（数据链/导航页/地图共用）。

    全局路径是 transient_local 锁存快照，自身 age 无法判过期；
    可靠信号是 nav runtime 心跳 + 局部候选路径是否仍在 4 Hz 回放窗口内。
    进程被 stop/崩溃时两者都会停更，锁存位必须随之失效。
    返回 (全局路径在任务中, 局部路径在任务中)。age_of 自带锁，需在锁外调用。
    """
    nav_age = STATE.age_of("nav_ready")
    plan_age = STATE.age_of("plan_path")
    return (nav_age is not None and nav_age <= 2.0,
            plan_age is not None and plan_age <= 2.0)


def _chain_state(topic, key, node):
    if key == "map_loaded":
        if not STATE.loc.get("map_loaded"):
            return "gray", "未加载"
        status_age = STATE.age_of("loc_status")
        if status_age is None:
            return "red", "已加载 · loc_status 无数据"
        if status_age > 3.0:
            return "red", "已加载 · loc_status 过期"
        return "green", "已加载"
    if topic is None:
        if node and _node_present(node):
            return "green", "已运行"
        return "gray", "未启动"

    age = STATE.age_of(key) if key else None
    # 锁存型一次性结果：/grid_map/static_occupancy 只在启动时发布一次
    # （TRANSIENT_LOCAL），收到即长期有效，不能按 1 s 判过期。
    if key == "nav_static":
        if age is None:
            return "gray", "未收到静态地图"
        return "green", "已加载（锁存）"

    # 全局路径：锁存快照 + 由局部路径活动状态判断“任务是否还在进行”。
    # trajectory_monitor 在轨迹活动期间以 4 Hz 回放候选路径，清空时发一条
    # 空 Path；所以 plan_active/plan_pending 才是可靠的任务进行中信号。
    if key == "global_path":
        if STATE.global_pending or STATE.plan_pending:
            return "gray", "空闲 · 已下发目标，等待规划"
        # 锁存位 + 心跳/局部路径回放同时成立才算“任务中”，与导航页/地图一致，
        # 避免导航栈停止后这里仍绿“已锁存”。
        nav_live, plan_live = _nav_paths_live()
        if (STATE.global_active and STATE.plan_active
                and nav_live and plan_live):
            return "green", (f"已锁存 · 最近 {age:.0f} s 前"
                             if age is not None else "已锁存")
        if age is None:
            return "gray", "空闲 · 无活动目标"
        if STATE.global_stale and age <= 3.0:
            return "gray", "空闲 · 轨迹已清空"
        return "gray", f"空闲 · 最近 {age:.0f} s 前"

    # 局部路径是“一次性结果 + 活动期连续流”的混合体：空闲期没有活动候选，
    # 既不是新鲜、也不是故障，如实显示“空闲”，避免误绿/误红。
    if key == "plan_path":
        if not STATE.plan_active:
            if age is None:
                return "gray", "空闲 · 无活动目标"
            if STATE.plan_pending:
                return "gray", "空闲 · 已下发目标，等待规划"
            if STATE.plan_stale and age <= 3.0:
                # 轨迹监视器清空轨迹时会发一条空 Path，age 因此被刷新；
                # 这里显式说明是“清空”而不是“刚收到新路径”。
                return "gray", "空闲 · 轨迹已清空"
            return "gray", f"空闲 · 最近 {age:.0f} s 前"

    limit = _chain_limit(key) if key else 1.0
    pn = _publisher_count(topic)
    note = ""
    if key in ("bridge_cloud", "body_pose"):
        blockers = _readiness_blockers()
        if blockers:
            note = " · " + "、".join(blockers[:2])
    if age is None:
        if pn:
            return "red", (f"{pn} 个发布者，无数据{note}" if pn > 1
                           else f"发布者在，无数据{note}")
        if node and _node_present(node):
            return "red", f"节点在 · 无发布者{note}"
        return "gray", "未启动"
    if pn > 1:
        return "red", f"多源发布者 {pn} · {age:.1f} s"
    if age <= limit:
        rate = STATE.rate_of(key) if key else 0.0
        txt = f"{age * 1000:.0f} ms" if age < 1.0 else f"{age:.1f} s"
        rate_txt = f"{rate:.0f} Hz · " if rate >= 1.0 else ""
        return "green", f"{rate_txt}{txt}"
    return "red", f"过期 {age:.1f} s{note}"


CHAIN_CSS = """
.chain-wrap{display:flex;align-items:stretch;gap:0;overflow-x:auto;padding:8px 2px;font-size:12px}
.ccol{min-width:170px;flex:1;display:flex;flex-direction:column;gap:7px}
.chead{font-weight:700;color:#cbd5e1;padding:2px 4px}
.cn{border:1px solid #334155;border-left-width:5px;border-radius:7px;padding:6px 8px;background:#0f172a;display:flex;flex-direction:column;gap:2px;color:#e2e8f0}
.cn b{font-size:12px}
.cn code{font-size:10px;color:#94a3b8;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.cn small{font-size:10px;color:#94a3b8}
.cn .dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:5px;background:#64748b;vertical-align:middle}
.cn.green{border-color:#16a34a;border-left-color:#16a34a}
.cn.green .dot{background:#22c55e}
.cn.gray{border-color:#334155;border-left-color:#64748b;opacity:.72}
.cn.red{border-color:#dc2626;border-left-color:#dc2626;background:#1f1114}
.cn.red .dot{background:#ef4444}
.carrow{display:flex;align-items:center;justify-content:center;min-width:28px;font-size:22px;font-weight:700;color:#64748b}
.carrow.green{color:#22c55e}.carrow.gray{color:#64748b}.carrow.red{color:#ef4444}
"""


def _chain_html() -> str:
    columns = []
    for title, nodes in CHAIN_COLUMNS:
        cells = []
        for label, topic, key, node in nodes:
            cls, detail = _chain_state(topic, key, node)
            topic_txt = f"<code>{topic}</code>" if topic else "<code>—</code>"
            cells.append(
                f'<div class="cn {cls}"><span class="dot"></span>'
                f'<b>{label}</b>{topic_txt}<small>{detail}</small></div>')
        columns.append(
            f'<div class="ccol"><div class="chead">{title}</div>'
            + "".join(cells) + "</div>")
    arrows = []
    for idx in range(len(columns) - 1):
        left = CHAIN_COLUMNS[idx][1]
        states = [_chain_state(t, k, n)[0] for _, t, k, n in left]
        color = "green" if all(s == "green" for s in states) else (
            "red" if any(s == "red" for s in states) else "gray")
        arrows.append(f'<div class="carrow {color}">→</div>')
    body = columns[0] + "".join(
        a + c for a, c in zip(arrows, columns[1:]))
    return (
        f'<div class="chain-wrap">{body}</div>'
        '<div style="font-size:11px;color:#94a3b8;margin-top:4px">'
        '绿=当前有数据 · 灰=未启动 · 红=节点在但数据缺失/过期</div>')

def _chain_hint() -> str:
    checks = [
        ("/livox/lidar", "livox_lidar", "启动 Livox 驱动（定位数据链）"),
        ("/livox/imu", "livox_imu", "检查 Livox IMU/驱动"),
        ("/livox/lidar/geo_filtered", "input_cloud", "启动定位 desk（预处理）"),
        ("/imu/rpy/filtered", "imu_rpy", "启动 IMU 互补滤波"),
        ("/odom", "odom", "检查底盘里程计"),
        ("/robot_pose", "pose", "加载 PCD、重定位并进入 tracking"),
        ("/loc_status", "loc_status", "确认定位节点在 tracking"),
        ("/n01/localization/pose", "n01_pose", "启动定位适配层"),
        ("/n01/localization/status", "n01_status", "启动定位适配层"),
        ("/cloud", "bridge_cloud", "启动导航桥接"),
        ("/body_pose", "body_pose", "启动导航桥接并确认 anchor"),
        ("/planning/global_path", "global_path", "启动导航并发送目标"),
        ("/n01/live_navigation/hive2_candidate_path", "plan_path",
         "等待规划器产生局部路径"),
    ]
    if not STATE.loc.get("map_loaded"):
        return "下一步：加载 PCD 地图"
    for topic, key, action in checks:
        # 与链路卡使用同一判据：只有“红”才算断点。规划输出空闲时是灰，
        # 一次性锁存的全局路径也不能按 age 判过期，否则会给出错误的“下一步”。
        cls, detail = _chain_state(topic, key, None)
        if cls == "red":
            return f"下一步：{action}（{topic} {detail}）"
        if STATE.age_of(key) is None and key not in ("plan_path", "global_path"):
            return f"下一步：{action}（{topic} 无数据）"
    return "数据链路完整，可以启动导航。"


_CHAIN_LAST_HTML = ""


def refresh_chain() -> None:
    """实时重算数据链，但只在 HTML 真正变化时才替换 DOM。

    绿色必须每 tick 重新争取（状态来自 age_of 实时计算）；内容不变时
    跳过 innerHTML 替换，避免每 0.5 s 整块重建造成闪烁与 CPU 抖动。
    """
    if not UI.get("chain_html"):
        return
    global _CHAIN_LAST_HTML
    html = _chain_html()
    if html != _CHAIN_LAST_HTML:
        UI["chain_html"].content = html
        _CHAIN_LAST_HTML = html
    hint = _chain_hint()
    if UI["chain_hint"].text != hint:
        UI["chain_hint"].set_text(hint)
    if UI.get("chain_last"):
        UI["chain_last"].set_text("最后刷新 " + time.strftime("%H:%M:%S"))



def refresh_panels() -> None:
    """Refresh localization/mapping/freshness labels that refresh_console does not own."""
    if not UI.get("loc_desk"):
        return
    with STATE.lock:
        loc = dict(STATE.loc)
        mapping = dict(STATE.mapping)
        pose = STATE.pose
        pose_z = STATE.pose_z
        n_loc = 0 if STATE.loc_pts is None else len(STATE.loc_pts)
        n_cloud = 0 if STATE.cloud_pts is None else len(STATE.cloud_pts)
    sess = str(loc.get("session", "—"))
    ml = loc.get("map_loaded")
    ro = loc.get("reloc_ok")
    fit = loc.get("reloc_fitness")
    loc_running = STATE.loc_node_up or (
        PROCS.desk_mode in ("fast_lio_loc", "fast_lio_realtime")
        and PROCS.running("desk"))
    _set(UI["loc_desk"], "运行" if loc_running else "停止",
         COL_OK if loc_running else COL_IDLE)
    _set(UI["loc_session"], f"{_session_cn(sess)} ({sess})", _session_color(sess))
    _set(UI["loc_map_loaded"], "是" if ml else ("否" if ml is False else "—"),
         COL_OK if ml else (COL_ERR if ml is False else MUTED))
    _set(UI["loc_map_path"], str(loc.get("map_path", "—")))
    _set(UI["loc_map_pts"], f"{n_loc:,}")
    pose_age = STATE.age_of("pose")
    _set(UI["loc_pose_age"], "—" if pose_age is None else
         (f"{pose_age * 1000:.0f} ms" if pose_age < 0.5 else f"{pose_age:.1f} s"),
         COL_ERR if pose_age is not None and pose_age > 2.0 else COL_OK)
    _set(UI["loc_reloc"], "是" if ro else ("否" if ro is False else "—"),
         COL_OK if ro else (COL_ERR if ro is False else MUTED))
    if fit is None:
        _set(UI["loc_fitness"], "—", MUTED)
    else:
        ok = float(fit) <= 0.045
        _set(UI["loc_fitness"], f"{float(fit):.5f} {'≤阈值' if ok else '>阈值'}",
             COL_OK if ok else COL_ERR)
    _set(UI["loc_status"],
         f"{sess} · map={'Y' if ml else 'N'} · reloc={'Y' if ro else 'N'}"
         f" · fit={'—' if fit is None else f'{float(fit):.3f}'}")
    _set(UI["loc_ground"],
         "—" if pose_z is None else f"{pose_z + GROUND_OFFSET_FROM_FOOTPRINT:+.2f}")
    _set(UI["l_fresh"],
         f"cloud {_age_text('input_cloud')} · pose {_age_text('pose')}",
         _fresh_color(_worst_age(STATE.age_of("input_cloud"), STATE.age_of("pose"))))
    _set(UI["tf_check"],
         "map→base_footprint 有" if pose_age is not None and pose_age <= 1.0 else "—",
         COL_OK if pose_age is not None and pose_age <= 1.0 else MUTED)
    if UI.get("nav_pcd_match") and UI.get("loc_pcd_match"):
        _set(UI["loc_pcd_match"], UI["nav_pcd_match"].text)
    _set(UI["map_cloud_pts"], f"{n_cloud:,}")
    _set(UI["m_fresh"], _age_text("input_cloud"),
         _fresh_color(STATE.age_of("input_cloud")))
    if UI.get("in_cloud_age"):
        _set(UI["in_cloud_age"], _age_text("input_cloud"),
             _fresh_color(STATE.age_of("input_cloud")))
    _set(UI["n_fresh"], _age_text("body_pose"),
         _fresh_color(STATE.age_of("body_pose")))
    if UI.get("drive_ready"):
        tracking = str(loc.get("session", "")).lower() == "tracking"
        body_age = STATE.age_of("body_pose")
        path_age = STATE.age_of("plan_path")
        # 空 Path（轨迹清空）会刷新 age，单看 age 会把"已清空"当新鲜；
        # 这里要求活动路径 + 2 s 窗口，与 _chain_limit("plan_path") 一致。
        path_ok = bool(STATE.plan_active) and path_age is not None and path_age <= 2.0
        reasons = []
        if not tracking:
            reasons.append("未 tracking")
        if body_age is None or body_age > 0.5:
            reasons.append("/body_pose 过期")
        if not path_ok:
            reasons.append("本地路径未就绪")
        if _nav_ready_state() is not True:
            reasons.append("runtime 未 READY")
        can_drive = not reasons
        _set(UI["drive_ready"],
             "可开始沿路径行驶" if can_drive else "未就绪：" + "、".join(reasons),
             COL_OK if can_drive else COL_WARN)
        if UI.get("drive_button"):
            UI["drive_button"].set_enabled(can_drive)
    if UI.get("slow_status"):
        _set(UI["slow_status"], str(getattr(STATE, "slow_status", "") or "—"))
    refresh_lifecycle()


def build_data_chain_panel() -> None:
    ui.add_head_html(f"<style>{CHAIN_CSS}</style>")
    with _card2("适配器链路"):
        UI["chain_hint"] = ui.label(_chain_hint()).classes(
            "w-full text-sm text-amber-300")
        UI["chain_last"] = ui.label("最后刷新 —").classes(
            "text-xs text-slate-500")
        with ui.expansion("查看完整实时链路", icon="account_tree").classes(
                "w-full compact-expansion"):
            UI["chain_html"] = ui.html(_chain_html(), sanitize=False).classes("w-full")
            ui.button("刷新链路", on_click=refresh_chain).props("flat dense")


def build_console_page() -> None:
    ui.dark_mode().enable()
    ui.add_head_html("""
    <style>
      body { background: #080d16; }
      .compact-card { border: 1px solid #1e293b; border-radius: 10px; }
      .q-tab-panel { padding: 8px !important; }
      .q-field--dense .q-field__control { min-height: 36px !important; }
      .q-btn { letter-spacing: 0; }
      .status-grid { background: #0b1220; border-radius: 8px; padding: 8px; }
      .compact-expansion .q-item { min-height: 34px; padding: 4px 8px; }
    </style>
    """)
    UI["mg_enable"] = ui.checkbox("启用节点管理", value=True).classes("hidden")
    UI["mg_mode"] = ui.select(["定位 desk", "建图 desk"], value="定位 desk").classes("hidden")
    UI["viz_mode"] = ui.select(["定位视图", "建图视图"], value="定位视图").classes("hidden")

    with ui.header().classes("items-center justify-between bg-slate-950 px-4 py-2"):
        with ui.column().classes("gap-0"):
            ui.label("N-01 机器人总控制台").classes(
                "text-lg font-bold text-white")
            ui.label("建图定位后端 → 标准适配器 → SCAN 导航") \
                .classes("text-xs text-slate-400")
        with ui.column().classes("items-end gap-0"):
            with ui.row().classes("items-center gap-2"):
                ui.button("RViz", icon="visibility", on_click=start_selected_rviz) \
                    .props("outline dense color=secondary")
                ui.button(icon="visibility_off", on_click=stop_rviz) \
                    .props("flat round dense color=secondary") \
                    .tooltip("停止 RViz")
                UI["ros_badge"] = ui.label("ROS 未连接").classes("text-sm font-bold")
                _lbl(f"domain={os.environ.get('ROS_DOMAIN_ID', '0')} · {ROSK}",
                     "text-xs text-slate-500")
            UI["ros_reason"] = ui.label("ROS worker 启动中…").classes(
                "text-xs text-slate-400 max-w-[720px] truncate")

    with ui.column().classes("w-full max-w-[1800px] mx-auto p-3 gap-3"):
        # ---------------- top: shared map ----------------
        with _card2("共享地图视图（投影栅格 / 目标选取）"):
            with ui.row().classes("items-center gap-3 w-full flex-wrap"):
                UI["src"] = ui.radio(
                    ["自动", "定位地图", "建图", "导航栅格"], value="自动",
                    on_change=lambda _: draw_grid(force=True)).props("inline dense")
                UI["range"] = ui.select(
                    ["10m", "20m", "40m"], value="20m", label="视野",
                    on_change=lambda _: draw_grid(force=True)) \
                    .classes("w-24").props("dense")
                UI["ground_z"] = ui.number(
                    "地面 z", value=0.139, format="%.2f",
                    on_change=lambda _: draw_grid(force=True)) \
                    .classes("w-28").props("dense")
                UI["trail_on"] = ui.checkbox(
                    "轨迹", value=True, on_change=lambda _: draw_grid(force=True))
            UI["grid_info"] = ui.label("等待地图数据…").classes(
                "text-xs text-slate-400")
            UI["map_image"] = ui.interactive_image(
                _BLANK_PNG, on_mouse=on_map_click, events=["click"]) \
                .classes("w-full max-w-[580px] max-h-[52vh] mx-auto rounded border border-slate-800")
            _lbl("点击地图会把 map 坐标填入“导航”页目标 x/y。",
                 "text-xs text-slate-500")

        # ---------------- middle: tabs ----------------
        with ui.tabs().classes("w-full bg-slate-900 rounded-lg").props(
                "dense active-color=cyan-4 indicator-color=cyan-4") as tabs:
            ui.tab("总控 / 适配器")
            ui.tab("建图")
            ui.tab("定位")
            ui.tab("导航")
            ui.tab("节点管理")
        with ui.tab_panels(tabs, value="总控 / 适配器").classes("w-full"):
            with ui.tab_panel("总控 / 适配器"):
                with _card2("整体项目生命周期（总览）"):
                    with ui.row().classes("w-full items-end gap-3 flex-wrap"):
                        UI["slam_backend"] = ui.select(
                            BACKEND_OPTIONS, value=BACKEND_VENDOR,
                            label="建图定位后端") \
                            .classes("w-72").props("outlined dense")
                        UI["fast_lio_mode"] = ui.select(
                            [FAST_MODE_OFFLINE, FAST_MODE_REALTIME],
                            value=FAST_MODE_OFFLINE, label="FAST-LIO2 模式") \
                            .classes("w-72").props("outlined dense")
                        UI["slam_task"] = ui.select(
                            ["定位", "建图"], value="定位", label="任务") \
                            .classes("w-32").props("outlined dense")
                        _lbl("两个后端统一输出 /robot_pose、/loc_status，"
                             "再由 N-01 Adapter 转为导航输入；FAST-LIO2 实时模式"
                             "忽略任务选项，直接启动建图定位一体链。",
                             "text-xs text-slate-400 pb-2")
                    with ui.grid(columns=5).classes("w-full gap-2"):
                        UI["gl_loc"] = _kv("定位进程")
                        UI["gl_map"] = _kv("建图进程")
                        UI["gl_nav"] = _kv("导航")
                        UI["gl_bag"] = _kv("录包")
                        UI["gl_rviz"] = _kv("RViz")
                    with ui.row().classes("items-center gap-2 flex-wrap"):
                        ui.button("启动建图定位", on_click=start_selected_slam_task) \
                            .props("color=positive")
                        ui.button("停止建图定位", on_click=stop_desk) \
                            .props("color=negative")
                        ui.button("启动导航", on_click=lambda: navigation_action("start")) \
                            .props("color=primary")
                        ui.button("停止导航", on_click=lambda: navigation_action("stop")) \
                            .props("outline color=primary")
                        ui.button("停止全部（保存建图）", on_click=stop_managed_all) \
                            .props("color=negative")
                        ui.button("刷新状态", on_click=refresh_console).props("flat dense")
                        UI["mg_skip_rsp"] = ui.checkbox(
                            "使用地盘 RSP（关闭 SLAM 自带 RSP）", value=True)
                    UI["svc_result"] = ui.label("最近操作：—").classes(
                        "w-full text-xs font-mono whitespace-pre-wrap break-all")
                build_data_chain_panel()

            with ui.tab_panel("建图"):
                with _card2("① 建图模块（建图 + bag 录制）"):
                    with ui.row().classes("w-full gap-4 items-start"):
                        with ui.column().classes("w-full lg:w-[430px] gap-2"):
                            UI["map_save_dir"] = ui.input(
                                "甲方 SLAM3D Lite 保存目录",
                                value="/home/demo/lyy/slam_data/maps") \
                                .classes("w-full")
                            UI["fast_lio_map"] = ui.input(
                                "FAST-LIO2 输出 PCD", value=DEFAULT_FAST_LIO_MAP) \
                                .classes("w-full")
                            _lbl("在“总控 / 适配器”选择后端与“建图”任务后统一启动；"
                                 "停止建图定位时自动保存并结束 bag。",
                                 "text-xs text-slate-400")
                        with ui.column().classes("w-full grow gap-2"):
                            with ui.grid(columns=4).classes("w-full gap-x-3 gap-y-1 status-grid"):
                                UI["life_map"] = _kv("生命周期")
                                UI["map_session"] = _kv("会话")
                                UI["map_bag"] = _kv("录包")
                                UI["map_cloud_pts"] = _kv("地图点数")
                            with ui.expansion("详细状态", icon="tune").classes(
                                    "w-full compact-expansion"):
                                with ui.grid(columns=4).classes("w-full gap-x-3 gap-y-1"):
                                    UI["map_desk"] = _kv("建图进程")
                                    UI["map_dir"] = _kv("保存目录")
                                    UI["map_pts"] = _kv("累计点云")
                                    UI["map_rviz"] = _kv("RViz")
                                    UI["m_fresh"] = _kv("建图输入")
                                    UI["in_cloud_age"] = _kv("点云年龄")

            with ui.tab_panel("定位"):
                with _card2("② 定位模块（定位 + 适配 + 导航桥接）"):
                    with ui.row().classes("w-full gap-4 items-start"):
                        with ui.column().classes("w-full lg:w-[470px] gap-2"):
                            _lbl("流程：启动定位 → 顶栏 RViz 手动选初始位姿 → 等待匹配 → "
                                 "进入跟踪 → 数据链绿后启动导航",
                                 "text-xs text-amber-300")
                            UI["map_path"] = ui.input(
                                "定位参考 PCD",
                                value=(DEFAULT_MAP_PATH if os.path.isfile(DEFAULT_MAP_PATH) else "")) \
                                .classes("w-full")
                            _lbl("① 在“总控 / 适配器”选择后端与“定位”任务并启动。",
                                 "text-xs text-slate-400")
                            with ui.row().classes("gap-2 flex-wrap"):
                                ui.button("② 选择/加载地图", on_click=apply_map_path) \
                                    .props("outline dense")
                                ui.button("③ 开始重定位", on_click=begin_relocalization) \
                                    .props("outline dense")
                            with ui.row().classes("gap-2 items-center flex-wrap"):
                                UI["ip_x"] = ui.number("x", value=0.0, format="%.2f").classes("w-24")
                                UI["ip_y"] = ui.number("y", value=0.0, format="%.2f").classes("w-24")
                                UI["ip_yaw"] = ui.number("yaw", value=0.0, format="%.2f").classes("w-24")
                                UI["ip_frame"] = ui.input("frame", value="map").classes("w-24")
                                ui.button("④ 发送初始位姿", on_click=send_initialpose) \
                                    .props("outline dense")
                            with ui.row().classes("gap-2 flex-wrap"):
                                ui.button("⑤ 进入跟踪", on_click=request_enter_tracking) \
                                    .props("outline dense")
                            _lbl("初始位姿：重定位阶段需要发布以初始化搜索；进入 tracking 后不需要。",
                                 "text-xs text-slate-500")
                        with ui.column().classes("w-full grow gap-2"):
                            with ui.grid(columns=4).classes("w-full gap-x-3 gap-y-1 status-grid"):
                                UI["life_loc"] = _kv("生命周期")
                                UI["loc_session"] = _kv("定位会话")
                                UI["loc_match"] = _kv("匹配状态")
                                UI["loc_pose"] = _kv("当前位姿")
                                UI["l_fresh"] = _kv("数据新鲜度")
                                UI["loc_rviz"] = _kv("RViz")
                            UI["loc_track_result"] = _kv("跟踪结果")
                            with ui.expansion("高级定位状态", icon="tune").classes(
                                    "w-full compact-expansion"):
                                with ui.grid(columns=4).classes("w-full gap-x-3 gap-y-1"):
                                    UI["loc_desk"] = _kv("定位进程")
                                    UI["loc_map_loaded"] = _kv("地图加载")
                                    UI["loc_reloc"] = _kv("重定位")
                                    UI["loc_fitness"] = _kv("fitness")
                                    UI["loc_pose_age"] = _kv("位姿年龄")
                                    UI["loc_map_pts"] = _kv("地图点数")
                                    UI["loc_ground"] = _kv("地面 z")
                                    UI["tf_check"] = _kv("TF")
                                    UI["loc_pcd_match"] = _kv("PCD 一致性")
                                    UI["loc_status"] = _kv("loc_status")
                                    UI["ip_result"] = _kv("初始位姿")
                                    UI["loc_svc_result"] = _kv("服务结果")
                                UI["loc_map_path"] = _kv("地图路径")

            with ui.tab_panel("导航"):
                with _card2("③ 导航模块（只规划，不直接驱动底盘）"):
                    with ui.row().classes("w-full gap-4 items-start"):
                        with ui.column().classes("w-full lg:w-[430px] gap-2"):
                            UI["nav_pcd"] = ui.input(
                                "导航 PCD",
                                value=(DEFAULT_MAP_PATH
                                       if os.path.isfile(DEFAULT_MAP_PATH) else ""),
                            ).classes("w-full")
                            UI["nav_ground_z"] = ui.number(
                                "导航地面 z", value=0.139, format="%.2f").classes("w-28")
                            with ui.row().classes("gap-2 flex-wrap"):
                                ui.button("同步 PCD", on_click=sync_nav_pcd) \
                                    .props("outline dense")
                                ui.button("同步地面", on_click=sync_nav_ground) \
                                    .props("outline dense")
                            with ui.row().classes("gap-2 items-center flex-wrap"):
                                UI["goal_x"] = ui.number("目标 x", value=0.0, format="%.2f").classes("w-28")
                                UI["goal_y"] = ui.number("目标 y", value=0.0, format="%.2f").classes("w-28")
                                UI["goal_yaw"] = ui.number("目标 yaw", value=0.0, format="%.2f").classes("w-28")
                                ui.button("发送导航目标", on_click=send_goal) \
                                    .props("color=primary")
                            _lbl("规划成功后，确认条件就绪再点“开始沿路径行驶”；不会自动动。",
                                 "text-xs text-amber-300")
                            UI["drive_ready"] = _kv("行驶条件")
                            with ui.row().classes("gap-2 items-center flex-wrap"):
                                UI["slow_speed"] = ui.number(
                                    "慢速 m/s", value=0.05, format="%.3f",
                                    min=0.01, max=0.08).classes("w-28")
                                UI["drive_button"] = ui.button(
                                    "开始沿路径行驶（慢速）",
                                    on_click=start_slow_motion) \
                                    .props("color=negative")
                                ui.button("停止行驶（零速）", on_click=stop_slow_motion) \
                                    .props("outline dense")
                            UI["slow_status"] = _kv("行驶状态")
                            UI["goal_result"] = _kv("目标结果")
                        with ui.column().classes("w-full grow gap-2"):
                            with ui.grid(columns=4).classes("w-full gap-x-3 gap-y-1 status-grid"):
                                UI["life_nav"] = _kv("生命周期")
                                UI["nav_ready"] = _kv("导航就绪")
                                UI["nav_gate"] = _kv("目标门控")
                                UI["nav_global_plan"] = _kv("导航路径（动态更新）")
                                UI["nav_feasibility"] = _kv("可行性")
                            UI["nav_blockers"] = _kv("阻塞项")
                            UI["nav_start_summary"] = ui.label("导航生命周期：尚未操作") \
                                .classes("text-xs whitespace-pre-wrap break-all w-full")
                            with ui.expansion("高级导航状态", icon="tune").classes(
                                    "w-full compact-expansion"):
                                UI["nav_plan"] = _kv("内部局部规划状态")
                                with ui.grid(columns=4).classes("w-full gap-x-3 gap-y-1"):
                                    UI["nav_pose"] = _kv("导航位姿")
                                    UI["n_fresh"] = _kv("导航输入")
                                    UI["nav_rviz"] = _kv("导航 RViz")
                                    UI["nav_pcd_match"] = _kv("PCD 一致性")
                                    UI["nav_ground_match"] = _kv("地面一致性")
                                UI["nav_result"] = _kv("导航操作")

            with ui.tab_panel("节点管理"):
                with _card2("系统节点管理"):
                    _lbl("按功能查看当前 ROS 图；分类不代表进程所有权。"
                         "话题随发布节点生灭，TF 允许多个节点发布不同坐标边。",
                         "text-xs text-slate-400")
                    with ui.row().classes("w-full gap-2 items-center"):
                        UI["node_category"] = ui.select(
                            ["全部", *NODE_CATEGORIES], value="全部", label="功能分类",
                            on_change=lambda _: refresh_health()).props("dense outlined").classes("w-52")
                        UI["node_search"] = ui.input(
                            "搜索节点 / 进程 / 关键话题", on_change=lambda _: refresh_health()) \
                            .props("dense outlined clearable").classes("grow")
                        ui.button("刷新节点", icon="refresh", on_click=refresh_census_now).props("flat dense")
                    UI["node_category_summary"] = ui.label("等待节点普查…").classes("text-xs text-slate-400")
                    with ui.grid(columns=4).classes("w-full gap-x-3 gap-y-1 status-grid"):
                        UI["health_count"] = _kv("节点数")
                        UI["health_dup"] = _kv("重复节点")
                        UI["proc_nodes"] = _kv("进程节点")
                        UI["proc_procs"] = _kv("进程")
                    UI["dup_banner"] = _kv("重复提示")
                    UI["health_nodes"] = ui.table(columns=[
                        {"name": key, "label": label, "field": key, "align": "left", "sortable": True}
                        for key, label in (
                            ("category", "分类"), ("name", "节点"), ("role", "功能"),
                            ("count", "同名数"), ("state", "状态"),
                            ("owner", "进程/归属"), ("management", "管理入口"),
                            ("topics", "关键发布话题"))], rows=[], row_key="name", pagination=10) \
                        .props("dense flat bordered wrap-cells").classes("w-full")
                    UI["node_detail_select"] = ui.select(
                        [], label="选择节点查看完整接口 / 填入停止计划", on_change=select_node_detail) \
                        .props("dense outlined clearable use-input").classes("w-full")
                    UI["node_detail"] = ui.label("选择节点查看发布/订阅接口。") \
                        .classes("text-xs font-mono whitespace-pre-wrap max-h-52 overflow-auto w-full")
                    with ui.expansion("关键话题发布者", icon="hub").classes("w-full compact-expansion"):
                        UI["health_topics"] = ui.table(columns=[
                            {"name": key, "label": label, "field": key, "align": "left"}
                            for key, label in (("topic", "话题"), ("n", "发布节点数"),
                                               ("state", "状态"), ("pubs", "发布节点"))],
                            rows=[], row_key="topic", pagination=10).props("dense flat wrap-cells").classes("w-full")
                    with ui.expansion("生命周期规则", icon="info").classes("w-full compact-expansion"):
                        _lbl("定位/建图：互斥启动，检查前置链和残留；只停止 GUI 登记的进程组。"
                             "导航：停止受管旧导航并复查，再启动；独立定位与驱动保留。"
                             "雷达：已有唯一正确发布者时复用；重复/类型冲突阻止再启动。"
                             "其他外部节点：核对归属后通过原启动入口管理。",
                             "text-xs text-slate-400")
                    with ui.expansion("节点停止工具", icon="power_settings_new").classes(
                            "w-full compact-expansion"):
                        UI["orphan_result"] = _kv("孤儿进程")
                        UI["kill_target"] = ui.input("停止节点名").classes("w-full")
                        UI["kill_result"] = _kv("停止结果")
                        with ui.row().classes("gap-2"):
                            ui.button("查看停止计划", on_click=show_kill_plan) \
                                .props("outline dense")
                            ui.button("执行停止", on_click=kill_selected) \
                                .props("color=negative dense")
                    with ui.expansion("完整诊断信息", icon="monitor_heart").classes(
                            "w-full compact-expansion"):
                        UI["diag_topics"] = _kv("话题")
                        UI["diag_nodes"] = _kv("节点")
                        UI["ds_table"] = _kv("数据源")
                        with ui.element("div").classes("hidden"):
                            UI["chart_info"] = _kv("图表信息")
                            UI["chart"] = _kv("图表")
                            UI["life_slam_note"] = _kv("生命周期说明")
                            UI["viz_ws"] = _kv("工作空间")
                        with ui.row().classes("gap-2"):
                            ui.button("启动 Foxglove 桥", on_click=start_bridge).props("outline dense")
                            ui.button("停止 Foxglove 桥", on_click=stop_bridge).props("outline dense")
                        UI["viz_bridge_status"] = _kv("Foxglove 桥")
                        UI["viz_rviz_status"] = _kv("RViz 状态")

    with ui.footer().classes("bg-slate-950 px-4 py-1"):
        with ui.row().classes("w-full items-center gap-3"):
            ui.label("运行日志").classes("text-xs text-slate-400 shrink-0")
            UI["log"] = ui.log(max_lines=300).classes("w-full max-h-32 text-xs")

def build_page() -> None:
    build_console_page()


_SHUTDOWN_DONE = False


def _shutdown_app() -> None:
    global _SHUTDOWN_DONE, _PRESERVE_SHUTDOWN_SAVE_FAIL
    if _SHUTDOWN_DONE:
        return
    _SHUTDOWN_DONE = True
    with STATE.lock:
        mode = PROCS.desk_mode
        session = str(STATE.mapping.get("session", "")).lower()
    if mode == "mapping" and session == "mapping":
        # 与交互路径 _close_mapping_session 对齐：同样是 60 s 保存超时。
        ok, text = PROCS.deactivate_mapping(timeout=60)
        push_log("ok" if ok else "err",
                 "退出前建图 PCD 已保存" if ok else
                 "退出前建图 PCD 保存失败：" + text[-1200:])
        if not ok and PROCS.running("desk"):
            # 保存失败/超时且 desk 仍在运行：服务端可能还在写盘。保留 bag 和 desk，
            # 并置位标志禁止 atexit 兜底把它们杀掉；只清理与保存无关的进程。
            # 注意：desk 由看门狗 desk_guard 监护（GUI 退出 0.5 s 内会 SIGINT 整组），
            # 仅靠置位本标志不够——必须写一个保留标记让看门狗在 GUI 退出后跳过清理，
            # 否则“保留”会在 GUI 进程消失的瞬间被看门狗打掉。
            _PRESERVE_SHUTDOWN_SAVE_FAIL = True
            desk_pgid = None
            try:
                with PROCS.lock:
                    item = PROCS.procs.get("desk") or {}
                    item_proc = item.get("proc")
                    if item_proc is not None:
                        try:
                            desk_pgid = os.getpgid(item_proc.pid)
                        except (ProcessLookupError, OSError):
                            desk_pgid = item.get("pgid")
            except (AttributeError, OSError):
                desk_pgid = None
            try:
                with open(f"/tmp/n01_desk_preserve_{os.getpid()}.flag", "w",
                          encoding="utf-8") as fh:
                    fh.write(f"pgid={desk_pgid}\n保存前 GUI 退出，desk 组保留待重试")
            except OSError:
                pass
            PROCS.stop("slow_motion")
            PROCS.stop("rviz")
            PROCS.stop("nav_rviz")
            PROCS.stop("foxglove")
            nav_ok, nav_out = LIFECYCLE.shutdown_navigation_only()
            push_log("ok" if nav_ok else "warn",
                     "导航栈已停止（建图 desk/bag 保留）" if nav_ok else
                     "导航栈停止有告警：" + nav_out[-1200:])
            pgid_txt = f"进程组 pgid={desk_pgid}；" if desk_pgid else ""
            push_log("warn", "建图 desk 已保留（看门狗只对标记内 pgid 跳过清理）。" + pgid_txt +
                             "bag 看门狗会发 SIGINT 让 ros2 bag 正常收尾（metadata 不丢）。"
                             "请回持久终端手动重试 deactivate_mapping，"
                             "确认 PCD 完整后手动停止该进程组")
            WORKER.stop(timeout=8.0)
            return
    PROCS.stop("slow_motion")
    PROCS.stop("bag")
    ok, output = LIFECYCLE.shutdown()
    push_log("ok" if ok else "err",
             "统一生命周期退出清理完成" if ok else
             "统一生命周期退出清理失败：" + output[-2000:])
    WORKER.stop(timeout=8.0)


def main() -> None:
    # 清理上次运行可能残留的保留标记（PID 复用时会误触发看门狗跳过清理）
    try:
        os.remove(f"/tmp/n01_desk_preserve_{os.getpid()}.flag")
    except OSError:
        pass
    atexit.register(_shutdown_app)
    app.on_shutdown(_shutdown_app)
    WORKER.start()
    build_page()
    ui.timer(0.5, refresh_console)
    ui.timer(0.5, refresh_panels)
    ui.timer(0.5, refresh_chain)
    ui.timer(1.0, refresh_health)
    ui.timer(2.0, lambda: draw_grid())
    ui.timer(0.3, drain_logs)
    host = os.environ.get("SLAM_GUI_HOST", "0.0.0.0")
    port = int(os.environ.get("SLAM_GUI_PORT", "8123"))
    ament = os.environ.get("AMENT_PREFIX_PATH", "")
    src_ok = _ENV_OK and _has_rosk_prefix()
    push_log("info" if src_ok else "warn",
             f"环境检查: 新树={_has_rosk_prefix()} "
             f"lite消息可用={_ENV_OK}（每次新 shell 都要 source {ROSK}/setup_runtime.sh）")
    push_log("info", f"GUI 启动: http://{host}:{port}  (Ctrl+C 退出)")
    ui.run(host=host, port=port, title="N-01 SLAM + Navigation 控制台", show=False,
           reload=False, dark=True, uvicorn_logging_level="warning")


if __name__ == "__main__":
    main()
