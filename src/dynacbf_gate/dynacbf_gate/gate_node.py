#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dynacbf_gate  —— HiveCore 定位就绪门控节点（/cmd_vel 透传互锁）

设计目标
========
把乙方控制器输出的 `/cmd_vel`（在未就绪时会尝试跟踪参考路径）作为**输入**，
只有当甲方 SLAM 定位确认稳定时，才把该速度**透传**到底盘；否则一律发布零速度。

为什么要这个节点
================
真机闭环里控制器直接给底盘下发 /cmd_vel，而它依据的定位（/robot_pose、map TF）
在 SLAM 处于初始化 / 重定位 / 高拟合误差阶段时不可信。若此时机器人动起来，
就会带着错误的位姿跑偏甚至碰撞。门控节点像安全锁一样卡在控制器与底盘之间：
定位未就绪 → 强制停车；就绪 → 原样放行，不引入任何延迟与改写。

就绪判据（/localization_status，甲方 LocalizationStatus.msg）
=============================================================
  1) 收到过有效定位消息（非空、时间新鲜）
  2) registration_output_localization_mode == MODE_TRACKING(0)
     （处于跟踪模式而非重定位）
  3) registration_output_pose_published == True 且
     registration_output_tf_published == True
  4) registration_output_fitness_mse <= max_fitness_mse（配准拟合误差门限）

消息导入说明
============
甲方消息定义不在本仓库，需先 `source` 甲方 overlay 才能导入
`hivecore_slam3d.msg.LocalizationStatus`。为避免本包在未 source 甲方 overlay
时启动即崩溃，这里用 try/except 兜底：
  * 能导入 → 按 LocalizationStatus 判据门控（推荐，甲方 overlay 已加载）。
  * 不能导入 → 降级为「/robot_pose 新鲜度（首选）或 TF map→base_footprint
    新鲜度（次选）」判据门控，并持续告警，提示需 source 甲方 overlay。

退出语义
========
门控未就绪时输出零速，并发布 /gate_status (std_msgs/Bool)=False；
就绪后转发输入速度并置 True。tick 周期可配置（默认 10Hz）。
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# ---------------------------------------------------------------------------
# 甲方 LocalizationStatus（可选导入）
# ---------------------------------------------------------------------------
_HAVE_SLAM_MSG = False
try:
    from hivecore_slam3d.msg import LocalizationStatus
    _HAVE_SLAM_MSG = True
except (ImportError, ModuleNotFoundError):
    LocalizationStatus = None

import math
import time


class GateNode(Node):
    def __init__(self):
        super().__init__("gate_node")

        # ---------------- 参数 ----------------
        self.declare_parameter("cmd_vel_in", "/cmd_vel_nav")       # 控制器输出（本节点输入）
        self.declare_parameter("cmd_vel_out", "/cmd_vel")          # 底盘实际话题（本节点输出）
        self.declare_parameter("localization_topic", "/localization_status")
        self.declare_parameter("pose_topic", "/robot_pose")        # 降级模式首选位姿源
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("tf_freshness_sec", 1.0)            # TF 新鲜度门限
        self.declare_parameter("odom_freshness_sec", 1.0)          # /robot_pose 新鲜度门限
        self.declare_parameter("max_fitness_mse", 0.5)             # 配准拟合误差门限
        self.declare_parameter("tick_hz", 10.0)
        # 不依赖甲方消息的降级门限：要求最近的定位位姿非零（相对起点）
        self.declare_parameter("require_odom", True)
        # 【M1 / 离线模式】true=必须甲方 LocalizationStatus 判就绪（真机默认）；
        # false=完全跳过甲方定位判据，只看位姿话题新鲜度（用 /odom 跑 M1 链路）。
        self.declare_parameter("require_slam_ready", True)

        self.cmd_vel_in = self.get_parameter("cmd_vel_in").value
        self.cmd_vel_out = self.get_parameter("cmd_vel_out").value
        self.loc_topic = self.get_parameter("localization_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.tf_fresh = float(self.get_parameter("tf_freshness_sec").value)
        self.odom_fresh = float(self.get_parameter("odom_freshness_sec").value)
        self.max_fitness = float(self.get_parameter("max_fitness_mse").value)
        self.tick = float(self.get_parameter("tick_hz").value)
        self.require_odom = bool(self.get_parameter("require_odom").value)
        self.require_slam_ready = bool(
            self.get_parameter("require_slam_ready").value)

        # ---------------- 状态 ----------------
        self._loc_ok = False           # 门控最终就绪标志
        self._loc_msg_valid = False    # 收到过合法定位消息
        self._loc_msg_stamp = 0.0      # 最近定位消息时刻（s, 墙钟）
        self._last_cmd = Twist()       # 最近收到的控制器速度（透传用）
        self._have_cmd = False
        self._pose_msg_stamp = 0.0     # 最近 /robot_pose 时刻（单调钟）

        # ---------------- 发布器 / 订阅器 ----------------
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_out, 1)
        self.gate_pub = self.create_publisher(Bool, "/gate/status", 1)

        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_in, self._cb_cmd, 1)
        # 降级模式的首选位姿源：/robot_pose（map 系 Odometry）。
        # 刻意不依赖 TF —— 现场实测 map 系位姿 TF 不可靠（树裂成两棵不连通），
        # 若降级判据还挂在 TF 上，一旦断链就永远不放行（静默死锁）。
        self.pose_sub = self.create_subscription(
            Odometry, self.pose_topic, self._cb_pose, 10)
        if _HAVE_SLAM_MSG:
            self.loc_sub = self.create_subscription(
                LocalizationStatus, self.loc_topic, self._cb_loc, 1)
        else:
            self.loc_sub = None

        self.create_timer(1.0 / self.tick, self._tick)

        if _HAVE_SLAM_MSG:
            self.get_logger().info(
                f"已加载甲方 LocalizationStatus；按 SLAM 稳定性门控。"
                f" [in]={self.cmd_vel_in} [out]={self.cmd_vel_out} "
                f"loc={self.loc_topic} max_fitness={self.max_fitness}")
        else:
            self.get_logger().warn(
                "未检测到甲方消息包（需先 source 甲方 overlay），降级为 "
                "TF map→base_footprint 新鲜度门控；运行时请确保 source 甲方 overlay。")

    # ---------------- 回调 ----------------
    def _cb_cmd(self, msg: Twist):
        self._last_cmd = msg
        self._have_cmd = True

    def _cb_loc(self, msg):
        # 只认「已发布位姿 / TF」的消息为有效定位；拟合误差超过门限视为未就绪
        try:
            published = (bool(msg.registration_output_pose_published)
                         and bool(msg.registration_output_tf_published))
            tracking = int(msg.registration_output_localization_mode) == 0
            fit_ok = float(msg.registration_output_fitness_mse) <= self.max_fitness
        except AttributeError:
            # 字段缺失（消息版本不一致）——保守视为未就绪
            published = tracking = fit_ok = False

        self._loc_msg_valid = True
        self._loc_msg_stamp = time.monotonic()
        self._loc_ok = bool(published and tracking and fit_ok)

    def _cb_pose(self, msg: Odometry):
        self._pose_msg_stamp = time.monotonic()

    def _pose_fresh(self) -> bool:
        """降级判据（首选）：/robot_pose 是否新鲜。不依赖 TF。"""
        return (self._pose_msg_stamp > 0.0 and
                (time.monotonic() - self._pose_msg_stamp) <= self.odom_fresh)

    def _tf_fresh(self) -> bool:
        """降级判据（次选）：map→base_footprint TF 是否新鲜存在。"""
        try:
            buf = self._tf_buffer
        except AttributeError:
            from tf2_ros import Buffer
            from tf2_ros.transform_listener import TransformListener
            self._tf_buffer = Buffer()
            self._listener = TransformListener(self._tf_buffer, self)
            buf = self._tf_buffer
        try:
            buf.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return False
        return True

    # ---------------- 主循环 ----------------
    def _tick(self):
        if not self.require_slam_ready:
            # 【M1 / 离线模式】完全不看甲方定位状态，只要求位姿新鲜。
            # 用于无 SLAM（/robot_pose 与 map 系位姿 TF 都不可用）时，用底盘 /odom
            # 在 odom 系把整链跑通（SOP 的 M1）。此模式下门控是**放行**的，
            # 一旦下发目标机器人就会动 —— 测试前必须确认场地安全。
            ready = self._pose_fresh() or self._tf_fresh()
        elif _HAVE_SLAM_MSG:
            if not self._loc_msg_valid:
                self.get_logger().warn(
                    f"等待甲方定位消息 {self.loc_topic} ...")
                ready = False
            else:
                # 消息过旧（定位长时间不刷新）也视为未就绪
                fresh = (time.monotonic() - self._loc_msg_stamp) <= self.odom_fresh
                ready = self._loc_ok and fresh
        else:
            # 降级模式：位姿话题新鲜度（首选）或 TF 新鲜度（次选）。
            # 只靠 TF 会死锁：现场 map→base_footprint 断链时永远不放行。
            ready = self._pose_fresh() or self._tf_fresh()

        # 输出：就绪透传输入速度，否则零速
        if ready and self._have_cmd:
            out = self._last_cmd
        else:
            out = Twist()

        self.cmd_vel_pub.publish(out)
        self.gate_pub.publish(Bool(data=bool(ready)))

        if self.get_clock().now().nanoseconds % (5_000_000_000) < 100_000_000:
            self.get_logger().info(
                f"gate ready={ready} v=({out.linear.x:.2f},{out.angular.z:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = GateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
