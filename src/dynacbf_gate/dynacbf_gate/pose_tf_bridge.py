#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dynacbf_gate/pose_tf_bridge —— 把 /robot_pose 话题补成 map→base_footprint 的 TF
                                    （M1 下同时搬运甲方 /tf_static 静态链进 /tf）

为什么需要它
============
甲方 slam3d_node 的 /localization_status 报 registration_output_tf_published = true，
但 2026-10-12 现场实测：**新起的 tf2 订阅者**（tf2_echo / view_frames / 任何新启动
的节点）都无法解析 map → base_footprint / map → base_link，TF 树裂成两棵互不连通：

    {map → sliding_map}                          ← 乙方 dynacbf_planner_node 发
    {base_footprint → base_link → 腿/雷达/IMU}    ← 甲方 robot_state_publisher 发

两者之间没有任何变换（tf2_echo 报 "Tf has two or more unconnected trees"）。
后果：controller_node（要 map→base_link 才能发布 /curr_state）、gate_node 的降级
判据、local_map 的 updateTF() 全部拿不到 map 系位姿。

本节点订阅 /robot_pose（nav_msgs/Odometry，header.frame_id=map、
child_frame_id=base_footprint），把它原样广播成 TF，使 TF 树重新连通。
**纯乙方节点**：不读也不改甲方代码/进程，只在 /tf 上补一条边；
一旦甲方侧 TF 恢复，本节点可关掉（见下方"冲突保护"）。

M1 模式（nav_frame=odom）的静态边搬运
====================================
乙方所有 TF 消费者（local_map.cpp / controller.py / gate_node.py）都只订阅 `/tf`，
**不订阅甲方的 `/tf_static`**——于是甲方的静态链 `base_footprint→base_link→lidar_link`
（TRANSIENT_LOCAL，只发一次）根本进不了乙方 buffer，M1 下 odom→lidar_link 必断。
本桥在 M1 时把这些静态边「搬」进 /tf：订阅 /tf_static，抽出 base_footprint→base_link、
base_link→lidar_link，连同 odom→base_footprint 一起广播。数值与帧名全是甲方数据，
乙方不凭空，只是统一走它唯一消费的 /tf 通道，整链 odom→base_footprint→base_link→
lidar_link 在乙方 buffer 里一次成型。

冲突保护
========
1) 启动后先静默监听 /tf 共 detect_sec 秒；若发现"别人已经在发" parent→child，
   本节点自动不启用并 WARN（避免双发布）。
2) 运行中若观测到非本节点发出的同名变换达到阈值，WARN 提醒运维：
   甲方侧 TF 已恢复，建议把 enable 置 false 或从 launch 移除。
3) enable 参数置 false 可整体停用（只发一次状态，不广播）。

参数
====
  enable          默认 true    —— 总开关
  pose_topic      默认 /robot_pose
  parent_frame    默认 map     —— 要补的父帧
  child_frame     默认 base_footprint
  detect_sec       默认 3.0    —— 启动静默检测窗口
  pose_timeout_sec 默认 2.0    —— 位姿停更告警门限
  republish_static 默认 true   —— 把 /tf_static 里的 base_footprint→base_link 与
                                  base_link→lidar_link 搬进 /tf（乙方只收 /tf）
"""
import time

import rclpy
from rclpy.node import Node

import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


class PoseTfBridge(Node):
    def __init__(self):
        super().__init__("pose_tf_bridge")

        self.declare_parameter("enable", True)
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "base_footprint")
        self.declare_parameter("detect_sec", 3.0)
        self.declare_parameter("pose_timeout_sec", 2.0)
        self.declare_parameter("republish_static", True)

        self.enable = bool(self.get_parameter("enable").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.parent = str(self.get_parameter("parent_frame").value)
        self.child = str(self.get_parameter("child_frame").value)
        self.detect_sec = float(self.get_parameter("detect_sec").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        self.republish_static = bool(self.get_parameter("republish_static").value)
        # 只搬运这些静态边（甲方 RSP /tf_static 提供，乙方 buffer 不订阅 /tf_static）
        self._static_parents = set()
        self._static_seen_stamps = {}

        self._start_mono = time.monotonic()
        # detect_sec<=0 或未启用时，跳过静默检测窗口
        self._detect_done = (not self.enable) or (self.detect_sec <= 0.0)
        self._external_seen = 0
        self._warned_external = False
        self._warned_mismatch = False
        self._stamps_sent = []
        self._last_pose_mono = 0.0
        self._published = 0

        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_sub = self.create_subscription(TFMessage, "/tf", self._cb_tf, 100)
        if self.republish_static:
            # 甲方 robot_state_publisher 的 /tf_static 已取证为
            # RELIABLE + TRANSIENT_LOCAL；显式匹配该 QoS，接收启动前缓存的静态链。
            static_qos = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            # 乙方所有 TF 消费者只订阅 /tf，不订阅 /tf_static → 甲方静态边进不了
            # 乙方 buffer。这里订阅甲方 /tf_static，把需要的静态边搬进 /tf。
            self.tf_static_sub = self.create_subscription(
                TFMessage, "/tf_static", self._cb_tf_static, static_qos)
        self.pose_sub = self.create_subscription(
            Odometry, self.pose_topic, self._cb_pose, 10)
        self.status_pub = self.create_publisher(Bool, "~/active", 1)
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            "pose_tf_bridge：%s -> %s  <- %s（enable=%s detect_sec=%.1f）"
            % (self.parent, self.child, self.pose_topic, self.enable, self.detect_sec))
        if not self.enable:
            self.get_logger().warn("enable=false，本节点不广播，仅发布 ~/active 状态。")

    # ------------------------------------------------------------------ /tf
    def _cb_tf(self, msg: TFMessage):
        """监听别人是否已经在发 parent->child，用于启动检测与运行期告警。"""
        for tr in msg.transforms:
            if tr.header.frame_id != self.parent or tr.child_frame_id != self.child:
                continue
            stamp = (tr.header.stamp.sec, tr.header.stamp.nanosec)
            # ① 自己转发 /tf_static 的边时，header.stamp 会被改成"当前时间"，
            #    但消息内容与 static 完全相同 → 用 _static_seen 判断是否自广播，
            #    防止把"自己搬运的静态链"误判为外部冲突而错误 disable。
            if stamp in self._stamps_sent:
                continue          # 是本节点自己广播的位姿边
            self._external_seen += 1
            if (not self._detect_done) and self.enable:
                self._detect_done = True
                self.enable = False
                self.get_logger().warn(
                    "/tf 上已存在 %s -> %s（非本节点发布），"
                    "pose_tf_bridge 自动不启用，避免双发布。"
                    % (self.parent, self.child))
                return

    # ------------------------------------------------------------ /tf_static
    def _cb_tf_static(self, msg: TFMessage):
        """把甲方 RSP 的静态链（base_footprint→base_link→lidar_link）搬进 /tf。

        乙方 buffer 只订阅 /tf，不订阅 /tf_static；甲方这些静态边是 TRANSIENT_LOCAL
        且早于乙方节点发布，乙方永远收不到。这里抽出需要的边，以当前时间戳
        广播到 /tf（数值/帧名全部来自甲方，乙方不凭空），使乙方程 buffer 内
        odom→base_footprint→base_link→lidar_link 一次成型。
        """
        if not (self.enable and self.republish_static):
            return
        for tr in msg.transforms:
            p = tr.header.frame_id
            c = tr.child_frame_id
            # 只搬甲方实际情况存在的链边（基座/雷达/IMU），不搬腿上的动态关节。
            if (p == "base_footprint" and c == "base_link") or \
               ((p == "base_link") and c in ("lidar_link", "lpms_imu_link")):
                if (p, c) in self._static_parents:
                    continue          # 已搬过，每边一次即可
                out = TransformStamped()
                out.header.stamp = self.get_clock().now().to_msg()
                out.header.frame_id = p
                out.child_frame_id = c
                out.transform = tr.transform
                self.broadcaster.sendTransform(out)
                self._static_parents.add((p, c))
                self._static_seen_stamps[(p, c)] = (
                    out.header.stamp.sec, out.header.stamp.nanosec)
                self.get_logger().info(
                    "搬运甲方静态边 %s -> %s 到 /tf（乙方 buffer 不订阅 /tf_static）"
                    % (p, c))

    # -------------------------------------------------------------- /robot_pose
    def _cb_pose(self, msg: Odometry):
        self._last_pose_mono = time.monotonic()
        if not self.enable:
            return

        if not self._detect_done:
            if time.monotonic() - self._start_mono < self.detect_sec:
                return                      # 静默检测窗口内
            self._detect_done = True
            self.get_logger().info(
                "%.1fs 内 /tf 上没有 %s -> %s，pose_tf_bridge 开始广播（补 TF 断链）。"
                % (self.detect_sec, self.parent, self.child))

        if (msg.header.frame_id and msg.header.frame_id != self.parent) or \
           (msg.child_frame_id and msg.child_frame_id != self.child):
            if not self._warned_mismatch:
                self._warned_mismatch = True
                self.get_logger().warn(
                    "%s 的 frame 与参数不一致：msg=(%s -> %s) param=(%s -> %s)；"
                    "以参数为准广播。"
                    % (self.pose_topic, msg.header.frame_id, msg.child_frame_id,
                       self.parent, self.child))

        tr = TransformStamped()
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            tr.header.stamp = self.get_clock().now().to_msg()
        else:
            tr.header.stamp = msg.header.stamp
        tr.header.frame_id = self.parent
        tr.child_frame_id = self.child
        tr.transform.translation.x = msg.pose.pose.position.x
        tr.transform.translation.y = msg.pose.pose.position.y
        tr.transform.translation.z = msg.pose.pose.position.z
        tr.transform.rotation.x = msg.pose.pose.orientation.x
        tr.transform.rotation.y = msg.pose.pose.orientation.y
        tr.transform.rotation.z = msg.pose.pose.orientation.z
        tr.transform.rotation.w = msg.pose.pose.orientation.w

        self.broadcaster.sendTransform(tr)
        self._published += 1
        self._stamps_sent.append((tr.header.stamp.sec, tr.header.stamp.nanosec))
        if len(self._stamps_sent) > 200:
            del self._stamps_sent[:100]

    # ------------------------------------------------------------------- 状态
    def _tick(self):
        self.status_pub.publish(Bool(data=bool(self.enable)))
        if not self.enable:
            return

        elapsed = time.monotonic() - self._start_mono
        if self._last_pose_mono == 0.0:
            # 【重要】从未收到过位姿 —— 这是最容易误判"节点正常"的盲区：
            # 只发 active=true 却一条都没广播过。必须明确报出来。
            if elapsed > (self.detect_sec + self.pose_timeout):
                self.get_logger().warn(
                    "%s 上从未收到任何消息（已等 %.1fs）—— TF %s -> %s 无法广播；"
                    "请先确认甲方 SLAM 是否在发布该话题（话题可能有端点但无数据）。"
                    % (self.pose_topic, elapsed, self.parent, self.child))
                self._last_pose_mono = float("inf")     # 只报一次
        elif (time.monotonic() - self._last_pose_mono) > self.pose_timeout:
            self.get_logger().warn(
                "%.1fs 未收到 %s，TF %s -> %s 已停更（累计广播 %d 次）。"
                % (self.pose_timeout, self.pose_topic, self.parent, self.child,
                   self._published))
            self._last_pose_mono = 0.0      # 只报一次，收到新位姿后重置

        if self._external_seen >= 5 and not self._warned_external:
            self._warned_external = True
            self.get_logger().warn(
                "/tf 上出现了非本节点广播的 %s -> %s（%d 次）—— 甲方侧 TF 可能已恢复，"
                "建议把 pose_tf_bridge 的 enable 置 false 或从 launch 移除。"
                % (self.parent, self.child, self._external_seen))


def main(args=None):
    rclpy.init(args=args)
    node = PoseTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
