#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dynacbf_gate/viz_tf_prefixer —— 乙方独立 Hive2 显示链的 TF 桥

背景
====
Humble 的 robot_state_publisher 3.0.3 不支持 frame_prefix（Iron/Jazzy 才引入），
但我们又要生成不带 `viz_` 前缀、又不与甲方同名 frame（base_footprint/base_link/
lidar_link 等）冲突的 TF。解法（用户批准的路径 A）：

  1) 乙方 RSP 把 `/tf`、`/tf_static` **remap 到独立话题 /tf_viz**，发布标准
     frame（base_footprint/base_link/fl_hip...，模型根=base_footprint）；
  2) 本节点订阅 /tf_viz，把每条 transform 的 frame_id 与 child_frame_id
     统一加前缀，再转发到公共 /tf —— 生成 viz_base_footprint/viz_base_link 等；
  3) 本节点同时订阅甲方 /odom（child_frame_id=base_link），按「odom→base_link
     位姿 × base_link→base_footprint(URDF 固定边反向)」算出 odom→base_footprint，
     加前缀后广播为 odom→viz_base_footprint —— 使模型树与 SCAN 的 odom 系连通；
  4) RViz RobotModel 的 TF Prefix 设为与 prefix 一致（默认 viz_），Fixed Frame=
     odom，即可渲染完整 Hive2 STL 模型且与 /local_pcd 同屏。

数据流
======
  乙方 RSP(remap→/tf_viz) ──base_footprint/base_link/腿...──▶ 前缀转发 → /tf(viz_*)
  甲方 /odom(child=base_link) ────────────────▶ odom桥 → odom→viz_base_footprint → /tf

关键：odom→viz_base_footprint 的 z 不是凭空设为 0/0.90，而是
  odom→base_link(/odom 位姿) × base_link→base_footprint(0,0,-base_footprint_z)。
base_footprint_z 默认 0.90，来自 hive2_body.xacro 的 footprint_xyz（模型原始值）。

参数
====
  in_tf / in_tf_static        默认 /tf_viz / /tf_viz_static（乙方 RSP remap 输入）
  prefix                       默认 viz_（加到乙方 frame_id 前；RViz TF Prefix 同值）
  out_tf / out_tf_static       默认 /tf / /tf_static（公共出口，RViz 默认订阅）
  enable_odom_root             默认 true（生成 odom→viz_base_footprint 使模型挂到 odom）
  odom_topic                   默认 /odom
  odom_child_frame             默认 base_link（甲方 /odom 的 child_frame_id）
  base_footprint_z             默认 0.90（URDF base_footprint→base_link 的 z，模型原始值）
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class VizTfPrefixer(Node):
    def __init__(self):
        super().__init__("viz_tf_prefixer")
        self.declare_parameter("in_tf", "/tf_viz")
        self.declare_parameter("in_tf_static", "/tf_viz_static")
        self.declare_parameter("prefix", "viz_")
        self.declare_parameter("out_tf", "/tf")
        self.declare_parameter("out_tf_static", "/tf_static")
        self.declare_parameter("enable_odom_root", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_child_frame", "base_link")
        self.declare_parameter("base_footprint_z", 0.90)

        self.in_tf = str(self.get_parameter("in_tf").value)
        self.in_tf_static = str(self.get_parameter("in_tf_static").value)
        self.prefix = str(self.get_parameter("prefix").value)
        self.out_tf = str(self.get_parameter("out_tf").value)
        self.out_tf_static = str(self.get_parameter("out_tf_static").value)
        self.enable_odom_root = bool(self.get_parameter("enable_odom_root").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_child = str(self.get_parameter("odom_child_frame").value)
        self.footprint_z = float(self.get_parameter("base_footprint_z").value)

        # 输入：乙方 RSP remap 的独立通道。静态用 TRANSIENT_LOCAL 匹配 RSP。
        self.tf_sub = self.create_subscription(
            TFMessage, self.in_tf, self._cb_dynamic, 100)
        self.tf_static_sub = self.create_subscription(
            TFMessage, self.in_tf_static, self._cb_static,
            QoSProfile(depth=100,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.tf_pub = self.create_publisher(TFMessage, self.out_tf, 100)
        self.tf_static_pub = self.create_publisher(
            TFMessage, self.out_tf_static,
            QoSProfile(depth=100,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._sent_dyn = 0
        self._sent_static = 0
        self._odom_bcast = 0
        self._warned_child = False

        if self.enable_odom_root:
            self.odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self._cb_odom, 10)
            self.get_logger().info(
                "odom 根桥启用：%s(child=%s, base_footprint_z=%.3f) → %s%s"
                % (self.odom_topic, self.odom_child, self.footprint_z,
                   self.prefix, "base_footprint"))
        else:
            self.get_logger().warn("enable_odom_root=false：不生成 odom 根边，"
                                   "模型只在自身 frame 内显示。")

        self.get_logger().info(
            "viz_tf_prefixer：%s/%s ─prefix='%s'→ %s/%s"
            % (self.in_tf, self.in_tf_static, self.prefix,
               self.out_tf, self.out_tf_static))

    # --------------------------------------------------------------- 工具
    def _prefix(self, tr: TransformStamped) -> TransformStamped:
        out = TransformStamped()
        out.header.stamp = tr.header.stamp
        out.header.frame_id = self.prefix + tr.header.frame_id
        out.child_frame_id = self.prefix + tr.child_frame_id
        out.transform = tr.transform
        return out

    def _publish_dynamic(self, transforms):
        if transforms:
            msg = TFMessage(transforms=transforms)
            self.tf_pub.publish(msg)
            self._sent_dyn += len(transforms)
            if self._sent_dyn == len(transforms):
                self.get_logger().info(
                    "开始转发动态 TF（首条 %s -> %s）"
                    % (transforms[0].header.frame_id,
                       transforms[0].child_frame_id))

    # ---------------------------------------------------------- 动态转发
    def _cb_dynamic(self, msg: TFMessage):
        self._publish_dynamic([self._prefix(t) for t in msg.transforms])

    # ---------------------------------------------------------- 静态转发
    def _cb_static(self, msg: TFMessage):
        if not msg.transforms:
            return
        out = TFMessage(transforms=[self._prefix(t) for t in msg.transforms])
        self.tf_static_pub.publish(out)
        self._sent_static += len(out.transforms)
        self.get_logger().info(
            "转发静态 TF %d 条（首条 %s -> %s）"
            % (len(out.transforms),
               out.transforms[0].header.frame_id,
               out.transforms[0].child_frame_id))

    # ------------------------------------------------------------ odom 根桥
    def _cb_odom(self, msg: Odometry):
        if msg.child_frame_id and msg.child_frame_id != self.odom_child:
            # 需求：#3 若 child_frame_id 与预期不一致，拒绝发布该帧并报告，
            # 而不是警告后继续计算 —— 防止把甲方坐标系数值挂错到 odom 下。
            self.get_logger().error(
                "%s 的 child_frame_id=%s 与预期 %s 不一致，拒绝生成 odom 根边。"
                "请核对甲方 /odom 的 child_frame_id 后再启动显示链。"
                % (self.odom_topic, msg.child_frame_id, self.odom_child))
            return
        # T(odom→base_link) = /odom 位姿；base_link→base_footprint = (0,0,-footprint_z)
        # T(odom→base_footprint) = odom→base_link × base_link→base_footprint
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # 四元数 (w,x,y,z) → 旋转矩阵第三列 [r02 r12 r22]，乘 (0,0,-h) 得到 odom 系偏移
        # r02=2(xz+wy)  r12=2(yz-wx)  r22=1-2(x²+y²)  （标准 XYZ 约定）
        w, x, y, z = q.w, q.x, q.y, q.z
        h = self.footprint_z
        off_x = 2 * (x * z + w * y) * (-h)
        off_y = 2 * (y * z - w * x) * (-h)
        off_z = (1 - 2 * (x * x + y * y)) * (-h)

        tr = TransformStamped()
        tr.header.stamp = (msg.header.stamp
                           if msg.header.stamp.sec or msg.header.stamp.nanosec
                           else self.get_clock().now().to_msg())
        tr.header.frame_id = "odom"
        tr.child_frame_id = self.prefix + "base_footprint"
        tr.transform.translation.x = p.x + off_x
        tr.transform.translation.y = p.y + off_y
        tr.transform.translation.z = p.z + off_z
        tr.transform.rotation.x = q.x
        tr.transform.rotation.y = q.y
        tr.transform.rotation.z = q.z
        tr.transform.rotation.w = q.w
        self._publish_dynamic([tr])
        self._odom_bcast += 1
        if self._odom_bcast == 1:
            self.get_logger().info(
                "odom 根桥已广播首个 %s -> %s (%.3f, %.3f, %.3f)"
                % (tr.header.frame_id, tr.child_frame_id,
                   tr.transform.translation.x, tr.transform.translation.y,
                   tr.transform.translation.z))


def main(args=None):
    rclpy.init(args=args)
    node = VizTfPrefixer()
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
