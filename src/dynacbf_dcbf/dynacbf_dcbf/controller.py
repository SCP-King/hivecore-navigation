# =============================================================================
# 【来源】本文件**逐字复制**自参考实现（另一套已在本机验证跑通的 ROS 2 复现）的
#         local_planner/local_planner/controller.py（全文 125 行）。
#         逻辑、话题名、消息类型、参数、类名（Controller）、节点名（'controller'）
#         一律未改。
#
# 【改动清单】相对参考实现，本文件只做了：
#   改动 1：在本文件顶部增加这段来源/改动说明注释块。
#   改动 2：无（连 import 都不需要调整：本文件没有引用参考实现的其它模块）。
#   改动 3（2026-10-12，真机联调）：TF 读取加固。
#     a) tf2 Buffer 显式给 cache_time 上界（tf_cache_sec，默认 1.0s）。
#        原因：Humble 的 Python 版 tf2_ros.Buffer() 无参构造默认缓存**无限**，
#        位姿 TF 一旦断供，lookup_transform(..., Time()) 会**永远返回旧位姿**，
#        表面 50Hz 正常、实则位置是冻结的 —— 静默失效，极难排查（已实测复现）。
#     b) lookup 失败不再静默 pass，改为按计数节流告警，暴露 TF 断链。
#   改动 4（2026-10-12，顿挫修复）：控制序列按时间索引推进。
#     背景：local_planner 的 /local_plan 里装的是**整条 N 拍控制序列**
#     （N×3 个数，N = MPC_N = 25，步长 MPC_step_size = 0.25s → 6.25s 前瞻），
#     但本文件原先只取 data[:3]（第 0 拍），其余 24 拍全部丢弃；叠加 MPC
#     单次求解 0.4–7.1s 远大于 command_timeout(0.15s)，于是表现为
#     「动 0.15s → 停几秒」的顿挫（实测：100% 的更新间隔 > 0.15s）。
#     改法：存下整条序列，按 k = floor(age / MPC_step_size) 依次取拍；失效
#     判据从「0.15s」改为「序列耗尽（N×step）+ plan_hold_extra 余量」，并在
#     余量内对末拍线性衰减到零，避免瞬时急停。
#     use_plan_sequence=False 可回退到原「只取第 0 拍 + command_timeout」行为。
#   改动 5（2026-10-13，M1 数据流重构）：状态源改为直接订阅甲方 /odom。
#     背景：乙方不再发布任何 odom→base_footprint / odom→base_link TF；
#     参考实现要求的 map→base_link 是甲方 SLAM 全局系，M1（无 SLAM）没有。
#     改法：/curr_state 直接从甲方 /odom 的 position + quaternion(yaw) 填充，
#     不再创建 tf2 Buffer/TransformListener。global_frame/base_frame/tf_cache_sec
#     参数保留声明仅为兼容 launch 传入，已不再使用。
#
# 运行前提（M1 数据源）：
#   * 甲方 /odom 持续发布（现场 ~50Hz RELIABLE），其 frame_id 为 odom 系、
#     child_frame_id 为 base_footprint；controller 用其 position/quaternion 直接
#     生成 /curr_state=[x, y, yaw]。local_planner_node 只依赖 /curr_state，
#     故 /odom 一到，整条规划链即动作。
#   * 订阅 /local_plan、/cmd_move，发布 /cmd_vel、/curr_state（均为根命名空间下的
#     绝对名；local_planner_node 发布的是相对名 "local_plan"/"cmd_move"，在根命名
#     空间下与之一致，故不需要任何 remap）。local_plan 的每组三元组为 [vx, vy, omega]。
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import math
import threading
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('command_timeout', 0.15)
        self.declare_parameter('tf_cache_sec', 1.0)
        # 【改动 5】位姿源话题（Odometry）：M1=/odom（甲方底盘里程计，child=base_link）；
        # 真机=/robot_pose（甲方 SLAM map 系）。launch 的 frame_override 已统一传入。
        self.declare_parameter('odom_topic', '/odom')
        # 【改动 4】控制序列推进参数
        self.declare_parameter('use_plan_sequence', True)
        self.declare_parameter('MPC_step_size', 0.25)  # 与 dcbf_params.yaml 同源
        self.declare_parameter('plan_hold_extra', 0.5)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.25)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.use_plan_sequence = bool(self.get_parameter('use_plan_sequence').value)
        self.mpc_step_size = float(self.get_parameter('MPC_step_size').value)
        self.plan_hold_extra = float(self.get_parameter('plan_hold_extra').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.rate = self.create_timer(0.02, self.control_loop)  # 50Hz
        self.get_state = self.create_timer(0.02, self.get_current_state)  # 50Hz
        # 【改动 5】位姿源话题参数化。launch 通过 frame_override["odom_topic"]=pose_topic
        # 传入：M1=/odom，真机=/robot_pose。其余消息传递与原实现一致。
        self.odom_topic = self.get_parameter('odom_topic').value

        self.local_plan_sub = self.create_subscription(
            Float32MultiArray,
            '/local_plan',
            self.local_planner_cb,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            10)
        self.cmd_move_sub = self.create_subscription(
            Bool,
            '/cmd_move',
            self.cmd_move_cb,
            10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.curr_state_pub = self.create_publisher(Float32MultiArray, '/curr_state', 10)

        # 【改动 5】M1 直接订阅甲方 /odom，不再创建 tf2 Buffer/TransformListener。
        # 状态源只有 /odom（甲方 RSP 发布，~50Hz RELIABLE），不依赖任何乙方
        # 补发的 odom→base_footprint / odom→base_link TF。
        self.odom_pose = None
        self.odom_lock = threading.Lock()
        self.state_fail_count = 0

        self.linear_speed = self.lateral_speed = self.angular_speed = 0.0
        self.motion_enabled = False
        self.last_plan_time = None
        # The controller executes the first [vx, vy, omega] MPC input.
        self.local_plan = np.zeros(3)
        # 【改动 4】整条控制序列与接收时刻（按时间索引推进用）。
        self.plan_seq = None        # shape (N, 3)；None = 尚无序列
        self.plan_recv_time = None

    def quart_to_rpy(self, x, y, z, w):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        sin_pitch = max(-1.0, min(1.0, 2*(w*y-z*x)))
        p = math.asin(sin_pitch)
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return r, p, y
    
    def get_current_state(self):
        with self.odom_lock:
            pose = self.odom_pose
        if pose is None:
            self.state_fail_count += 1
            if self.state_fail_count == 1 or self.state_fail_count % 200 == 0:
                self.get_logger().warn(
                    '/odom 尚未收到有效位姿；/curr_state 暂停发布'
                    '（累计 %d 次），下游 local_planner_node 将不动作。'
                    % self.state_fail_count)
            return

        position, orientation = pose
        _, _, yaw = self.quart_to_rpy(
            orientation.x, orientation.y, orientation.z, orientation.w)
        curr_state = Float32MultiArray()
        curr_state.data = [position.x, position.y, yaw]
        self.curr_state_pub.publish(curr_state)
        self.state_fail_count = 0

    def odom_cb(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        values = (position.x, position.y, position.z,
                  orientation.x, orientation.y, orientation.z, orientation.w)
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().warning('Rejected /odom with NaN or Inf pose')
            return
        with self.odom_lock:
            self.odom_pose = (position, orientation)
    
    def pub_vel(self):
        control_cmd = Twist()
        control_cmd.linear.x = self.linear_speed
        control_cmd.linear.y = self.lateral_speed
        control_cmd.linear.z = 0.0
        control_cmd.angular.x = 0.0
        control_cmd.angular.y = 0.0
        control_cmd.angular.z = self.angular_speed
        # self.get_logger().info('Linear Speed: %.1f, Angular Speed: %.1f' % (self.linear_speed, self.angular_speed))
        self.vel_pub.publish(control_cmd)

    def _pick_cmd(self):
        """【改动 4】按时间索引取当前应执行的拍。

        返回 np.ndarray([vx, vy, omega]) 或 None（None = 无有效指令，应清零）。
        序列用尽后在 plan_hold_extra 内对末拍线性衰减到零，避免瞬时急停。
        """
        if self.plan_recv_time is None:
            return None
        age = (self.get_clock().now() - self.plan_recv_time).nanoseconds * 1e-9

        if self.use_plan_sequence and self.plan_seq is not None:
            n = len(self.plan_seq)
            if n == 0:
                return None
            horizon = n * self.mpc_step_size
            if age < horizon:
                k = int(age / self.mpc_step_size)
                k = min(max(k, 0), n - 1)
                return self.plan_seq[k]
            over = age - horizon
            if over < self.plan_hold_extra:
                fade = max(0.0, 1.0 - over / self.plan_hold_extra)
                return self.plan_seq[-1] * fade
            return None

        # 回退分支：原行为（只取第 0 拍 + command_timeout）。
        if age < self.command_timeout:
            return self.local_plan
        return None

    def control_loop(self):
        cmd = self._pick_cmd() if self.motion_enabled else None
        if cmd is not None:
            linear = np.asarray(cmd[:2], dtype=float)
            linear_norm = float(np.linalg.norm(linear))
            if linear_norm > self.max_linear_speed and linear_norm > 0.0:
                linear *= self.max_linear_speed / linear_norm
            self.linear_speed = float(linear[0])
            self.lateral_speed = float(linear[1])
            self.angular_speed = float(np.clip(
                cmd[2], -self.max_angular_speed, self.max_angular_speed))
        else:
            self.linear_speed = 0.0
            self.lateral_speed = 0.0
            self.angular_speed = 0.0
        self.pub_vel()

    def cmd_move_cb(self, msg):
        self.motion_enabled = msg.data
        if not self.motion_enabled:
            self.local_plan.fill(0.0)

    def local_planner_cb(self, msg):
        if len(msg.data) < 3:
            self.get_logger().warning(
                'Rejected local plan with %d values; expected at least 3 [vx, vy, omega]' %
                len(msg.data))
            return
        if not all(math.isfinite(value) for value in msg.data[:3]):
            self.get_logger().warning('Rejected local plan containing NaN or Inf')
            return
        self.local_plan[:] = msg.data[:3]
        # 【改动 4】整条序列入队：data 是扁平化的 n×3，按 MPC_step_size 时间推进。
        if self.use_plan_sequence:
            n = len(msg.data) // 3
            seq = np.asarray(msg.data[:n * 3], dtype=float).reshape(n, 3)
            if np.all(np.isfinite(seq)):
                self.plan_seq = seq
                self.plan_recv_time = self.get_clock().now()
            else:
                self.get_logger().warning(
                    'Rejected plan sequence containing NaN or Inf')
        self.last_plan_time = self.get_clock().now()

    def stop(self):
        self.motion_enabled = False
        self.linear_speed = 0.0
        self.lateral_speed = 0.0
        self.angular_speed = 0.0
        self.pub_vel()

def main(args = None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    controller = Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            controller.stop()
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
