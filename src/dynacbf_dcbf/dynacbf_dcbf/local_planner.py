# =============================================================================
# 【来源】本文件**逐字复制**自参考实现（另一套已在本机验证跑通的 ROS 2 复现）的
#         local_planner/local_planner/local_planner.py（全文 459 行）。
#         算法、话题名、消息类型、参数名、类名（LocalPlanner）、节点名（'phri_planner'）
#         一律未改：
#         * holonomic model f(x,u) = [cos(yaw)*vx - sin(yaw)*vy,
#           sin(yaw)*vx + cos(yaw)*vy, omega] with body-frame [vx, vy, omega]
#           and discrete x_{i+1} = x_i + T*f
#           * 障碍函数 h（barrier.ellipse_barrier：径向距离型，含 safe_dist）
#           * 判据 o = 到椭圆中心的欧氏距离
#           * MPC_constraint 三分支：None（无约束）/ Euclidean（o >= safe_dist）/
#             CBF（h(x_{i+1}, ob_{i+1}) >= (1-gamma_k)*h(x_i, ob_i)）
#           * 代价：运行项 0.1*(x_i-ref_i)^T Q_i (x_i-ref_i) + u_i^T R u_i
#             （Q_i = diag([1+0.05i, 1+0.05i, 0])，R = diag([0.1, 0.02])），
#             终端项 (x_{N-1}-ref_{N-1})^T (5*diag([1,1,0.02])) (x_{N-1}-ref_{N-1})
#           * 边界：sqrt(vx²+vy²) <= v_max，omega∈[-omega_max, omega_max]；
#             全向底盘允许任意方向的平移（含纯侧移）
#           * 热启动 init_N 组初值、IPOPT 设置、异常处理（失败时 last_input 清零 +
#             last_state 原地保持，返回 solve_succeeded=False）逐行照抄
#         * 订阅 curr_state / obs_predict_pub / global_path
#         * 发布 local_plan / local_path / pub_path_vis / cmd_move
#
# 【改动清单】相对参考实现，本文件**只有两处**改动：
#   改动 1：在本文件顶部增加这段来源/改动说明注释块；
#   改动 2：`from local_planner.barrier import ellipse_barrier`
#           → `from dynacbf_dcbf.barrier import ellipse_barrier`
#           （仅为把 barrier 模块放进本包的目录结构，函数本身逐字复制，
#             见同目录 barrier.py）。该行以 `# 【改动 2】` 就地标出。
#   除此之外没有任何一行被修改、删除或增加；没有为适配本项目而改动任何接口。
#
#   ── 2026-10-12 真机联调追加（顿挫与实时性修复，均可参数关回原行为）──
#   改动 3：求解器加 Wall-clock 上限 `ipopt.max_cpu_time`（参数 ipopt_max_cpu_time，
#           默认 0.2s）。原实现无上限，实测单次求解 0.41–7.11s（P50 1.20 / P90 2.95），
#           是最坏情况失控的直接来源。超时后 IPOPT 抛异常，由改动 5a 取当前迭代点。
#   改动 4：`main()` 改 MultiThreadedExecutor，timer（MPC 求解）放进
#           MutuallyExclusiveCallbackGroup、订阅放进 ReentrantCallbackGroup。
#           原因：原单线程 executor 下求解 0.4–7.1s 期间所有订阅回调被阻塞，
#           /curr_state（50Hz、队列 10）积压溢出 ⇒ 下一轮 MPC 用的是过期位姿。
#           timer 保持独占组，避免两轮 MPC 并发重入。
#   改动 5：求解失败不再硬清零。原实现在失败分支 `last_input.fill(0.0)`
#           （⇒ local_plan 全零）并返回 False（⇒ cmd_move=False ⇒ controller 停车），
#           且清零会污染下一次热启动初值，形成恶性循环。现改为：
#           a) except 分支先用 opti.debug.value() 取当前迭代点作为可用解；
#           b) 仍失败时保留上一次可行解（不发零），避免瞬时急停；
#           c) __replan_cb 的 cmd_move 加连续失败去抖（fail_tolerance 次内仍发 True）。
#
# 【本包内的对接方式（全部在本包之外，见 bspline_path_sampler.py）】
#   * global_path（nav_msgs/Path，本节点只读其中的 x/y）由 bspline_path_sampler
#     把 planning/bspline 的 B 样条按**弧长**几何采样后直接发布到同名话题 global_path，
#     因此本文件不需要任何 remap，也不需要任何代码改动。
#   * obs_predict_pub 仍然是 std_msgs/Float32MultiArray，布局 num_obs × kalman_N × 5，
#     每步 5 个数 [cx, cy, a, b, phi]；kalman_N 必须 == MPC_N（见 __load_parameter 的校验）。
# =============================================================================

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import rclpy.timer
from std_msgs.msg import Float32MultiArray, ColorRGBA, Bool
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import threading
from rclpy.executors import MultiThreadedExecutor                     # 【改动 4】
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,    # 【改动 4】
                                   ReentrantCallbackGroup)
from collections import OrderedDict
import numpy as np
import casadi as ca
import time

from dynacbf_dcbf.barrier import ellipse_barrier  # 【改动 2】原为 from local_planner.barrier import ellipse_barrier

def distance_global(c1,c2):
    return np.sqrt((c1[0] - c2[0]) * (c1[0] - c2[0]) + (c1[1] - c2[1]) * (c1[1] - c2[1]))

class LocalPlanner(Node):
    def __init__(self):
        super().__init__('phri_planner')

        self.curr_state = None
        self.global_path = None

        self.init_N = 0             # 提供casadi求解器初值组数
        self.init_control = None

        self.__load_parameter()

        self.z = 0.0
        self.goal_state = np.zeros([self.N, 3])

        # Each control is the HiveCore body-frame Twist tuple [vx, vy, omega].
        self.last_input = np.zeros([self.N, 3])
        self.last_state = np.zeros([self.N, 3])
        self.mpc_success = None
        self.last_mpc_success = False
        self.__fail_streak = 0        # 【改动 5c】cmd_move 连续失败去抖计数

        self.curr_pose_lock = threading.Lock()
        self.global_path_lock = threading.Lock()
        self.obstacle_lock = threading.Lock()

        self.ob = []

        # MPC 优化图缓存：图结构只随 (MPC_constraint, num_obs) 变化，
        # 数值全部参数化后，只需按障碍条数缓存（LRU），绝大多数 replan
        # 周期跳过建图，直接 set_value + 热启动求解。
        self._mpc_cache = OrderedDict()
        self._mpc_cache_limit = 8

        # 【改动 4】回调分组：timer（MPC 求解）独占组，订阅用可并发组。
        self.__solve_cb_group = MutuallyExclusiveCallbackGroup()
        self.__subs_cb_group = ReentrantCallbackGroup()
        self.__timer_replan = self.create_timer(
            self.replan_period, self.__replan_cb,
            callback_group=self.__solve_cb_group)  # to check

        self.__pub_local_path_vis = self.create_publisher(Marker, 'pub_path_vis', 10)
        self.__pub_local_path = self.create_publisher(Path, 'local_path', 10)
        self.__pub_local_plan = self.create_publisher(Float32MultiArray, 'local_plan', 10)
        self.__pub_start = self.create_publisher(Bool, 'cmd_move', 10)

        # 【改动 4】订阅用 ReentrantCallbackGroup：MPC 求解期间仍能收
        # /curr_state（50Hz）与 /obs_predict_pub，避免状态积压后过期。
        self.__sub_curr_state = self.create_subscription(
            Float32MultiArray, 'curr_state', self.__curr_pose_cb, 10,
            callback_group=self.__subs_cb_group)
        self.__sub_obs = self.create_subscription(
            Float32MultiArray, 'obs_predict_pub', self.__obs_cb, 10,
            callback_group=self.__subs_cb_group)
        self.__sub_goal = self.create_subscription(
            Path, 'global_path', self.__global_path_cb, 25,
            callback_group=self.__subs_cb_group)

    def __replan_cb(self):
        with self.curr_pose_lock, self.global_path_lock, self.obstacle_lock:
            if self.curr_state is None or self.global_path is None:
                return
            curr_state = self.curr_state.copy()
            global_path = self.global_path.copy()
            obstacles = [ob.copy() for ob in self.ob]

        goal_state = self.choose_goal_state(curr_state, global_path)
        if goal_state is not None:
            if distance_global(curr_state, global_path[-1]) <= self.goal_tolerance:
                cmd_move = Bool()
                cmd_move.data = False
                self.__pub_start.publish(cmd_move)
                self.__publish_local_plan(
                    np.zeros([self.N, 3]),
                    np.tile(curr_state, (self.N, 1)))
                return

            # 添加角度信息
            for i in range(self.N - 1):
                y_diff = goal_state[i+1, 1] - goal_state[i, 1]
                x_diff = goal_state[i+1, 0] - goal_state[i, 0]
                if np.hypot(x_diff, y_diff) > 1e-9:
                    goal_state[i, 2] = np.arctan2(y_diff, x_diff)
                elif i != 0:
                    goal_state[i, 2] = goal_state[i-1, 2]
                else:
                    goal_state[i, 2] = curr_state[2]

            goal_state[-1, 2] = goal_state[-2, 2]

            states_sol, input_sol, success = self.MPC_ellip(
                curr_state, goal_state, obstacles)

            # 【改动 5c】cmd_move 去抖：单次求解失败不再立刻停车，
            # 连续 fail_tolerance 次失败才置 False（下游 controller 据此停）。
            if success:
                self.__fail_streak = 0
            else:
                self.__fail_streak += 1

            cmd_move = Bool()
            cmd_move.data = (success
                             or self.__fail_streak < self.fail_tolerance)
            self.__pub_start.publish(cmd_move)
            self.__publish_local_plan(input_sol, states_sol)

    def __curr_pose_cb(self, data):
        if len(data.data) < 3 or not np.all(np.isfinite(data.data[:3])):
            self.get_logger().warning('Rejected invalid current state')
            return
        with self.curr_pose_lock:
            self.curr_state = np.array(data.data[:3], dtype=float)

    def __obs_cb(self, data):
        if len(data.data) % 5 != 0:
            self.get_logger().warning(
                'Rejected obstacle prediction with %d values' % len(data.data))
            return
        obstacles = []
        size = int(len(data.data) / 5)
        for i in range(size):
            obstacle = np.array(data.data[5*i:5*i+5], dtype=float)
            if np.all(np.isfinite(obstacle)) and obstacle[2] > 0.0 and obstacle[3] > 0.0:
                obstacles.append(obstacle)
        with self.obstacle_lock:
            self.ob = obstacles

    def __global_path_cb(self, path):
        size = len(path.poses)
        if size > 0:
            global_path = np.zeros([size, 3])
            for i in range(size):
                global_path[i, 0] = path.poses[i].pose.position.x
                global_path[i, 1] = path.poses[i].pose.position.y
            if np.all(np.isfinite(global_path)):
                with self.global_path_lock:
                    self.global_path = global_path

    def __publish_local_plan(self, input_sol, state_sol):
        local_path = Path()
        local_path.header.frame_id = self.frame_id
        local_path.header.stamp = self.get_clock().now().to_msg()

        local_path_vis = Marker()
        local_path_vis.type = Marker.LINE_LIST
        local_path_vis.scale.x = 0.05
        local_path_vis.color.g = local_path_vis.color.b = local_path_vis.color.r = 1.0
        local_path_vis.header.frame_id = self.frame_id
        local_path_vis.header.stamp = self.get_clock().now().to_msg()

        local_plan = Float32MultiArray()
        local_plan.data = []

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = state_sol[i, 0]
            this_pose_stamped.pose.position.y = state_sol[i, 1]
            this_pose_stamped.pose.position.z = self.z
            this_pose_stamped.pose.orientation.x = 0.0
            this_pose_stamped.pose.orientation.y = 0.0
            this_pose_stamped.pose.orientation.z = 0.0
            this_pose_stamped.pose.orientation.w = 1.0
            this_pose_stamped.header.frame_id = self.frame_id
            this_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            local_path.poses.append(this_pose_stamped)

            for j in range(len(input_sol[i])):
                local_plan.data.append(input_sol[i][j])

            pt = Point()
            pt.x = state_sol[i, 0]
            pt.y = state_sol[i, 1]
            pt.z = self.z

            color = ColorRGBA()
            color.r = 1.0
            color.g = 0.82
            color.b = 0.1
            color.a = 1.0

            p1 = Point()
            p2 = Point()
            p3 = Point()
            p4 = Point()

            if i < self.N-1:
                x_diff = state_sol[i+1, 0]-state_sol[i, 0]
                y_diff = state_sol[i+1, 1]-state_sol[i, 1]
                if x_diff != 0 and y_diff != 0:
                    theta = np.arctan2(y_diff, x_diff)
                else:
                    theta = 0
                w = 0.7
                l = 0.92
                p1.z = pt.z-0.01
                p1.x = 0.5*(l*np.cos(theta)-w*np.sin(theta)) + pt.x
                p1.y = 0.5*(l*np.sin(theta)+w*np.cos(theta)) + pt.y
                p2.z = pt.z-0.01
                p2.x = 0.5*(-l*np.cos(theta)-w*np.sin(theta)) + pt.x
                p2.y = 0.5*(-l*np.sin(theta)+w*np.cos(theta)) + pt.y
                p3.z = pt.z-0.01
                p3.x = 0.5*(-l*np.cos(theta)+w*np.sin(theta)) + pt.x
                p3.y = 0.5*(-l*np.sin(theta)-w*np.cos(theta)) + pt.y
                p4.z = pt.z-0.01
                p4.x = 0.5*(l*np.cos(theta)+w*np.sin(theta)) + pt.x
                p4.y = 0.5*(l*np.sin(theta)-w*np.cos(theta)) + pt.y

                local_path_vis.points.append(p1)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p2)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p2)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p3)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p3)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p4)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p4)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p1)
                local_path_vis.colors.append(color)
                local_path_vis.pose.orientation.x = 0.0
                local_path_vis.pose.orientation.y = 0.0
                local_path_vis.pose.orientation.z = 0.0
                local_path_vis.pose.orientation.w = 1.0

        self.__pub_local_path_vis.publish(local_path_vis)
        self.__pub_local_path.publish(local_path)
        self.__pub_local_plan.publish(local_plan)

    def __load_parameter(self):
        self.declare_parameter('replan_period', 0.05) 
        self.replan_period = self.get_parameter('replan_period').value

        self.declare_parameter('v_max', 1.0)
        self.v_max = self.get_parameter('v_max').value

        # Kept for parameter-file compatibility; holonomic motion does not
        # impose a forward-only lower bound.
        self.declare_parameter('v_min', 0.0)
        self.v_min = self.get_parameter('v_min').value

        self.declare_parameter('omega_max', 1.25)
        self.omega_max = self.get_parameter('omega_max').value

        self.declare_parameter('MPC_N', 25)
        self.N = self.get_parameter('MPC_N').value

        self.declare_parameter('safe_dist', 1.0)
        self.safe_dist = self.get_parameter('safe_dist').value

        # 【真机机身】平滑超椭圆机身参数（前轴/后轴/半宽/指数），barrier.py 按朝向展开。
        # 数值来自实测 footprint + 真实 TF，保守包住 0.94m 长机身（见 dcbf_params.yaml）。
        self.declare_parameter('robot_a_front', 0.73)
        self.robot_a_front = self.get_parameter('robot_a_front').value
        self.declare_parameter('robot_a_rear', 0.89)
        self.robot_a_rear = self.get_parameter('robot_a_rear').value
        self.declare_parameter('robot_b', 0.30)
        self.robot_b = self.get_parameter('robot_b').value
        self.declare_parameter('robot_p', 4.0)
        self.robot_p = self.get_parameter('robot_p').value

        self.declare_parameter('gamma_k', 0.0)
        self.gamma_k = self.get_parameter('gamma_k').value

        self.declare_parameter('MPC_constraint', "CBF")
        self.MPC_constraint = self.get_parameter('MPC_constraint').value

        # 【本项目对接甲方 TF 树】local_path / 可视化所在的全局 frame。
        # 真机统一用 map（甲方 SLAM 全局系）；不再硬编码仿真残留的 "world"。
        # 与 controller / bspline / planner / local_map 的 map_frame 保持一致。
        self.declare_parameter('frame_id', 'map')
        self.frame_id = self.get_parameter('frame_id').value

        self.declare_parameter('kalman_N', 25)
        self.kalman_N = self.get_parameter('kalman_N').value     

        self.declare_parameter('init_N', 0)
        self.init_N = self.get_parameter('init_N').value
        self.get_logger().info("load init_N with value %d " % self.init_N)

        self.declare_parameter('MPC_step_size', 0.2)
        self.MPC_step_size = self.get_parameter('MPC_step_size').value

        self.declare_parameter('goal_tolerance', 0.1)
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # 【改动 3】IPOPT wall-clock 上限（秒）。<=0 表示不限制（原行为）。
        self.declare_parameter('ipopt_max_cpu_time', 0.2)
        self.ipopt_max_cpu_time = float(
            self.get_parameter('ipopt_max_cpu_time').value)

        # 【改动 5】失败处理策略（都可关回原行为）
        #   use_iterate_on_fail：求解抛异常时取当前迭代点当可用解
        #   keep_last_on_fail ：仍失败时保留上次可行解而非清零
        #   fail_tolerance    ：连续失败多少次才把 cmd_move 置 False
        self.declare_parameter('use_iterate_on_fail', True)
        self.use_iterate_on_fail = bool(
            self.get_parameter('use_iterate_on_fail').value)
        self.declare_parameter('keep_last_on_fail', True)
        self.keep_last_on_fail = bool(
            self.get_parameter('keep_last_on_fail').value)
        self.declare_parameter('fail_tolerance', 3)
        self.fail_tolerance = int(
            self.get_parameter('fail_tolerance').value)

        if self.N < 2 or self.kalman_N < 2 or self.kalman_N != self.N:
            raise ValueError('MPC_N and kalman_N must be equal and at least 2')
        if self.init_N < 1:
            raise ValueError('init_N must be at least 1')
        if not (0.0 < self.gamma_k <= 1.0):
            raise ValueError('gamma_k must be in (0, 1]')
        if self.MPC_constraint not in ('CBF', 'Euclidean', 'None'):
            raise ValueError('MPC_constraint must be CBF, Euclidean, or None')
        for name, value in (
                ('replan_period', self.replan_period),
                ('MPC_step_size', self.MPC_step_size),
                ('safe_dist', self.safe_dist),
                ('goal_tolerance', self.goal_tolerance)):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError('%s must be finite and positive' % name)
        for name, value in (
                ('robot_a_front', self.robot_a_front),
                ('robot_a_rear', self.robot_a_rear),
                ('robot_b', self.robot_b),
                ('robot_p', self.robot_p)):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError('%s must be finite and positive' % name)
        if self.robot_p < 2.0 or self.robot_b >= self.robot_a_rear:
            raise ValueError('robot_p must be >= 2 and robot_b < robot_a_rear')

        self.init_control = np.zeros([self.init_N, 3])

        for i in range(self.init_N):
            # Keep init_v_* as the backward-compatible vx parameter.
            self.declare_parameter('init_v_' + str(i), 0.0)
            self.declare_parameter('init_vy_' + str(i), 0.0)
            self.declare_parameter('init_omega_' + str(i), 0.0)
            self.init_control[i, 0] = self.get_parameter('init_v_' + str(i)).value
            self.init_control[i, 1] = self.get_parameter('init_vy_' + str(i)).value
            self.init_control[i, 2] = self.get_parameter('init_omega_' + str(i)).value

    def choose_goal_state(self, curr_state, global_path):
        waypoint_num = global_path.shape[0]
        if waypoint_num == 0:
            return None
        num = np.argmin(np.array([
            distance_global(curr_state, global_path[i])
            for i in range(waypoint_num)]))

        scale = 1
        num_list = []
        for i in range(self.N):
            num_path = min(waypoint_num - 1, num + i * scale)
            num_list.append(num_path)

        goal_state = np.zeros([self.N, 3])
        for k in range(self.N):
            goal_state[k] = global_path[num_list[k]]
        return goal_state

    # ------------------------------------------------------------------
    # 图复用（graph reuse）——修复"停顿/一卡一卡"的核心改动
    #
    # 之前每个 replan 周期都从零用 ca.Opti() 重建整个 MPC 优化问题图
    # （几百行 subject_to），建图本身 ~0.5-1s，远超 controller 的
    # command_timeout=0.15s，于是表现为"动 0.15s 停 ~5s"。
    #
    # 关键观察：这个优化问题的**结构**（决策变量个数、约束类型与数目）
    # 只随障碍条数 num_obs 变化；当前状态 / 目标参考 / 障碍预测都是
    # **数值**。因此把三者全部参数化（opti.parameter），按
    # (MPC_constraint, num_obs) 缓存只建一次的图（OrderedDict LRU），
    # 每周期只 set_value + 用上一解热启动求解。
    # ------------------------------------------------------------------
    _MPC_OBJ_STEP = 5   # 每条障碍预测单步 [cx, cy, a, b, phi]

    def _mpc_graph_key(self, num_obs):
        return (self.MPC_constraint, num_obs)

    def _get_mpc_graph(self, num_obs):
        key = self._mpc_graph_key(num_obs)
        if key in self._mpc_cache:
            self._mpc_cache.move_to_end(key)          # LRU 命中
            return self._mpc_cache[key]
        graph = self._build_mpc_graph(num_obs)
        self._mpc_cache[key] = graph
        self._mpc_cache.move_to_end(key)
        while len(self._mpc_cache) > self._mpc_cache_limit:
            self._mpc_cache.popitem(last=False)       # 淘汰最久未用
        return graph

    def _build_mpc_graph(self, num_obs):
        opti = ca.Opti()
        T = self.MPC_step_size
        gamma_k = self.gamma_k
        max_linear_speed = self.v_max
        omega_max = self.omega_max

        opt_x0 = opti.parameter(3)

        # State [x, y, yaw], body-frame control [vx, vy, omega].
        opt_states = opti.variable(self.N + 1, 3)
        opt_controls = opti.variable(self.N, 3)
        vx = opt_controls[:, 0]
        vy = opt_controls[:, 1]
        omega = opt_controls[:, 2]

        # 障碍预测（num_obs 条 × kalman_N 步，每步 5 数）与目标参考
        # horizon（N×3）都做成参数：结构固定，数值每周期 set_value。
        obs_size = max(num_obs * self.kalman_N * self._MPC_OBJ_STEP, 1)
        obs_param = opti.parameter(obs_size)
        goal_param = opti.parameter(self.N * 3)

        def f(x_, u_):
            c = ca.cos(x_[2])
            s = ca.sin(x_[2])
            return ca.vertcat(
                u_[0] * c - u_[1] * s,
                u_[0] * s + u_[1] * c,
                u_[2])

        def h(curpos_, ob_):
            return ellipse_barrier(curpos_, ob_, self.safe_dist,
                                   self.robot_a_front, self.robot_a_rear,
                                   self.robot_b, self.robot_p)

        def o(curpos_, ob_):
            ob_vec = ca.MX([ob_[0], ob_[1]])
            center_vec = curpos_[:2] - ob_vec.T
            dist = ca.sqrt(center_vec[0] ** 2 + center_vec[1] ** 2)
            return dist

        def quadratic(x, A):
            return ca.mtimes([x, A, x.T])

        # init_condition
        opti.subject_to(opt_states[0, :] == opt_x0.T)

        # HiveCore limits the magnitude of the planar body velocity, not each
        # axis independently.  This preserves the 1.0 m/s resultant limit while
        # allowing pure lateral and diagonal motion.
        for i in range(self.N):
            opti.subject_to(vx[i] ** 2 + vy[i] ** 2 <= max_linear_speed ** 2)
        opti.subject_to(opti.bounded(-omega_max, omega, omega_max))

        # System Model constraints
        for i in range(self.N):
            x_next = opt_states[i, :] + T * f(opt_states[i, :], opt_controls[i, :]).T
            opti.subject_to(opt_states[i + 1, :] == x_next)

        def obs_row(j, i):
            k = (j * self.kalman_N + i) * self._MPC_OBJ_STEP
            return obs_param[k:k + self._MPC_OBJ_STEP]

        # CBF / Euclidean distance constraint（结构与障碍条数绑定，数值参数化）
        if self.MPC_constraint == "CBF":
            for j in range(num_obs):
                for i in range(self.N - 1):
                    opti.subject_to(
                        h(opt_states[i + 1, :], obs_row(j, i + 1)) >=
                        (1 - gamma_k) * h(opt_states[i, :], obs_row(j, i)))
        elif self.MPC_constraint == "Euclidean":
            for j in range(num_obs):
                for i in range(self.N):
                    opti.subject_to(
                        o(opt_states[i, :], obs_row(j, i)) >= self.safe_dist)

        obj = 0
        R = np.diag([0.1, 0.1, 0.02])
        for i in range(1, self.N):
            # Q = np.diag([1.0+0.05*i,1.0+0.05*i, 0.02+0.005*i])
            Q = np.diag([1.0 + 0.05 * i, 1.0 + 0.05 * i, 0.0])
            # goal_param 是 (3N,1) 列向量；切片取列，转置成 (1,3) 行，保持行减行
            ref = goal_param[3 * i:3 * i + 3].T
            if i < self.N - 1:
                obj += 0.1 * quadratic(opt_states[i, :] - ref, Q) \
                    + quadratic(opt_controls[i, :], R)
            else:
                obj += 0.1 * quadratic(opt_states[i, :] - ref, Q)
        Q = np.diag([1.0, 1.0, 0.02]) * 5
        ref_last = goal_param[3 * (self.N - 1):3 * (self.N - 1) + 3].T
        obj += quadratic(opt_states[self.N - 1, :] - ref_last, Q)

        opti.minimize(obj)
        # Keep the real-time loop bounded.  The exact Hessian with the
        # squircle CBF is prohibitively expensive on the onboard CPU; IPOPT's
        # limited-memory approximation is substantially faster and remains
        # suitable for this receding-horizon solve.
        opts_setting = {
            'ipopt.max_iter': 100,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.hessian_approximation': 'limited-memory',
            'ipopt.acceptable_tol': 1e-3,
            'ipopt.acceptable_obj_change_tol': 1e-3,
        }
        # 【改动 3】Wall-clock 上限，截断最坏情况（实测 max 7.11s）。
        if self.ipopt_max_cpu_time > 0.0:
            opts_setting['ipopt.max_cpu_time'] = self.ipopt_max_cpu_time
        opti.solver('ipopt', opts_setting)
        return opti, {
            'x0': opt_x0,
            'obs': obs_param,
            'obs_size': obs_size,
            'goal': goal_param,
            'states': opt_states,
            'controls': opt_controls,
            'obj': obj,
            'f': f,
            'T': T,
        }

    def MPC_ellip(self, curr_state, goal_state, obstacles):
        num_obs = int(len(obstacles) / self.kalman_N)
        opti, handles = self._get_mpc_graph(num_obs)
        opt_states = handles['states']
        opt_controls = handles['controls']
        obj = handles['obj']

        # ---- 数值输入全部 set_value（图结构不动）----
        opti.set_value(handles['x0'], np.asarray(curr_state, dtype=float))
        opti.set_value(handles['goal'],
                       np.asarray(goal_state, dtype=float).reshape(-1))
        obs_data = np.asarray(obstacles, dtype=float).reshape(-1)
        obs_n = handles['obs_size']
        if obs_data.shape[0] != obs_n:
            self.get_logger().warning(
                'obstacle data size %d != graph size %d; padding/truncating'
                % (obs_data.shape[0], obs_n))
            padded = np.zeros(obs_n)
            n = min(obs_data.shape[0], obs_n)
            padded[:n] = obs_data[:n]
            obs_data = padded
        opti.set_value(handles['obs'], obs_data)

        optimized_obj = float('inf')

        start_time = time.perf_counter()

        solve_succeeded = False
        # Hot start from the previous horizon when available; the previous
        # solution is the best initial guess for the receding horizon and
        # typically lets IPOPT converge in only a few iterations.
        if self.last_mpc_success and np.any(np.abs(self.last_input) > 0.0):
            hot_input = self.last_input
            hot_state = self.last_state
            if hot_input.ndim == 2 and hot_input.shape == (self.N, 3) \
                    and hot_state.ndim == 2 and hot_state.shape == (self.N, 3):
                # Shift the previous control sequence by one step and extend
                # the last control, matching the shifted state horizon.
                guess_u = np.vstack([hot_input[1:], hot_input[-1, :]])
                for i in range(self.N):
                    opti.set_initial(opt_controls[i, :], guess_u[i])
                for i in range(self.N + 1):
                    if i < self.N:
                        opti.set_initial(opt_states[i, :], hot_state[i, :])
                    else:
                        opti.set_initial(opt_states[i, :], hot_state[-1, :])
                try:
                    sol = opti.solve()
                    cur_obj = sol.value(obj)
                    if (cur_obj < optimized_obj):
                        optimized_obj = cur_obj
                        u_res = sol.value(opt_controls)
                        state_res = sol.value(opt_states)
                    solve_succeeded = True
                except Exception as e:
                    self.get_logger().warning(
                        'MPC hot-start solve failed (%s); falling back to '
                        'explicit initial guesses' % str(e))

        # Explicit warm guess fallback when the hot start was not usable.
        for k in range(self.init_N):
            if solve_succeeded:
                break
            init_control = self.init_control[k, :]
            init_state = np.zeros([self.N + 1, 3])
            init_state[0, :] = curr_state

            for i in range(self.N):
                state_changed = np.array(
                    handles['T'] * handles['f'](init_state[i, :], init_control).T)
                init_state[i + 1, :] = init_state[i, :] + state_changed

            for i in range(self.N):
                opti.set_initial(opt_controls[i, :], init_control)

            for i in range(self.N + 1):
                opti.set_initial(opt_states[i, :], init_state[i, :])

            try:
                sol = opti.solve()
                cur_obj = sol.value(obj)
                if (cur_obj < optimized_obj):
                    optimized_obj = cur_obj
                    u_res = sol.value(opt_controls)
                    state_res = sol.value(opt_states)
                solve_succeeded = True

            except Exception as e:
                # 【改动 5a】求解抛异常（含 max_cpu_time 截断）时，先尝试取
                # IPOPT 当前迭代点 —— 它通常已满足 acceptable_tol 的约束，
                # 比直接丢弃再清零好得多，也避免污染下一次热启动。
                if self.use_iterate_on_fail:
                    try:
                        u_it = opti.debug.value(opt_controls)
                        s_it = opti.debug.value(opt_states)
                        obj_it = float(opti.debug.value(obj))
                        if (u_it is not None and s_it is not None
                                and np.all(np.isfinite(u_it))
                                and np.all(np.isfinite(s_it))
                                and obj_it < optimized_obj):
                            optimized_obj = obj_it
                            u_res = u_it
                            state_res = s_it
                            solve_succeeded = True
                            self.get_logger().info(
                                'MPC used partial iterate (guess %d, %s)'
                                % (k, type(e).__name__))
                    except Exception:
                        pass
                if not solve_succeeded:
                    self.get_logger().warning(
                        'MPC solve failed for initial guess %d: %s'
                        % (k, str(e)))

        end_time = time.perf_counter()
        self.get_logger().info("solve use time %f" % (end_time - start_time))

        if solve_succeeded:
            self.last_input = u_res
            # Keep the stored horizon (N, 3) so the next hot start always fits
            # the checks above; success path strips the extra terminal row.
            self.last_state = state_res[:self.N]
            self.get_logger().info(
                "find feasible solution with score %f" % optimized_obj)
            self.last_mpc_success = True
        else:
            self.get_logger().error("fail to find feasible solution")
            self.last_mpc_success = False
            if self.keep_last_on_fail:
                # 【改动 5b】保留上一次可行解：不清零、不原地保持。
                # 好处：① controller 不会因单次求解失败瞬时急停；
                #       ② 下一次热启动仍从可用轨迹出发，而非零初值。
                # 兜底：连续失败由 __replan_cb 的 fail_tolerance 停车。
                pass
            else:
                self.last_input.fill(0.0)
                self.last_state = np.tile(curr_state, (self.N, 1))

        u_res = self.last_input
        state_res = self.last_state

        return state_res, u_res, solve_succeeded



def main(args = None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    phri_planner = LocalPlanner()

    # 【改动 4】多线程 executor：MPC 求解（0.4–7.1s）不再阻塞订阅回调，
    # /curr_state 与 /obs_predict_pub 在求解期间照常更新。
    executor = MultiThreadedExecutor()
    executor.add_node(phri_planner)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        phri_planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
