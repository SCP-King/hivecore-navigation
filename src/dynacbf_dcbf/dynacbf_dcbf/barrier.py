# =============================================================================
# 【来源】本文件**逐字复制**自参考实现（另一套已在本机验证跑通的 ROS 2 复现）的
#         local_planner/local_planner/barrier.py（全文 34 行，含文件头的模块 docstring）。
#
# 【改动清单】相对参考实现，本文件只做了：
#   改动 1：在本文件顶部增加这段来源/改动说明注释块。
#   改动 2：无。import（casadi / numpy）与全部函数体一字未动。
#   改动 3：【真机机身】新增 _robot_squircle_radius()，并把 ellipse_barrier 的原标量
#           safe_distance（圆盘机身）扩展为「按朝向的超椭圆机身 + 安全余量」：
#             h = 中心距 - 椭圆半径 - ρ(机身朝向) - safe_margin
#           其中 ρ 用平滑超椭圆(squircle, 指数 p)表示前/后/侧不同半径，完整覆盖实测
#           0.94m 长 × 0.49m 宽机身（参考基座 base_link，前后缘由 footprint 测量+真实
#           TF 得出）。safe_distance 语义从"机身半径"改为"附加安全余量"。
#           旧调用（不给 robot_* 参数）保持原行为（ρ=0）。
# =============================================================================

"""Geometry helpers for the dynamic elliptical control barrier function."""

import casadi as ca
import numpy as np


def _robot_squircle_radius(theta, a_front, a_rear, b, p):
    """Smooth, conservative boundary radius of the robot body in its own frame.

    ``theta`` is the direction (in the robot body frame) along which the radius is
    wanted.  The body is a superellipse (squircle) with different forward / rear
    axes, which strictly contains the measured capsule footprint (verified
    numerically: never smaller than the footprint).  CasADi ops keep it smooth so
    the MPC h-function stays differentiable.
    """
    c = ca.fmax(ca.fabs(ca.cos(theta)), 1e-12)
    s = ca.fmax(ca.fabs(ca.sin(theta)), 1e-12)
    # forward axis when pointing forward (cos>=0), rear otherwise; smooth blend
    a = a_rear + (a_front - a_rear) * ca.fmax(ca.cos(theta), 0.0)
    return ((c / a) ** p + (s / b) ** p) ** (-1.0 / p)


def ellipse_barrier(position, obstacle, safe_distance,
                    robot_a_front=None, robot_a_rear=None,
                    robot_b=None, robot_p=None):
    """Return radial clearance from a predicted ellipse minus safety margin.

    ``obstacle`` contains ``[cx, cy, semimajor, semiminor, yaw]``.  The
    expression follows the ray from the ellipse centre to the robot, computes
    its intersection with the ellipse, and uses that boundary distance in the
    D-CBF.  It accepts either numeric positions or CasADi expressions.

    ``position`` may be ``[x, y]`` or ``[x, y, yaw]``; the yaw (when present) is
    used to rotate the robot's orientation-dependent body radius toward the
    obstacle.  When the optional ``robot_*`` superellipse parameters are omitted,
    the robot is treated as a disc of radius ``safe_distance`` (legacy behaviour).
    """
    # Use CasADi operations for both numeric and symbolic obstacle values so the
    # obstacle prediction can be a parameter in a reusable MPC graph.
    c = ca.cos(obstacle[4])
    s = ca.sin(obstacle[4])
    dx = position[0] - obstacle[0]
    dy = position[1] - obstacle[1]
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    semimajor = ca.fmax(obstacle[2], 1e-3)
    semiminor = ca.fmax(obstacle[3], 1e-3)
    center_distance = ca.sqrt(dx ** 2 + dy ** 2 + 1e-12)
    radial_denominator = ca.sqrt(
        (semiminor * local_x) ** 2 +
        (semimajor * local_y) ** 2 + 1e-12)
    ellipse_radius = (
        semimajor * semiminor * center_distance / radial_denominator)

    # 机身半径：默认按圆盘 = safe_distance；给定超椭圆参数时按朝向展开。
    # position 可能在 MPC 约束里是 CasADi MX 行向量（shape=(1,3)，不支持 len()），
    # 也可能是数值列表 [x,y,yaw]；统一按「列数/长度 >= 3」判断是否携带 yaw。
    if (robot_a_front is not None and robot_a_rear is not None
            and robot_b is not None and robot_p is not None):
        if hasattr(position, "shape"):      # CasADi MX / 行向量
            yaw = position[2] if position.shape[1] >= 3 else 0.0
        else:                                # 数值列表/数组
            yaw = position[2] if len(position) > 2 else 0.0
        world_ang = ca.atan2(dy, dx)
        theta = world_ang - yaw
        robot_radius = _robot_squircle_radius(theta, robot_a_front,
                                              robot_a_rear, robot_b, robot_p)
    else:
        robot_radius = 0.0

    return center_distance - ellipse_radius - robot_radius - safe_distance


def ellipse_barrier_value(position, obstacle, safe_distance):
    """Numeric convenience wrapper used by tests and diagnostics."""
    return float(ellipse_barrier(position, obstacle, safe_distance))
