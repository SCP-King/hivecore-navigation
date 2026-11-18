"""Registry and ROS parameter I/O for tunables that belong to this project.

Only parameters defined by our own navigation stack are listed. Client
(甲方) SLAM, chassis, lidar and remote-control parameters are deliberately
absent: this panel never touches them.
"""
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

PLANNER = "/scan_planner_node"
MONITOR = "/n01_hive2_trajectory_monitor"
CONTROLLER = "/n01_navigation_controller"

# name, node, label(zh), desc(zh), type, min, max, live(立即生效)
REGISTRY = [
    ("recovery.local_attempts", PLANNER, "局部绕行次数",
     "局部规划连续失败后，保留全局参考继续重试的次数上限；用尽后进入全局重规划。",
     "int", 1, 50, True),
    ("recovery.global_attempts", PLANNER, "全局重规划次数",
     "局部绕行用尽后，从当前位姿重算全局参考的次数上限；用尽后进入零速等待。",
     "int", 1, 20, True),
    ("recovery.wait_timeout_s", PLANNER, "等待超时",
     "进入等待恢复后，超过该秒数自动重试原目标；0 表示只等人工“继续任务”。",
     "float", 0.0, 600.0, True),
    ("recovery.auto_resume", PLANNER, "等待后自动恢复",
     "等待超时到达时是否自动重试；关闭则只能手动继续或重发目标。",
     "bool", None, None, True),
    ("fsm.max_replan_interval_s", PLANNER, "重规划剩余时间",
     "当前局部轨迹剩余时长低于该值时触发重规划；越小越频繁，负载越高。",
     "float", 0.2, 5.0, False),
    ("fsm.thresh_replan", PLANNER, "偏离触发重规划",
     "当前位置距轨迹起点超过该距离时重规划。", "float", 0.0, 2.0, False),
    ("fsm.thresh_no_replan", PLANNER, "到点不重规划",
     "距最终目标小于该距离时不再重规划，交给到点判定。", "float", 0.0, 2.0, False),
    ("fsm.planning_horizon", PLANNER, "局部前瞻距离",
     "沿全局参考向前取局部目标的最大距离；受滑动窗口限制。", "float", 1.0, 30.0, False),
    ("grid_map.observation_ttl_s", PLANNER, "观测覆盖有效期",
     "观测覆盖用于诊断的有效时间；覆盖门控已关闭，不决定是否通行。",
     "float", 1.0, 30.0, False),
    ("grid_map.environment_margin", PLANNER, "环境余量",
     "碰撞包络外扩量，越大越保守；不是定位精度承诺。", "float", 0.0, 0.3, False),
    ("manager.max_vel", PLANNER, "局部规划速度上限",
     "SCAN 优化的速度约束；必须不高于控制器实际限速。", "float", 0.01, 1.0, False),
    ("manager.max_acc", PLANNER, "局部规划加速度上限",
     "SCAN 优化的加速度约束；过小会导致轨迹可行性检查失败。", "float", 0.01, 1.0, False),
    ("perception.observed_free_max_age_s", MONITOR, "观测覆盖时限（旧门控）",
     "观测覆盖的接收时限；覆盖门控已关闭，仅在重新启用时才生效。",
     "float", 0.1, 3.0, True),
    ("feedback.pose_max_age_s", CONTROLLER, "控制位姿接收时限",
     "超过该时间未收到新位姿就停车等待；放宽会增加使用旧位姿的风险。",
     "float", 0.1, 3.0, True),
    ("feedback.pose_stamp_max_age_s", CONTROLLER, "控制位姿时间戳时限",
     "位姿消息头时间戳的最大年龄；与接收时限分开判断。",
     "float", 0.1, 3.0, True),
    ("feedback.odom_max_age_s", CONTROLLER, "底盘速度反馈时限",
     "超过该时间未收到底盘速度反馈就停车。", "float", 0.1, 3.0, True),
    ("feedback.joints_max_age_s", CONTROLLER, "轮组反馈时限",
     "超过该时间未收到轮组/关节反馈就停车。", "float", 0.1, 3.0, True),
    ("feedback.packet_max_age_s", CONTROLLER, "轨迹包接收时限",
     "收到轨迹后超过该时间未更新就拒绝执行。", "float", 0.1, 3.0, True),
    ("feedback.scene_max_age_s", CONTROLLER, "执行场景时限",
     "执行场景/轨迹时间戳的最大年龄。", "float", 0.1, 3.0, True),
    ("feedback.observed_space_max_age_s", CONTROLLER, "观测覆盖时限（已停用）",
     "仅在 require_observed_space 打开时生效；当前生产配置为关闭。",
     "float", 0.1, 3.0, True),
    ("arrival.confirm_s", CONTROLLER, "到达停稳确认时间",
     "到达区停稳后需要持续的确认时间，之后才锁存 ARRIVED。",
     "float", 0.1, 3.0, True),
    ("perception.scene_max_age_s", MONITOR, "占用图新鲜度",
     "占用图时间戳（原始扫描时间）超过该值即拒绝局部轨迹；不是甲方合同值。",
     "float", 0.1, 3.0, True),
    ("perception.ground_max_age_s", MONITOR, "地面参考新鲜度",
     "地面参考接收时间超过该值即拒绝局部轨迹。", "float", 0.5, 10.0, True),
    ("perception.risk_speed_scale", MONITOR, "预测风险降速比例",
     "预测障碍与轨迹冲突时的限速比例；1.0 表示不降速。", "float", 0.05, 1.0, True),
    ("execution_preview_m", MONITOR, "执行预判距离",
     "轨迹审核向前预判的距离；过短会在高速时提前失效。", "float", 0.3, 2.0, False),
    ("max_bspline_receive_age_s", MONITOR, "样条接收有效期",
     "收到 SCAN 样条后允许等待审核的最大时间。", "float", 0.2, 5.0, False),
    ("max_speed_mps", CONTROLLER, "控制器速度上限",
     "控制器允许输出的最大线速度；仍需底盘自身限速允许。", "float", 0.01, 1.0, False),
    ("max_accel_mps2", CONTROLLER, "控制器加速度上限",
     "控制器速度斜坡的加速度上限。", "float", 0.01, 1.0, False),
    ("max_yaw_rate_radps", CONTROLLER, "控制器角速度上限",
     "原地转向/对齐时的最大角速度。", "float", 0.01, 1.5, False),
]

BY_KEY = {row[0]: row for row in REGISTRY}


def spec(key):
    row = BY_KEY.get(key)
    if row is None:
        raise KeyError(key)
    name, node, label, desc, kind, lo, hi, live = row
    return dict(name=name, node=node, label=label, desc=desc, kind=kind,
                min=lo, max=hi, live=live)


def validate(key, value):
    """Coerce and clamp one operator value; raises ValueError on bad type."""
    s = spec(key)
    kind = s["kind"]
    if kind == "bool":
        if isinstance(value, bool):
            return value
        text = str(value).strip().lower()
        if text in ("1", "true", "on", "yes", "是", "开"):
            return True
        if text in ("0", "false", "off", "no", "否", "关"):
            return False
        raise ValueError("bool value required")
    if kind == "int":
        number = int(round(float(value)))
    else:
        number = float(value)
        if number != number:  # NaN
            raise ValueError("NaN")
    lo, hi = s["min"], s["max"]
    if number < lo or number > hi:
        raise ValueError(f"超出范围 [{lo}, {hi}]")
    return number


def build_set_request(key, value):
    s = spec(key)
    value = validate(key, value)
    req = SetParameters.Request()
    if s["kind"] == "bool":
        pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    elif s["kind"] == "int":
        pv = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    else:
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
    req.parameters = [Parameter(name=s["name"], value=pv)]
    return req


def build_get_request(key):
    req = GetParameters.Request()
    req.names = [spec(key)["name"]]
    return req


def parse_value(key, response):
    """Extract the typed value from a GetParameters response."""
    s = spec(key)
    value = response.values[0]
    if s["kind"] == "bool":
        return bool(value.bool_value)
    if s["kind"] == "int":
        return int(value.integer_value)
    return float(value.double_value)


def describe(key, current=None):
    s = spec(key)
    mode = "立即生效" if s["live"] else "重启后生效"
    rng = "" if s["kind"] == "bool" else f"范围 [{s['min']}, {s['max']}]"
    head = f"{s['label']}（{s['name']}）· {mode}"
    tail = f"{s['desc']}" + (f" {rng}" if rng else "")
    if current is not None:
        return f"{head}\n当前值：{current}\n{tail}"
    return f"{head}\n{tail}"


def read_async(node, key, on_done):
    """Read one parameter; on_done(value, error_text)."""
    s = spec(key)
    try:
        client = node.create_client(GetParameters, f"{s['node']}/get_parameters")
    except Exception as exc:  # noqa: BLE001
        on_done(None, f"无法创建参数客户端：{exc}")
        return
    if not client.service_is_ready() and not client.wait_for_service(timeout_sec=0.3):
        on_done(None, f"{s['node']} 未运行")
        return
    future = client.call_async(build_get_request(key))

    def _done(fut):
        try:
            on_done(parse_value(key, fut.result()), "")
        except Exception as exc:  # noqa: BLE001
            on_done(None, f"读取失败：{exc}")

    future.add_done_callback(_done)


def apply_async(node, key, value, on_done):
    """Write one parameter immediately when the node reads it live."""
    s = spec(key)
    try:
        request = build_set_request(key, value)
    except ValueError as exc:
        on_done(f"参数无效：{exc}")
        return
    try:
        client = node.create_client(SetParameters, f"{s['node']}/set_parameters")
    except Exception as exc:  # noqa: BLE001
        on_done(f"无法创建参数客户端：{exc}")
        return
    if not client.service_is_ready() and not client.wait_for_service(timeout_sec=0.3):
        on_done(f"{s['node']} 未运行，参数未写入")
        return
    future = client.call_async(request)

    def _done(fut):
        try:
            response = fut.result()
            rejected = [r.reason for r in response.results if not r.successful]
            if rejected:
                on_done(f"被拒绝：{rejected[0]}")
            else:
                on_done("已应用" if s["live"] else "已写入，重启对应节点后生效")
        except Exception as exc:  # noqa: BLE001
            on_done(f"写入失败：{exc}")

    future.add_done_callback(_done)


def options():
    """NiceGUI select options: value -> label (NiceGUI dict semantics)."""
    out = {}
    for row in REGISTRY:
        name, node, label, _desc, _kind, _lo, _hi, live = row
        mode = "立即" if live else "重启"
        out[name] = f"{label} · {name}（{mode}）"
    return out
