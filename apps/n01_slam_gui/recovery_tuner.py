"""Runtime tuning for the bounded recovery ladder.

The planner reads these limits from its ROS 2 parameters on every failure, so
writing them through the standard parameter service takes effect immediately
and needs no restart. This module only talks to the planner node; it never
touches motion enable, the chassis or the safety gate.
"""
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

PLANNER_NODE = "/scan_planner_node"
LOCAL_PARAM = "recovery.local_attempts"
GLOBAL_PARAM = "recovery.global_attempts"
WAIT_PARAM = "recovery.wait_timeout_s"
# (minimum, maximum, fallback) for each editable limit
RANGES = {
    "local": (1, 50, 5),
    "global": (1, 20, 3),
    "wait": (0.0, 600.0, 0.0),
}


def normalize(local, global_attempts, wait_s):
    """Clamp operator input into the ranges the planner accepts."""
    lo, hi, fallback = RANGES["local"]
    local_i = int(round(float(local))) if local is not None else fallback
    local_i = min(hi, max(lo, local_i))
    lo, hi, fallback = RANGES["global"]
    global_i = int(round(float(global_attempts))) if global_attempts is not None else fallback
    global_i = min(hi, max(lo, global_i))
    lo, hi, fallback = RANGES["wait"]
    wait_f = float(wait_s) if wait_s is not None else fallback
    if wait_f != wait_f:  # NaN
        wait_f = fallback
    wait_f = min(hi, max(lo, wait_f))
    return local_i, global_i, wait_f


def describe(status):
    """Human stage text from one /planning/status payload."""
    status = dict(status or {})
    stage = str(status.get("recovery_stage", ""))
    local = status.get("recovery_local_attempts", 0)
    global_attempts = status.get("recovery_global_attempts", 0)
    local_limit = status.get("recovery_local_limit", RANGES["local"][2])
    global_limit = status.get("recovery_global_limit", RANGES["global"][2])
    wait_s = float(status.get("recovery_wait_s", 0.0) or 0.0)
    if stage == "wait":
        return f"等待恢复：已停稳 {wait_s:.1f}s，局部/全局均用尽（{local_limit}/{global_limit}）"
    if stage == "global_replan":
        return f"全局重规划 {global_attempts}/{global_limit}：局部绕行已用尽"
    if stage == "local_detour":
        return f"局部绕行 {local}/{local_limit}：仅调整当前段，不改全局参考"
    return "恢复策略：未触发"


def build_request(local, global_attempts, wait_s):
    """SetParameters request for the three editable limits."""
    local_i, global_i, wait_f = normalize(local, global_attempts, wait_s)
    req = SetParameters.Request()
    req.parameters = [
        Parameter(name=LOCAL_PARAM,
                  value=ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                       integer_value=local_i)),
        Parameter(name=GLOBAL_PARAM,
                  value=ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                       integer_value=global_i)),
        Parameter(name=WAIT_PARAM,
                  value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                       double_value=wait_f)),
    ]
    return req


def apply_async(node, local, global_attempts, wait_s, on_done):
    """Write the limits to the planner node; on_done receives a short status."""
    try:
        client = node.create_client(SetParameters, f"{PLANNER_NODE}/set_parameters")
    except Exception as exc:  # noqa: BLE001
        on_done(f"无法创建参数客户端：{exc}")
        return None
    if not client.service_is_ready() and not client.wait_for_service(timeout_sec=0.3):
        on_done("规划节点未运行，参数未写入")
        return None
    future = client.call_async(build_request(local, global_attempts, wait_s))

    def _done(fut):
        try:
            response = fut.result()
            rejected = [r.reason for r in response.results if not r.successful]
            on_done("已应用" if not rejected else f"被拒绝：{rejected[0]}")
        except Exception as exc:  # noqa: BLE001
            on_done(f"写入失败：{exc}")

    future.add_done_callback(_done)
    return future


def resume_async(node):
    """Publish one resume request for the held task."""
    from std_msgs.msg import Bool
    try:
        pub = node.create_publisher(Bool, "/n01/navigation/resume", 10)
        pub.publish(Bool(data=True))
        return True
    except Exception:  # noqa: BLE001
        return False
