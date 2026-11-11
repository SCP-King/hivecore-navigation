#!/usr/bin/env python3
"""Static regression checks for process-global ROS worker and clean ROS env."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "slam_lite_gui.py").read_text()
launcher = (ROOT / "run_slam_lite_gui.sh").read_text()
checks = {
    "builtins process-wide worker registry": "_N01_SLAM_GUI_ROS_WORKER_REGISTRY" in source,
    "global worker object reused": 'WORKER = _registry["worker"]' in source,
    "idempotent worker start": "ROS worker 已运行且已连接；复用当前唯一 worker" in source,
    "dedicated executor": "executor = SingleThreadedExecutor()" in source,
    "context ownership tracked": "self._owns_context = True" in source and "if self._owns_context and rclpy.ok()" in source,
    "clean environment launch": 'exec env -i "${clean_env[@]}" bash "$0" "$@"' in launcher,
    "old prefixes not inherited": "AMENT_PREFIX_PATH" not in launcher.split("clean_env=(", 1)[1].split(")", 1)[0],
    "lite prefix verified": 'ros2 pkg prefix hivecore_slam3d_lite_localization' in launcher,
    "preflight invoked before GUI": 'python3 "${DIR}/tools/startup_preflight.py"' in launcher,
    "one global lifecycle orchestrator": "class GlobalLifecycleManager" in source and "LIFECYCLE = GlobalLifecycleManager(PROCS)" in source,
    "GUI shutdown preserves externally started navigation": "if owned_navigation:" in source and "独立导航实例保持不变" in source,
    "desk passes N-01 static TF override": 'static_tf_params_file:={STATIC_TF_PARAMS}' in source and 'n01_static_tf_params.yaml' in source,
}
failed = []
for name, ok in checks.items():
    print(("PASS " if ok else "FAIL ") + name)
    if not ok:
        failed.append(name)
raise SystemExit(1 if failed else 0)
