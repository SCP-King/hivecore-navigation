#!/usr/bin/env python3
"""Unit tests for the two-stage preflight; no ROS graph or robot required."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("startup_preflight", ROOT / "tools" / "startup_preflight.py")
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)

def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond:
        raise AssertionError(name)

# --- ownership classification of the ambiguous SCAN bridge -------------------
check("bridge absent", m.classify_scan_bridge([], None) == ("absent", ""))
check("bridge with managed navigation PID is nav-owned",
      m.classify_scan_bridge([m.SCAN_BRIDGE], 1234)[0] == "nav")
check("bridge with localization stack is upstream",
      m.classify_scan_bridge(
          [m.SCAN_BRIDGE, "/n01_localization_adapter", "/localization_lite_node"],
          None)[0] == "upstream")
check("bridge with mapping node is upstream",
      m.classify_scan_bridge([m.SCAN_BRIDGE, "/mapping_lite_node"], None)[0] == "upstream")
check("unowned bridge is orphan",
      m.classify_scan_bridge([m.SCAN_BRIDGE], None)[0] == "orphan")
check("bridge is not a static navigation marker",
      m.SCAN_BRIDGE not in m.NAV_MARKERS)

# --- scoped cleanup trigger set ---------------------------------------------
with patch.object(m, "run") as run:
    run.return_value.returncode = 0
    run.return_value.stdout = "stopped\n"
    rc = m.stop_scoped_navigation(["/scan_planner_node"])
    check("navigation marker invokes scoped manager", rc == 0 and run.call_count == 1)

with patch.object(m, "run") as run:
    run.return_value.returncode = 0
    # Bridge alone must not trigger a navigation stop: it is started by the
    # localization data path, which the navigation lifecycle does not own.
    rc = m.stop_scoped_navigation(["/n01_live_scan_bridge"])
    check("bridge alone does not invoke navigation manager",
          rc == 0 and run.call_count == 0)

with patch.object(m, "run") as run:
    rc = m.stop_scoped_navigation(["/localization_lite_node", "/livox_lidar_publisher"])
    check("protected SLAM and sensor nodes are not stopped", rc == 0 and run.call_count == 0)

with patch.object(m, "active_managed_navigation_pid", return_value=4321), \
     patch.object(m, "run") as run:
    run.return_value.returncode = 0
    rc = m.stop_scoped_navigation(["/scan_planner_node"])
    check("active managed navigation is preserved", rc == 0 and run.call_count == 0)

# --- full main() flows -------------------------------------------------------
with patch.object(m, "census", side_effect=[
    ["/n01_live_scan_bridge", "/localization_lite_node", "/livox_lidar_publisher"],
    ["/n01_live_scan_bridge", "/localization_lite_node", "/livox_lidar_publisher"],
]), patch.object(m, "stop_scoped_navigation", return_value=0), \
     patch.object(m, "active_managed_navigation_pid", return_value=None):
    rc = m.main()
    check("localization-owned bridge passes preflight", rc == 0)

with patch.object(m, "census", side_effect=[
    ["/n01_live_scan_bridge"],
    ["/n01_live_scan_bridge"],
]), patch.object(m, "stop_scoped_navigation", return_value=0), \
     patch.object(m, "active_managed_navigation_pid", return_value=None):
    rc = m.main()
    check("unowned bridge blocks startup", rc == 23)

with patch.object(m, "census", side_effect=[
    ["/n01_live_scan_bridge"],
    ["/n01_live_scan_bridge"],
]), patch.object(m, "stop_scoped_navigation", return_value=0), \
     patch.object(m, "active_managed_navigation_pid", return_value=4321):
    rc = m.main()
    check("managed-navigation-owned bridge passes preflight", rc == 0)

with patch.object(m, "census", return_value=["/scan_planner_node"]), \
     patch.object(m, "stop_scoped_navigation", return_value=0), \
     patch.object(m, "active_managed_navigation_pid", return_value=None):
    rc = m.main()
    check("remaining N01 node blocks startup", rc == 22)

with patch.object(m, "census", side_effect=[
    ["/scan_planner_node"],
    ["/scan_planner_node"],
]), patch.object(m, "stop_scoped_navigation", return_value=0), \
     patch.object(m, "active_managed_navigation_pid", return_value=4321):
    rc = m.main()
    check("active managed navigation remains visible without cleanup", rc == 0)

print("all preflight tests passed")