#!/usr/bin/env python3
"""Contract checks: one map/ground for localization, nav and grid."""
from pathlib import Path

GUI = Path(__file__).resolve().parents[1]
NAV = GUI.parents[1]
gui = (GUI / "slam_lite_gui.py").read_text()
nav_script = (NAV / "scripts" / "n01_navigation.sh").read_text()
nav_launch = (NAV / "src" / "n01_live_navigation" / "launch" /
              "live_scan.launch.py").read_text()
loc_cfg = (GUI / "config" / "n01_localization_lite.yaml").read_text()
monitor = (NAV / "src" / "n01_live_navigation" / "n01_live_navigation" /
           "trajectory_monitor.py").read_text()
checks = {
    "candidate path replay while active": "last_path_active" in monitor
                                           and "self.path_pub.publish(self.last_path)" in monitor,
    "nav PCD env override": "N01_NAV_PCD" in nav_script
                            and 'PCD="${N01_NAV_PCD:-$DEFAULT_PCD}"' in nav_script,
    "nav ground env override": "N01_NAV_GROUND_Z" in nav_script
                               and 'ground_z_map_m:="$NAV_GROUND_Z"' in nav_script,
    "nav rviz headless default": 'NAV_RVIZ="${N01_NAV_RVIZ:-false}"' in nav_script
                                 and 'rviz:="$NAV_RVIZ"' in nav_script,
    "realtime navigation skips static PCD": all(x in nav_script + nav_launch for x in (
        'N01_NAV_MAP_MODE', 'map_mode:="$NAV_MAP_MODE"',
        'map_mode not in ("offline", "realtime")',
        '"grid_map.static_cloud_enabled": map_mode == "offline"')),
    "GUI passes nav PCD/ground": all(x in gui for x in (
        '"N01_NAV_PCD"', '"N01_NAV_GROUND_Z"', '"N01_NAV_RVIZ"', 'nav_pcd', 'nav_ground_z')),
    "GUI shows map/ground consistency": "nav_pcd_match" in gui
                                       and "nav_ground_match" in gui,
    "localization config map exists": any(
        line.split("map_path:", 1)[1].strip()
        and Path(line.split("map_path:", 1)[1].strip()).is_file()
        for line in loc_cfg.splitlines() if "map_path:" in line),
}
failed = []
for name, ok in checks.items():
    print(("PASS " if ok else "FAIL ") + name)
    if not ok:
        failed.append(name)
raise SystemExit(1 if failed else 0)
