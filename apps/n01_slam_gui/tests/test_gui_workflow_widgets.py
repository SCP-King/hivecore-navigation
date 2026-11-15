"""Build actual NiceGUI widgets without starting a server, ROS or actuators."""
import sys
from pathlib import Path
from unittest.mock import Mock

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_real_widgets_switch_workflows_and_allow_ready_realtime_mapping(monkeypatch):
    pytest.importorskip('nicegui')
    pytest.importorskip('rclpy')
    import slam_lite_gui as g
    monkeypatch.setattr(g.PROCS, 'running', lambda _:False)
    monkeypatch.setattr(g.WORKER, 'start', Mock(side_effect=AssertionError('ROS must not start')))
    monkeypatch.setattr(g.ProcManager, '_spawn', Mock(side_effect=AssertionError('process must not start')))
    monkeypatch.setattr(g.PROCS, 'desk_mode', '')
    monkeypatch.setattr(g.PROCS, 'slam_selection', {})
    try:
        g.build_console_page()
        assert 'slam_task' not in g.UI
        assert g.UI['offline_slam_actions'].visible
        assert not g.UI['realtime_slam_actions'].visible
        g.UI['slam_map_mode'].value = g.FAST_MODE_REALTIME
        g.refresh_backend_descriptions()
        assert g.UI['realtime_slam_actions'].visible
        for key in ('offline_slam_actions','offline_localization_controls','offline_loc_advanced'):
            assert not g.UI[key].visible

        monkeypatch.setattr(g.PROCS, 'desk_mode', 'mapping')
        monkeypatch.setattr(g.PROCS, 'slam_selection', {'map_mode':'realtime','navigation_allowed':True})
        monkeypatch.setattr(g.PROCS, 'running', lambda key:key=='desk')
        g.STATE.mapping_nav=dict(readiness_semantics='mapping_session_and_fresh_pose', session='mapping',
            mapping_pose_fresh='true', status_mapping_valid='true', source_status_age_s='0.1')
        g.STATE.nav_ready=True
        g.STATE.plan_active=True
        for key in ('n01_status','pose','body_pose','plan_path','nav_ready'):
            g.STATE.recv(key)
        # No offline tracking/map-loaded/relocation flags are synthesized.
        assert g.STATE.loc['session'] != 'tracking'
        g.refresh_panels()
        assert g.UI['drive_button'].enabled
        assert not g.UI['realtime_slam_start'].enabled
        assert 'SLAM' in g.UI['loc_session'].text
        g.STATE.plan_active=False
        g.refresh_panels()
        assert not g.UI['drive_button'].enabled
        assert '本地路径未就绪' in g.UI['drive_ready'].text
        g.WORKER.start.assert_not_called()
        g.ProcManager._spawn.assert_not_called()
    finally:
        g.ui.context.client.content.clear()
        g.UI.clear()
