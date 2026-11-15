"""Behavioral checks of GUI orchestration without starting ROS or GUI servers."""
import ast
import os
import uuid
import math
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parents[1]
SOURCE=(ROOT/'slam_lite_gui.py').read_text()


class OwnershipTests(unittest.TestCase):
    def test_missing_ros_rsp_is_distinguished_from_missing_process(self):
        tree=ast.parse(SOURCE)
        fn=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='rsp_unavailable_message')
        scope={}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'rsp_diagnosis','exec'),scope)
        diagnose=scope['rsp_unavailable_message']
        blocked=diagnose([{'pid':42,'cmd':'/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'}])
        self.assertIn('PID 42',blocked)
        self.assertIn('DDS',blocked)
        self.assertIn('未发现底盘 RSP 进程',diagnose([]))
        self.assertNotIn('请取消勾选',blocked)

    def test_motion_launch_explicitly_passes_cbf_choice(self):
        tree=ast.parse(SOURCE)
        cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='ProcManager')
        method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='start_slow_motion')
        scope={'os':os,'uuid':uuid,'math':math,'NAV_REPO':'/tmp/n01_test'}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'motion_launch','exec'),scope)
        fake=Mock();fake._spawn.return_value=True
        self.assertTrue(scope['start_slow_motion'](fake,.03))
        self.assertIn('cbf_enabled:=false',fake._spawn.call_args.args[1])
        self.assertTrue(scope['start_slow_motion'](fake,.03,True))
        self.assertIn('cbf_enabled:=true',fake._spawn.call_args.args[1])

    def setUp(self):
        tree=ast.parse(SOURCE)
        cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='GlobalLifecycleManager')
        self.run=Mock(return_value=(True,'ok'))
        scope={'threading':threading,'ProcManager':object,'Path':Path,'os':os,
            'NAV_REPO':'/tmp/n01_test','_proc_start_ticks':lambda pid:1,
            'run_navigation_lifecycle':self.run}
        exec(compile(ast.Module(body=[cls],type_ignores=[]),'gui_lifecycle','exec'),scope)
        self.proc=Mock();self.manager=scope['GlobalLifecycleManager'](self.proc)

    def test_external_stack_is_not_stopped_or_restarted(self):
        self.manager._active_navigation_identity=lambda:(123,45)
        self.assertFalse(self.manager.navigation('start')[0])
        self.assertTrue(self.manager.navigation('stop')[0])
        self.manager.stop_all()
        self.manager.shutdown()
        self.run.assert_not_called()

    def test_gui_owned_navigation_is_stopped_on_shutdown(self):
        identities=iter([None,(123,45)])
        self.manager._active_navigation_identity=lambda:next(identities)
        self.assertTrue(self.manager.navigation('start')[0])
        self.manager._active_navigation_identity=lambda:(123,45)
        self.manager.shutdown()
        self.assertEqual([c.args[0] for c in self.run.call_args_list],['start','stop'])

    def test_pid_replacement_does_not_grant_ownership(self):
        self.manager._navigation_started_by_gui=True
        self.manager._navigation_identity=(123,45)
        self.manager._active_navigation_identity=lambda:(123,46)
        self.manager.shutdown()
        self.run.assert_not_called()

    def test_main_only_starts_worker_and_ui(self):
        tree=ast.parse(SOURCE);main=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='main')
        class Ignore:
            def __getattr__(self,k):return Mock()
        worker=Mock();ui=Mock();register=Mock();ui.page.return_value=register;scope={'os':os,'atexit':Ignore(),'app':Ignore(),
            'WORKER':worker,'build_page':Mock(),'ui':ui,'_shutdown_app':Mock(),
            'refresh_console':Mock(),'refresh_panels':Mock(),'refresh_chain':Mock(),
            'refresh_health':Mock(),'draw_grid':Mock(),'drain_logs':Mock(),
            '_ENV_OK':True,'_has_rosk_prefix':lambda:True,'push_log':Mock(),
            'ROSK':'/opt/slam3d_release'}
        exec(compile(ast.Module(body=[main],type_ignores=[]),'gui_main','exec'),scope)
        scope['main']()
        worker.start.assert_called_once()
        scope['build_page'].assert_not_called()
        ui.page.assert_called_once_with('/')
        register.assert_called_once_with(scope['build_page'])

    def test_launcher_and_localization_do_not_spawn_external_drivers(self):
        shell=(ROOT/'run_slam_lite_gui.sh').read_text()
        self.assertNotIn('python3 "${DIR}/tools/startup_preflight.py"',shell)
        launch=(ROOT.parents[1]/'src/n01_live_navigation/launch/n01_localization_stack.launch.py').read_text()
        self.assertNotIn('start_livox_driver.sh',launch)
        self.assertIn("DeclareLaunchArgument('launch_robot_state_publisher', default_value='false', choices=['false'])",launch)
