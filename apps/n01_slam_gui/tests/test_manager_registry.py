"""Browser/page imports must not lose process ownership."""
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

SOURCE=(Path(__file__).resolve().parents[1]/'slam_lite_gui.py').read_text()


class ManagerRegistryTests(unittest.TestCase):
    def test_page_reimport_preserves_process_and_navigation_ownership(self):
        process_code=SOURCE.split('_process_manager_registry = getattr',1)[1].split("PROCS = _process_manager_registry['manager']",1)[0]
        process_code='_process_manager_registry = getattr'+process_code+"PROCS = _process_manager_registry['manager']"
        lifecycle_code=SOURCE.split('_lifecycle_registry = getattr',1)[1].split("LIFECYCLE = _lifecycle_registry['manager']",1)[0]
        lifecycle_code='_lifecycle_registry = getattr'+lifecycle_code+"LIFECYCLE = _lifecycle_registry['manager']"
        registry=SimpleNamespace();proc_factory=Mock(return_value=SimpleNamespace())
        lifecycle_factory=Mock(side_effect=lambda p:SimpleNamespace(owns_navigation=False))
        def scope():
            return dict(builtins=registry,os=SimpleNamespace(getpid=lambda:123),
                        WORKER=SimpleNamespace(_run=SimpleNamespace(__func__=SimpleNamespace(__globals__={}))),
                        ProcManager=proc_factory,GlobalLifecycleManager=lifecycle_factory)
        first=scope();exec(process_code,first);exec(lifecycle_code,first)
        first['LIFECYCLE'].owns_navigation=True
        second=scope();exec(process_code,second);exec(lifecycle_code,second)
        self.assertIs(first['PROCS'],second['PROCS'])
        self.assertIs(first['LIFECYCLE'],second['LIFECYCLE'])
        self.assertTrue(second['LIFECYCLE'].owns_navigation)
        proc_factory.assert_called_once();lifecycle_factory.assert_called_once()


if __name__=='__main__':unittest.main()
