"""Check command routing and archived-entry refusal without ROS/hardware."""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

SCRIPTS=Path(__file__).resolve().parent


class EntryPointTests(unittest.TestCase):
    def test_router_preserves_arguments_without_launching_real_modules(self):
        routes={
            'gui':'apps/n01_slam_gui/run_slam_lite_gui.sh',
            'localization':'scripts/runtime/start_localization_stack.sh',
            'fast-lio-backend':'scripts/runtime/start_fast_lio_gui_backend.sh',
            'motion':'scripts/runtime/start_slow_motion.sh',
            'navigation':'scripts/n01_navigation.sh',
        }
        with tempfile.TemporaryDirectory(prefix='n01_router_test_') as tmp:
            root=Path(tmp);(root/'scripts').mkdir();shutil.copy2(SCRIPTS/'n01',root/'scripts/n01')
            for command,target in routes.items():
                real=SCRIPTS.parent/target
                self.assertTrue(real.is_file(),str(real))
                p=root/target;p.parent.mkdir(parents=True,exist_ok=True)
                p.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n')
                result=subprocess.run(['bash',str(root/'scripts/n01'),command,'first','arg with spaces'],capture_output=True,text=True,check=True)
                self.assertEqual(result.stdout.splitlines(),['first','arg with spaces'])

    def test_archived_scripts_refuse_before_touching_hardware(self):
        files=list((SCRIPTS/'archive').glob('*.disabled'))
        self.assertEqual(len(files),7)
        for path in files:
            executable='python3' if '.py.' in path.name else 'bash'
            result=subprocess.run([executable,str(path)],capture_output=True,text=True,timeout=3)
            self.assertNotEqual(result.returncode,0,path.name)
            self.assertIn('停用',result.stderr+result.stdout)

    def test_shell_syntax(self):
        for p in [SCRIPTS/'n01',*(SCRIPTS/'runtime').glob('*.sh'),*(SCRIPTS/'archive').glob('*.sh.disabled')]:
            subprocess.run(['bash','-n',str(p)],check=True)


if __name__=='__main__':unittest.main()
