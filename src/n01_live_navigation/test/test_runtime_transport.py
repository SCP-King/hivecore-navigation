"""Transport selection remains local and preserves an explicit fallback."""
import os
from pathlib import Path
import subprocess

SCRIPT=Path(__file__).resolve().parents[3]/'scripts/lib/ros_transport.sh'


def run_transport(prefix, selected='auto'):
    env=dict(os.environ,N01_CYCLONE_PREFIX=str(prefix),N01_RUNTIME_RMW=selected,
             RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    return subprocess.run(['bash','-c',
        'source "$1"; n01_configure_project_transport || exit 42; printf "%s" "$RMW_IMPLEMENTATION"',
        '_',str(SCRIPT)],env=env,text=True,capture_output=True)


def test_auto_keeps_existing_transport_when_private_dependency_is_missing(tmp_path):
    p=run_transport(tmp_path)
    assert p.returncode==0 and p.stdout=='rmw_fastrtps_cpp'
    assert 'WARN' in p.stderr


def test_explicit_cyclone_fails_if_dependency_is_missing(tmp_path):
    p=run_transport(tmp_path,'rmw_cyclonedds_cpp')
    assert p.returncode==42 and 'runtime missing' in p.stderr


def test_auto_prefers_available_private_runtime_but_honors_explicit_fallback(tmp_path):
    for relative in ('lib/librmw_cyclonedds_cpp.so','lib/x86_64-linux-gnu/libddsc.so.0'):
        p=tmp_path/relative;p.parent.mkdir(parents=True,exist_ok=True);p.touch()
    p=run_transport(tmp_path)
    assert p.returncode==0 and p.stdout=='rmw_cyclonedds_cpp'
    p=run_transport(tmp_path,'rmw_fastrtps_cpp')
    assert p.returncode==0 and p.stdout=='rmw_fastrtps_cpp'
