"""Runtime selection must survive inherited vendor paths and reject bad roots."""
import os
from pathlib import Path
import subprocess

SCRIPT=Path(__file__).resolve().parents[3]/'scripts/lib/slam_runtime.sh'

def test_explicit_runtime_replaces_old_vendor_paths_and_preserves_other_dependencies(tmp_path):
    vendor=tmp_path/'vendor';vendor.mkdir()
    (vendor/'setup_runtime.sh').write_text('export AMENT_PREFIX_PATH="$SLAM_ROSK/install/hivecore_slam3d_lite_localization:${AMENT_PREFIX_PATH:-}"\nros2() { printf "%s\\n" "$SLAM_ROSK/install/hivecore_slam3d_lite_localization"; }\n')
    env=dict(os.environ,SLAM_ROSK=str(vendor),
        AMENT_PREFIX_PATH='/opt/slam3d_release/install/old:/opt/ros/humble',
        LD_LIBRARY_PATH='/opt/slam3d_release/lib:/opt/hivecore/lib',
        PYTHONPATH='/opt/slam3d_release/install/old/python:/other/python')
    p=subprocess.run(['bash','-uc','export LD_LIBRARY_PATH=/opt/slam3d_release/lib:/opt/hivecore/lib; source "$1"; n01_source_slam_runtime || exit $?; printf "%s\\n%s\\n%s\\n" "$AMENT_PREFIX_PATH" "$LD_LIBRARY_PATH" "$PYTHONPATH"','_',str(SCRIPT)],env=env,text=True,capture_output=True)
    assert p.returncode==0,p.stderr
    assert '/opt/slam3d_release' not in p.stdout
    assert str(vendor) in p.stdout and '/opt/ros/humble' in p.stdout
    assert '/opt/hivecore/lib' in p.stdout and '/other/python' in p.stdout

def test_missing_selected_release_does_not_fall_back(tmp_path):
    p=subprocess.run(['bash','-c','source "$1"; n01_source_slam_runtime','_',str(SCRIPT)],env=dict(os.environ,SLAM_ROSK=str(tmp_path/'missing')),text=True,capture_output=True)
    assert p.returncode==2 and 'Missing runtime' in p.stderr

def test_gui_children_do_not_replay_workspace_underlays():
    root=SCRIPT.parents[2]
    for name in ('start_localization_stack.sh','start_fast_lio_gui_backend.sh','start_navigation_execution.sh'):
        text=(root/'scripts/runtime'/name).read_text()
        assert 'n01_source_slam_runtime' in text
        assert '/install/setup.bash' not in text
        assert '/install/local_setup.bash' in text
