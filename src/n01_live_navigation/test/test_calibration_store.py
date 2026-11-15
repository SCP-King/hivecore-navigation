import os
from pathlib import Path
import subprocess
import sys
import pytest
from n01_live_navigation.calibration_store import load_map_ground, save_map_ground

PLANE = dict(a=-.0067, b=-.0139, c=-.8868, support=5600, residual=.026)


def test_profile_survives_a_new_process_and_matches_the_map(tmp_path):
    pcd=tmp_path/'map.pcd';pcd.write_bytes(b'original map data')
    store=tmp_path/'config/n01_map_ground_profiles.json'
    save_map_ground(pcd, PLANE, (-1.,1.), store)
    profile=load_map_ground(pcd, store)
    assert {k:profile[k] for k in PLANE} == PLANE
    module=Path(__file__).resolve().parents[1]/'n01_live_navigation/calibration_store.py'
    output=subprocess.check_output([sys.executable,str(module),str(pcd)],
                                   env={**os.environ,'N01_NAV_REPO':str(tmp_path)},text=True)
    assert list(map(float,output.split())) == [PLANE['c'],PLANE['a'],PLANE['b']]
    # Replacing a map in-place must invalidate the saved plane.
    pcd.write_bytes(b'replacement map data')
    assert load_map_ground(pcd,store) is None


def test_profiles_are_separate_and_invalid_values_do_not_overwrite(tmp_path):
    a=tmp_path/'a.pcd';b=tmp_path/'b.pcd';a.write_bytes(b'a');b.write_bytes(b'b')
    store=tmp_path/'profiles.json'
    save_map_ground(a,PLANE,(0.,0.),store)
    save_map_ground(b,{**PLANE,'c':-.4},(1.,0.),store)
    assert load_map_ground(a,store)['c']==PLANE['c']
    assert load_map_ground(b,store)['c']==-.4
    with pytest.raises(ValueError):
        save_map_ground(a,{**PLANE,'c':float('nan')},(0,0),store)
    assert load_map_ground(a,store)['c']==PLANE['c']
