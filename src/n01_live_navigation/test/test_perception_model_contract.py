"""Geometry consistency checks independent of robot motion."""
import ast
import json
from pathlib import Path
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[3]


def params():
    result={}
    for node in ast.walk(ast.parse((ROOT/'src/n01_live_navigation/launch/live_scan.launch.py').read_text())):
        if isinstance(node,ast.Dict):
            for k,v in zip(node.keys,node.values):
                try:result[ast.literal_eval(k)]=ast.literal_eval(v)
                except (ValueError,TypeError):pass
    return result


def test_solver_tracking_frame_and_mount_are_consistent():
    config=ROOT/'apps/n01_slam_gui/config'
    loc=yaml.safe_load((config/'n01_localization_lite.yaml').read_text())['/**']['ros__parameters']
    mapping=yaml.safe_load((config/'n01_mapping_lite.yaml').read_text())['/**']['ros__parameters']
    assert loc['tracking_frame']==loc['mola_vehicle_frame']=='lidar_link'
    assert loc['vehicle_to_scan_xyz']==[0.,0.,0.]
    assert loc['pose_topic']=='/n01/vendor_localization/pose_raw'
    assert mapping['tracking_frame']==mapping['mola_vehicle_frame']=='lidar_link'
    assert mapping['vehicle_to_scan_xyz']==[0.,0.,0.]
    assert mapping['tracking_to_vehicle_xyz']==[0.,0.,0.]
    assert loc['lidar_imu_params_file']==mapping['lidar_imu_params_file']
    adapter=yaml.safe_load((ROOT/'src/n01_live_navigation/config/slam3d_lite_adapter.yaml').read_text())['n01_localization_adapter']['ros__parameters']
    assert adapter['expected_pose_child_frame']=='lidar_link'
    assert adapter['output_pose_child_frame']=='base_link'
    assert adapter['pose_input_topic']==loc['pose_topic']
    assert adapter['pose_mirror_topic']=='/robot_pose'


def test_navigation_selects_shared_urdf_footprint_with_six_cm_margin():
    p=params()
    assert p['grid_map.use_urdf_footprint'] is True
    assert p['grid_map.environment_margin']==.06
    launch=(ROOT/'src/n01_live_navigation/launch/live_scan.launch.py').read_text()
    assert 'executable="n01_robot_footprint"' in launch


def test_vertical_extent_and_ground_band_are_separate():
    p=params()
    assert p['grid_map.obstacles_inflation_z_up']>=.789
    assert p['grid_map.obstacles_inflation_z_down']>=.06
    geometry=json.loads((ROOT/'config/n01_sensor_geometry.json').read_text())
    # Re-referencing the body below the lidar must not leave the sensor above
    # the planner's collision volume. This checks two independent consumers.
    assert p['grid_map.obstacles_inflation_z_down'] >= geometry['body_to_lidar_xyz_m'][2]+.10
    assert .03<=p['grid_map.ground_clearance']<=.06
