from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
import pytest
from n01_live_navigation.environment_geometry import visible_geometry, ground_grid


def geometry(points, **kwargs):
    return visible_geometry(points,(0.,0.,-1.),resolution=.1,**kwargs)


def test_adjacent_voxels_remove_internal_faces_and_projected_internal_edges():
    result=geometry([[.05,.05,.05],[.15,.05,.05]])
    assert result['voxels']==2 and result['faces']==10
    assert result['triangles'].shape==(60,3)
    assert result['outlines'].shape==(12,3)
    assert np.allclose(result['outlines'][:,2],-.985)


def test_surface_triangles_face_outward_for_correct_rviz_lighting():
    points=geometry([[.05,.05,.05]])['triangles'].reshape((-1,3,3))
    normals=np.cross(points[:,1]-points[:,0],points[:,2]-points[:,0])
    assert np.all((normals*(points.mean(axis=1)-.05)).sum(axis=1)>0)


def test_ground_filter_uses_same_sloped_plane_and_preserves_observed_overhang():
    result=visible_geometry([[0.,0.,-.8],[1.,0.,-.7],[0.,0.,.4]],(.1,0.,-.8),resolution=.1)
    assert result['voxels']==1
    assert result['triangles'][:,2].min()>.3  # no invented solid pedestal to floor


def test_unobserved_gaps_are_not_bridged_and_source_cloud_is_unchanged():
    points=np.array([[.05,.05,.05],[1.05,.05,.05]])
    original=points.copy();result=geometry(points)
    assert result['faces']==12
    assert not np.any((result['triangles'][:,0]>.2)&(result['triangles'][:,0]<.9))
    np.testing.assert_array_equal(points,original)


def test_adaptive_resolution_bounds_rendering_work_without_silently_truncating_map():
    points=np.array(np.meshgrid(*([np.arange(.025,.4,.05)]*3))).reshape(3,-1).T
    result=visible_geometry(points,(0.,0.,-1.),resolution=.05,max_faces=300)
    assert result['resolution']>.05 and result['faces']<=300
    assert result['points']==len(points)
    with pytest.raises(ValueError,match='budget'):
        visible_geometry(points,(0.,0.,-1.),resolution=.05,max_resolution=.05,max_faces=1)


def test_empty_map_and_nonfinite_or_out_of_height_points_clear_geometry():
    result=geometry([[float('nan'),0,0],[0,0,3.],[0,0,-1.]])
    assert result['faces']==0 and result['triangles'].shape==(0,3)
    assert result['outlines'].shape==(0,3)
    pytest.importorskip('rclpy')
    from n01_live_navigation.environment_display import display_markers
    from std_msgs.msg import Header
    from visualization_msgs.msg import Marker
    assert all(m.action==Marker.DELETE for m in display_markers(result,Header(frame_id='map')).markers)


def test_ground_grid_follows_affine_plane_instead_of_assuming_map_z_zero():
    grid=ground_grid((.01,-.02,-.85),center=(1.5,-2.))
    np.testing.assert_allclose(grid[:,2],.01*grid[:,0]-.02*grid[:,1]-.85+.005)


@pytest.mark.parametrize('grid_stamp,scan_stamp', [(1,1),(10,1),(1,10)])
def test_stale_live_surfaces_are_deleted_not_latched_as_current_obstacles(grid_stamp,scan_stamp):
    pytest.importorskip('rclpy')
    from n01_live_navigation.environment_display import EnvironmentDisplay
    from visualization_msgs.msg import Marker
    fake=S(live_pub=S(get_subscription_count=lambda:1,publish=Mock()),
           reference_pub=S(get_subscription_count=lambda:0),grid_pub=S(get_subscription_count=lambda:0),
           live_sub=object(),live_visible=True,last_scan_stamp_ns=scan_stamp*10**9,center=(0.,0.),reference=None,
           live=S(header=S(frame_id='map',stamp=S(sec=grid_stamp,nanosec=0))),
           get_clock=lambda:S(now=lambda:S(nanoseconds=10_000_000_000)))
    fake.clear_live=lambda:EnvironmentDisplay.clear_live(fake)
    EnvironmentDisplay.update(fake)
    output=fake.live_pub.publish.call_args.args[0]
    assert all(marker.action==Marker.DELETE for marker in output.markers)
    assert not fake.live_visible


def test_gui_and_launch_use_same_surface_first_rviz_configuration():
    from pathlib import Path
    import yaml
    root=Path(__file__).resolve().parents[3]
    config=root/'src/n01_live_navigation/rviz/n01_live_navigation.rviz'
    assert config.read_bytes()==(root/'rviz/n01_full_pipeline.rviz').read_bytes()
    displays=yaml.safe_load(config.read_text())['Visualization Manager']['Displays']
    assert all(not d['Enabled'] for d in displays if d['Class']=='rviz_default_plugins/PointCloud2')
    topics={d.get('Topic',{}).get('Value') for d in displays if d['Enabled']}
    assert {'/n01/navigation/live_surfaces','/n01/navigation/reference_surfaces','/n01/navigation/path'}<=topics


def test_oriented_patches_follow_sloped_surface_without_voxel_extrusion():
    from n01_live_navigation.environment_geometry import contour_geometry
    x,y=np.meshgrid(np.linspace(-.5,.5,20),np.linspace(-.5,.5,20))
    points=np.column_stack((x.ravel(),y.ravel(),.6+.4*x.ravel()))
    g=contour_geometry(points,(0,0,0))
    tri=g['triangles']
    assert len(tri)>0
    assert np.max(np.abs(tri[:,2]-(.6+.4*tri[:,0])))<1e-8
    assert len(g['outlines'])==0
    assert g['faces']<=24000


def test_contour_geometry_does_not_bridge_separate_objects():
    from n01_live_navigation.environment_geometry import contour_geometry
    points=np.array([[0,0,.5],[2,0,.5],[float('nan'),0,0],[0,0,.02]])
    g=contour_geometry(points,(0,0,0))
    tri=g['triangles']
    assert g['points']==2
    assert np.all((tri[:,0]<.1)|(tri[:,0]>1.9))


def test_live_surface_uses_compensated_measurement_instead_of_planning_voxels():
    from n01_live_navigation.environment_display import EnvironmentDisplay
    msg=S(header=S(frame_id='map',stamp=S(sec=3,nanosec=4)),width=100,height=1)
    fake=S(live=None,on_scan=Mock())
    EnvironmentDisplay.on_live_cloud(fake,msg)
    assert fake.live is msg
    fake.on_scan.assert_called_once_with(msg)
