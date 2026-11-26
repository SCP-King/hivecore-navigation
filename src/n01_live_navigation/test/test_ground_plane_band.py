import json
import numpy as np
from n01_live_navigation.core import ground_plane_height_band


def test_sloped_floor_filters_locally_and_summary_is_json_serializable():
    points=np.array([[0.,0.,-.7],[10.,0.,-.6],[10.,0.,-.86],[0.,0.,.5]])
    band,bounds=ground_plane_height_band(points,-.9,.9,.01,0.)
    np.testing.assert_allclose(band,points[:2])
    data=json.loads(json.dumps({'z_band_map_m':bounds}))
    np.testing.assert_allclose(data['z_band_map_m'],[-.84,.4])


def test_empty_and_horizontal_clouds_have_scalar_diagnostic_bounds():
    for points in (np.empty((0,3)), np.array([[0.,0.,-.7]])):
        _,bounds=ground_plane_height_band(points,-.9,.9)
        assert all(type(v) is float for v in bounds)
        json.dumps(bounds)


def test_only_objects_above_six_centimeters_survive_ground_band():
    points=np.array([[0,0,.0],[0,0,.02],[0,0,.04],[0,0,.055],[0,0,.10]])
    band,_=ground_plane_height_band(points,0.,.9)
    np.testing.assert_allclose(band,points[4:])
