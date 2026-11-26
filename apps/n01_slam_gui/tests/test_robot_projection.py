import sys,math
from pathlib import Path
import numpy as np
import pytest
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_projection import RobotProjection,paint_robot

XML='''<robot name="test"><link name="base_link"><collision><geometry><box size="0.6 0.3 0.2"/></geometry></collision></link>
<link name="leg"><collision><origin xyz="0.5 0 0"/><geometry><box size="0.2 0.1 0.1"/></geometry></collision></link>
<joint name="steer" type="revolute"><parent link="base_link"/><child link="leg"/><origin xyz="0.2 0 0"/><axis xyz="0 0 1"/></joint></robot>'''


def test_measured_joint_and_collision_origin_transform_geometry():
    r=RobotProjection(XML)
    straight=dict(r.polygons({'steer':0}))
    turned=dict(r.polygons({'steer':math.pi/2}))
    np.testing.assert_allclose(straight['leg'].mean(axis=0),[.7,0],atol=1e-7)
    np.testing.assert_allclose(turned['leg'].mean(axis=0),[.2,.5],atol=1e-7)
    np.testing.assert_allclose(np.ptp(straight['base_link'],axis=0),[.6,.3])
    with pytest.raises(ValueError,match='缺少'):r.polygons({})


def test_unsupported_mesh_does_not_silently_create_default_robot():
    with pytest.raises(ValueError,match='mesh'):
        RobotProjection('<robot><link name="base_link"><collision><geometry><mesh filename="unknown.stl"/></geometry></collision></link></robot>')


def test_robot_and_environment_envelopes_use_distinct_colors():
    view={'x0':-1,'y0':-1,'size_m':2.}
    base=Image.new('RGB',(320,320),(12,20,31))
    result=paint_robot(base,view,(0,0,0),RobotProjection(XML).polygons({'steer':0}),
        {'centers_x_m':[-.43,-.10,.23],'radius_m':.385},np.array([[.8,.8]]))
    a=np.array(result)
    assert np.any(np.all(a==[226,116,42],axis=-1))
    assert np.any(np.all(a==[202,150,255],axis=-1))
    assert np.any(np.all(a!=[12,20,31],axis=-1))
    # Removing both real layer inputs removes their colors, not the URDF body.
    clean=np.array(paint_robot(base.copy(),view,(0,0,0),RobotProjection(XML).polygons({'steer':0})))
    assert not np.any(np.all(clean==[202,150,255],axis=-1))


def test_ground_hint_follows_four_wheel_geometry_not_fixed_body_height():
    xml='<robot><link name="base_link"/>'
    for side,x,y in [('fl',.3,.2),('fr',.3,-.2),('rl',-.3,.2),('rr',-.3,-.2)]:
        xml+=f'<link name="{side}_wheel_link"><collision><geometry><box size=".19 .04 .19"/></geometry></collision></link>'
        xml+=f'<joint name="{side}" type="fixed"><parent link="base_link"/><child link="{side}_wheel_link"/><origin xyz="{x} {y} -.7"/></joint>'
    r=RobotProjection(xml+'</robot>')
    height,tolerance=r.floor_hint({},.1)
    assert height==pytest.approx(-.695) and tolerance==.12
    assert r.floor_hint({},.3)[0]==pytest.approx(-.495)


def test_polygon_envelope_draws_without_legacy_disks():
    base=Image.new('RGB',(320,320),(12,20,31))
    result=paint_robot(base,{'x0':-1,'y0':-1,'size_m':2.},(0,0,.3),[],
        {'model':'urdf_steering_hull','vertices_m':[[-.5,-.2],[.4,-.2],[.4,.2],[-.5,.2]]})
    assert np.any(np.all(np.array(result)==[202,150,255],axis=-1))
    paint_robot(base,{'x0':-1,'y0':-1,'size_m':2.},(0,0,0),[],{'model':'urdf_unavailable'})
