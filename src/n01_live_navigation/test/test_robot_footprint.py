import math
import numpy as np
from n01_live_navigation.urdf_projection import RobotProjection
from n01_live_navigation.robot_footprint import steering_hull,polygon_hits,rotation_sweep
from n01_live_navigation.footprint_collision import FootprintCollision

XML='''<robot name="test"><link name="base_link"><collision><geometry><box size="0.8 0.30 0.2"/></geometry></collision></link>
<link name="leg"/><joint name="leg_pitch" type="revolute"><parent link="base_link"/><child link="leg"/><axis xyz="0 1 0"/><origin xyz="0.25 0.1 0"/></joint>
<link name="shank"/><joint name="fl_ankle_yaw" type="revolute"><parent link="leg"/><child link="shank"/><axis xyz="0 0 1"/></joint>
<link name="wheel_link"><collision><geometry><box size="0.19 0.04 0.19"/></geometry></collision></link><joint name="wheel_fl" type="continuous"><parent link="shank"/><child link="wheel_link"/><axis xyz="0 1 0"/><origin xyz="0 0.085 0"/></joint></robot>'''
BOX=np.array([[-.4,-.15],[.4,-.15],[.4,.15],[-.4,.15]])

def test_continuous_rotation_is_outer_bounded_between_samples():
    ring=rotation_sweep(np.array([[.2,0,0]]),np.zeros(3),[0,0,1],steps=12)[:,:2]
    angles=np.linspace(0,2*math.pi,1000)
    assert polygon_hits(np.column_stack((.2*np.cos(angles),.2*np.sin(angles))),ring,0).all()

def test_urdf_sweep_contains_random_steering_and_spin_at_tilted_leg_pose():
    m=RobotProjection(XML);j=dict(leg_pitch=.35,fl_ankle_yaw=.2,wheel_fl=.1)
    hull=steering_hull(m,j);rng=np.random.default_rng(13)
    for _ in range(90):
        j.update(fl_ankle_yaw=float(rng.uniform(-math.pi,math.pi)),wheel_fl=float(rng.uniform(-8,8)))
        points=np.concatenate([p for _,p in m.polygons(j)])
        assert polygon_hits(points,hull,0).all()

def test_exact_six_cm_rounded_margin_and_no_double_inflation():
    assert polygon_hits([[0,.209]],BOX).all()
    assert not polygon_hits([[0,.211]],BOX).any()
    assert not polygon_hits([[.45,.20]],BOX).any() # diagonal 7.07cm away
    assert polygon_hits([[.44,.19]],BOX).all()

def test_narrow_corridor_and_rotated_body_use_polygon_not_three_disks():
    wall=np.array([[x,y,.4]for x in np.linspace(-3,3,100)for y in [-.27,.27]])
    c=FootprintCollision(wall,[0,0,0],.5,[[-3,-2],[3,2]],footprint=BOX)
    assert c.radius==.06 and c.free([[0,0,0]])
    assert not c.free([[0,0,math.pi/2]])
    import pytest
    with pytest.raises(ValueError,match='urdf_footprint_required'):
        FootprintCollision(wall,[0,0,0],.5,[[-3,-2],[3,2]])


def test_only_integrated_out_joints_can_be_missing():
    import pytest
    from n01_live_navigation.robot_footprint import sweep_joints
    m=RobotProjection(XML)
    j=sweep_joints(m,{'leg_pitch':.35})
    assert j=={'leg_pitch':.35,'fl_ankle_yaw':0.,'wheel_fl':0.}
    with pytest.raises(ValueError,match='leg_pitch'):sweep_joints(m,{})


def test_review_and_planner_share_three_second_model_budget():
    import pytest,rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from geometry_msgs.msg import PolygonStamped,Point32
    from n01_live_navigation.footprint_client import FootprintClient
    own_context=not rclpy.ok()
    if own_context:rclpy.init()
    node=Node('test_review_model_budget')
    try:
        client=FootprintClient(node)
        def model(age):
            msg=PolygonStamped();msg.header.frame_id='base_link'
            msg.header.stamp=(node.get_clock().now()-Duration(seconds=age)).to_msg()
            msg.polygon.points=[Point32(x=float(x),y=float(y)) for x,y in BOX]
            client.receive(msg)
        model(1.5)
        assert len(client.get())==4
        model(4.)
        with pytest.raises(ValueError,match='missing_or_stale'):client.get()
    finally:
        node.destroy_node()
        if own_context:rclpy.shutdown()
