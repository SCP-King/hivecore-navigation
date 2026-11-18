import numpy as np
import pytest
from n01_live_navigation.scene_perception import BodyGeometry,LocalGround,ScenePerception

XML='''<robot name="test"><link name="base_link"><collision><geometry><box size="1 .4 .2"/></geometry></collision></link>
<link name="arm"><collision><origin xyz=".2 0 0"/><geometry><box size=".4 .1 .1"/></geometry></collision></link>
<joint name="steer" type="revolute"><parent link="base_link"/><child link="arm"/><origin xyz=".5 0 0"/><axis xyz="0 0 1"/></joint></robot>'''


def test_self_mask_follows_actual_joint_and_keeps_nearby_external_return():
    body=BodyGeometry(XML)
    p=np.array([[0,0,0],[.7,0,0],[.5,.3,0],[.7,.08,0],[0,0,.15]])
    assert body.outside(p,{'steer':0}).tolist()==[False,False,True,True,True]
    assert body.outside(p,{'steer':np.pi/2}).tolist()==[False,True,False,True,True]
    with pytest.raises(ValueError):body.outside(p,{})


def test_ground_does_not_promote_chair_seat_or_low_box_to_floor():
    x,y=np.meshgrid(np.linspace(0,.8,12),np.linspace(0,.8,12));floor=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))
    ground=LocalGround([0,0,0])
    for height in (.05,.4,.8):
        cloud=np.concatenate((floor,floor+[0,0,height]))
        mask,confidence=ground.classify(cloud,1.)
        assert mask[:len(floor)].all()
        assert not mask[len(floor):].any()
        assert (confidence==1).all()
    isolated=floor+[0,0,.05]
    assert not LocalGround([0,0,0]).classify(isolated,2.)[0].any()


def test_ground_sloped_seed_patch_and_expiration():
    x,y=np.meshgrid(np.linspace(0,.8,12),np.linspace(0,.8,12));p=np.column_stack((x.ravel(),y.ravel(),(.01*x+.005*y+.01).ravel()))
    g=LocalGround([.01,.005,0]);assert g.classify(p,1.)[0].all()
    # A stale patch cannot classify a sparse high point as ground.
    mask,confidence=g.classify(np.array([[.2,.2,.055]]),4.)
    assert not mask[0] and confidence[0]<1


def test_joints_are_interpolated_at_scan_time_not_replaced_by_latest():
    s=ScenePerception([0,0,0]);s.body=BodyGeometry(XML)
    s.history={'steer':[(1.,3.1),(1.1,-3.1)]}
    assert abs(abs(s.joints_at(1.05)['steer'])-np.pi)<1e-6
    assert 'steer'not in s.joints_at(2.)
    with pytest.raises(ValueError):s.process(np.array([[2.,0,0]]),([0,0,0],[0,0,0,1]),2.)


def test_production_bridge_preserves_xyz_and_carries_ground_label_for_ray_clearing():
    from types import SimpleNamespace as S
    from sensor_msgs.msg import PointCloud2,PointField
    from n01_live_navigation.live_scan_bridge import LiveScanBridge
    from n01_live_navigation.core import decode_pointcloud_xyz
    p=np.array([[0,0,0],[2,0,0],[2,0,.4],[3,0,.05]],dtype='<f4')
    msg=PointCloud2();msg.header.frame_id='lidar';msg.header.stamp.sec=1;msg.height=1;msg.width=len(p)
    msg.fields=[PointField(name=name,offset=i*4,datatype=7,count=1)for i,name in enumerate(('x','y','z'))]
    msg.point_step=12;msg.row_step=len(p)*12;msg.data=p.tobytes()
    perception=ScenePerception([0,0,0]);perception.body=BodyGeometry(XML);perception.history={'steer':[(1.,0.)]}
    node=S(max_points=100,max_range=12.,map_frame='map',scene_perception=perception,_stamp_ns=lambda stamp:stamp.sec*10**9+stamp.nanosec)
    output=LiveScanBridge._xyz_map_cloud(node,msg,[0,0,0],[0,0,0,1],([0,0,0],[0,0,0,1]))
    np.testing.assert_allclose(decode_pointcloud_xyz(output,100,12.),p[1:])
    assert output.point_step==16
    assert np.frombuffer(output.data,dtype='<f4').reshape(-1,4)[:,3].tolist()==[1.,0.,0.]
    assert output.header.stamp==msg.header.stamp
    assert msg.header.frame_id=="lidar"


def test_sphere_cylinder_and_collision_origin_are_used_for_body_mask():
    xml='''<robot name="primitives"><link name="base_link">
    <collision><origin xyz="2 0 0"/><geometry><sphere radius=".1"/></geometry></collision>
    <collision><origin xyz="0 2 0" rpy="0 1.5707963267948966 0"/><geometry><cylinder radius=".1" length=".4"/></geometry></collision>
    </link></robot>'''
    b=BodyGeometry(xml)
    points=np.array([[2,0,0],[2,.2,0],[.15,2,0],[.25,2,0],[0,2.11,0]])
    assert b.outside(points,{}).tolist()==[False,True,False,True,True]


def test_old_ground_scan_cannot_rewind_temporal_patch():
    x,y=np.meshgrid(np.linspace(0,.8,12),np.linspace(0,.8,12));p=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))
    g=LocalGround([0,0,0]);g.classify(p,2.)
    g.classify(p+[0,0,.01],1.)
    assert g.last_stamp==2. and all(v[1]==2. for v in g.patches.values())


def test_unconfigured_ground_never_assumes_map_zero_and_snapshot_is_immutable():
    scene=ScenePerception(None);scene.body=BodyGeometry(XML);scene.history={'steer':[(1.,0.)]}
    world=np.array([[2.,0.,0.],[2.,0.,-.9],[2.,0.,-.85]])
    pose=([0,0,0],[0,0,0,1])
    before=scene.snapshot(1.)
    _,mask=scene.process(world,pose,1.,snapshot=before)
    assert not mask.any() and not scene.status['ground_plane_ready']
    scene.next_ground_plane=(0.,0.,-.9)
    _,mask=scene.process(world,pose,1.)
    assert mask.tolist()==[False,True,False]
    _,mask=scene.process(world,pose,1.,snapshot=before)
    assert not mask.any(), 'worker must use the plane paired with its scan snapshot'
