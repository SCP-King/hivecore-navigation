import json
import math
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teach_routes import TeachingSession,RouteStore,map_identity,execution_points,matching_arrival


def context(tmp_path):
    p=tmp_path/'map.pcd';p.write_bytes(b'map')
    g=tmp_path/'geometry.json';g.write_text('{}')
    return p,g,map_identity(p,g),RouteStore(tmp_path/'routes')


def test_teaching_round_trip_and_optional_return(tmp_path):
    p,g,identity,store=context(tmp_path);s=TeachingSession();s.begin(identity)
    for i in range(3):s.record(identity,(i,0,-.1,0),(i,0,-.1,0),str(i),2.)
    path=s.save(store,'待命-A-B',identity,True)
    assert not s.active
    route=store.load(path,identity)
    assert [p['x']for p in route['points']]==[0,1,2]
    assert [p['x']for p in execution_points(route)]==[0,1,2,0]
    assert route['created_utc'] and route['id']
    assert path in store.list()


def test_station_state_is_preserved_and_independent_of_later_robot_updates(tmp_path):
    _,_,identity,store=context(tmp_path);s=TeachingSession();s.begin(identity)
    state={'source':'measured_stop','captured_unix_ns':100,
           'pose':{'stamp_ns':90,'position':[0,0,0],'quaternion':[0,0,0,1]},
           'joints':{'names':['wheel'],'position':[.2]},
           'odometry':{'linear':[0,0,0],'angular':[0,0,0]}}
    s.record(identity,(0,0,0,0),(0,0,0,0),'A',1.,state=state)
    state['joints']['position'][0]=1.2
    s.record(identity,(1,0,0,0),(1,0,0,0),'B',1.,state={'source':'measured_stop','captured_unix_ns':200})
    route=store.load(s.save(store,'实际停靠 A-B',identity),identity)
    assert route['points'][0]['state']['joints']['position']==[.2]
    assert route['points'][0]['state']['pose']['stamp_ns']==90
    assert route['points'][1]['x']==1
    assert [p['name']for p in execution_points(route)]==['A','B']


def test_changed_map_or_geometry_cannot_load_or_record(tmp_path):
    p,g,identity,store=context(tmp_path);s=TeachingSession();s.begin(identity)
    s.record(identity,(0,0,0,0),(0,0,0,0),'A')
    g.write_text('{"mount":0.15}')
    with pytest.raises(ValueError):s.record(map_identity(p,g),(1,0,0,0),(1,0,0,0),'B')
    path=s.save(store,'test',identity)
    with pytest.raises(ValueError):store.load(path,map_identity(p,g))
    p.write_bytes(b'rebuilt map')
    with pytest.raises(ValueError):store.load(path,map_identity(p,g))


def test_unstable_pose_and_invalid_values_do_not_change_draft(tmp_path):
    _,_,identity,store=context(tmp_path);s=TeachingSession();s.begin(identity)
    for after in [(0.1,0,0,0),(0,0,0,.2),(0,0,float('nan'),0)]:
        with pytest.raises(ValueError):s.record(identity,(0,0,0,0),after,'A')
    assert s.points==[]
    with pytest.raises(ValueError):s.save(store,'empty',identity)
    assert s.active
    s.record(identity,(0,0,0,0),(0,0,0,0),'A');s.undo();assert not s.points


def test_path_escape_and_corrupt_files_are_rejected(tmp_path):
    _,_,identity,store=context(tmp_path);store.directory.mkdir()
    outside=tmp_path/'outside.json';outside.write_text('{}')
    with pytest.raises(ValueError):store.load(outside)
    (store.directory/'bad.json').write_text('[]')
    assert store.list()=={}


def test_old_or_incomplete_feedback_never_advances_a_station():
    target={'x':1.,'y':0.,'yaw':math.pi/2}
    assert not matching_arrival({'task_id':'old','state':'ARRIVED'},'new',(1,0,math.pi/2),target,'rotate')
    assert not matching_arrival({'task_id':'new','state':'SESSION_COMPLETE'},'new',(1,0,math.pi/2),target,'rotate')
    assert not matching_arrival({'task_id':'new','state':'ARRIVED'},'new',(1,0,0),target,'rotate')
    assert matching_arrival({'task_id':'new','state':'ARRIVED'},'new',(1,0,math.pi/2),target,'rotate')


def test_delete_removes_only_selected_route_and_keeps_recoverable_copy(tmp_path):
    _,_,identity,store=context(tmp_path)
    points=[dict(name='A',x=0,y=0,z=0,yaw=0,wait_s=0)]
    selected=store.save('删除',identity,points)
    other=store.save('保留',identity,points)
    removed=store.delete(selected)
    assert not Path(selected).exists()
    assert list(store.list())==[other]
    saved=list((store.directory/'.trash').glob('*.json'))
    assert len(saved)==1 and json.loads(saved[0].read_text())==removed


def test_delete_rejects_outside_paths_and_symlinks(tmp_path):
    _,_,identity,store=context(tmp_path)
    route=store.save('保留',identity,[dict(name='A',x=0,y=0,z=0,yaw=0,wait_s=0)])
    outside=tmp_path/'outside.json';outside.write_bytes(Path(route).read_bytes())
    link=store.directory/'link.json';link.symlink_to(outside)
    for path in (outside,link,store.directory/'../outside.json'):
        with pytest.raises(ValueError):store.delete(path)
    assert outside.exists() and Path(route).exists()
    (store.directory/'.trash').symlink_to(tmp_path,target_is_directory=True)
    with pytest.raises(ValueError):store.delete(route)
    assert Path(route).exists()


def test_new_routes_persist_after_arrival_orientation_without_changing_legacy(tmp_path):
 from teach_routes import validate
 _,_,identity,store=context(tmp_path)
 path=store.save('四站',identity,[dict(name='A',x=0,y=0,z=0,yaw=0,wait_s=1)])
 route=store.load(path);assert route['orientation_timing']=='after_arrival'
 route.pop('orientation_timing');validate(route)
 route['orientation_timing']='invalid'
 with pytest.raises(ValueError,match='朝向执行时机'):validate(route)
