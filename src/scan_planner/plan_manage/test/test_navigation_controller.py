"""Synthetic ROS-domain test of the REAL C++ controller, never a hardware test.

Run explicitly with ROS_DOMAIN_ID=215 N01_CONTROLLER_INTEGRATION=1 pytest ...
No live sensors or desktop nodes are started or modified.
"""
import json,math,os,signal,subprocess,time,struct
from pathlib import Path
from collections import Counter
import pytest
pytestmark=pytest.mark.skipif(os.getenv('N01_CONTROLLER_INTEGRATION')!='1',reason='explicit isolated ROS integration only')


def test_controller_route_and_authority(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID')=='215', 'must use isolated domain 215'
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import Twist,Point,PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState,PointCloud2,PointField
    from std_msgs.msg import Bool,UInt8,String
    from scan_planner_msgs.msg import NavigationTrajectory,Bspline
    from n01_live_navigation.hive2_kinematics import inverse
    rclpy.init();node=rclpy.create_node('synthetic_navigation_test')
    pubs={}
    for topic,typ in [('/body_pose',Odometry),('/odom',Odometry),('/joint_states',JointState),
            ('/n01/live_navigation/system_health',String),('/n01/navigation/trajectory',NavigationTrajectory),
            ('/n01/live_navigation/goal_gated',PoseStamped),('/n01/navigation/enable',Bool)]:
        pubs[topic]=node.create_publisher(typ,topic,10)
    latched=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    dec=node.create_publisher(UInt8,'/chassis_dec_stop/state',latched)
    emergency=node.create_publisher(Bool,'/estop',latched)
    external=node.create_publisher(Twist,'/cmd_vel',10)
    pose=[0.,0.,0.];command=[0.,0.];joints=[0.]*4
    statuses=[];outputs=[];current={};packet=None;goal_id=0;enabled=False;publish_scene=True
    def status(m):
        value=json.loads(m.data);current.clear();current.update(value);statuses.append(value)
    def cmd(m):command[:]=[m.linear.x,m.angular.z];outputs.append((time.monotonic(),*command))
    node.create_subscription(String,'/n01/navigation/execution_status',status,10)
    node.create_subscription(Twist,'/cmd_vel',cmd,10)
    log=tmp_path/'controller.log'
    stream=log.open('w')
    process=subprocess.Popen(['ros2','run','scan_planner','n01_navigation_controller','--ros-args',
       '-p','task_id:=synthetic_route','-p','enabled:=false','-p','max_speed_mps:=0.2',
       '-p','max_yaw_rate_radps:=0.4','-p','max_accel_mps2:=0.3','-p','require_final_yaw:=true',
       '-p','follow_goal_updates:=true'],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    last=time.monotonic();last_packet=0.
    def step(duration):
        nonlocal last,last_packet,joints
        end=time.monotonic()+duration
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.003)
            now=time.monotonic();dt=min(.03,now-last);last=now
            v,w=command;pose[2]+=w*dt;pose[0]+=v*math.cos(pose[2])*dt;pose[1]+=v*math.sin(pose[2])*dt
            stamp=node.get_clock().now().to_msg();ns=stamp.sec*10**9+stamp.nanosec
            p=Odometry();p.header.stamp=stamp;p.header.frame_id='map';p.child_frame_id='base_link'
            p.pose.pose.position=Point(x=pose[0],y=pose[1],z=.775)
            p.pose.pose.orientation.z=math.sin(pose[2]/2);p.pose.pose.orientation.w=math.cos(pose[2]/2)
            pubs['/body_pose'].publish(p)
            od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_link'
            # Reproduce the known doubled raw pose; measured twist stays true.
            od.pose.pose.position.x=pose[0]*2;od.pose.pose.position.y=pose[1]*2;od.pose.pose.orientation.w=1.
            od.twist.twist.linear.x=v;od.twist.twist.angular.z=w;pubs['/odom'].publish(od)
            if abs(v)+abs(w)>1e-8:joints=inverse((v,0.,w),joints)[0].tolist()
            j=JointState();j.header.stamp=stamp;j.name=[f'{leg}_ankle_yaw'for leg in ('fl','fr','rl','rr')];j.position=joints;j.velocity=[0.]*4;pubs['/joint_states'].publish(j)
            health={'bridge':{'pose_prediction':{'fault':'','source_stamp_ns':ns,'source_age_s':0.,
               'source_pose':{'x':pose[0],'y':pose[1],'quaternion':[0.,0.,math.sin(pose[2]/2),math.cos(pose[2]/2)]}}}}
            pubs['/n01/live_navigation/system_health'].publish(String(data=json.dumps(health)))
            dec.publish(UInt8(data=0));emergency.publish(Bool(data=False))
            if packet is not None and now-last_packet>.08 and publish_scene:
                packet.header.stamp=stamp;packet.scene_stamp=stamp;packet.rotation_stamp=stamp;packet.scene_center=Point(x=pose[0],y=pose[1])
                pubs['/n01/navigation/trajectory'].publish(packet);last_packet=now
            assert process.poll()is None,log.read_text()
            time.sleep(.003)
    def obstacles(points):
        cloud=PointCloud2();cloud.header.frame_id='map';cloud.height=1;cloud.width=len(points)
        cloud.fields=[PointField(name=name,offset=i*4,datatype=7,count=1)for i,name in enumerate(('x','y','z'))]
        cloud.point_step=12;cloud.row_step=12*len(points);cloud.is_dense=True
        cloud.data=b''.join(struct.pack('<fff',p.x,p.y,p.z)for p in points)
        return cloud
    def goal(x,y,yaw,hold=False):
        nonlocal packet,goal_id
        goal_id=node.get_clock().now().nanoseconds
        g=PoseStamped();g.header.frame_id='map';g.header.stamp=node.get_clock().now().to_msg()
        goal_id=g.header.stamp.sec*10**9+g.header.stamp.nanosec
        g.pose.position=Point(x=float(x),y=float(y),z=.775);g.pose.orientation.z=math.sin(yaw/2);g.pose.orientation.w=math.cos(yaw/2)
        pubs['/n01/live_navigation/goal_gated'].publish(g)
        packet=NavigationTrajectory();packet.header.frame_id='map';packet.map_id='synthetic_session';packet.goal_id=goal_id;packet.revision=1
        packet.goal=Point(x=float(x),y=float(y));packet.goal_yaw=float(yaw)
        packet.scene_valid=True;packet.valid=True;packet.max_steering_rate=1.57;packet.max_wheel_rate=20.;packet.max_speed=.2;packet.max_accel=.3;packet.environment_margin=.06
        packet.chassis_geometry=[.095,.56,.27,.085,2.,2.5,.01,math.pi,1.]
        packet.footprint=[Point(x=x,y=y)for x,y in [(-.5,-.22),(.5,-.22),(.5,.22),(-.5,.22)]]
        packet.ground_plane=[0.,0.,0.];packet.window_bounds=[-5.,-5.,5.,5.];packet.rotation_observed=True
        packet.scene_radius=4.;packet.hold_position=hold;packet.obstacles=obstacles([])
        s=Bspline();s.order=3;s.traj_id=1;s.goal_id=goal_id;s.start_time=g.header.stamp
        # Nominal 0.4 s domain intentionally shorter than actual traversal.
        s.knots=[0.,0.,0.,0.,.4,.4,.4,.4]
        s.pos_pts=[Point(x=pose[0]+(x-pose[0])*f,y=pose[1]+(y-pose[1])*f,z=.775)for f in (0.,1/3,2/3,1.)]
        packet.spline=s
        current.clear()
    def until(predicate,timeout=30):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            step(.04)
            if predicate() and (not goal_id or current.get("goal_id")==goal_id):return
        raise AssertionError(f'timeout; pose={pose}, feedback={current}; log={log.read_text()}')
    results=[]
    try:
        step(1.5)
        goal(0,0,0,hold=True)
        until(lambda:current.get('start_ready'))
        assert not any(abs(v)+abs(w)>1e-8 for _,v,w in outputs), 'disabled must not move'
        pubs['/n01/navigation/enable'].publish(Bool(data=True))
        until(lambda:current.get('state')=='ARRIVED')
        results.append({'case':'recorded A: hold, stop and measured confirmation','passed':True})
        step(1.)
        goal(1,0,math.pi/2)
        until(lambda:current.get('state')=='MOVING')
        clock=current['execution_time_s'];step(.3)
        assert current['execution_time_s']>clock,'duplicate certificates must not reset clock'
        assert current['trajectory_id']==1
        until(lambda:current.get('state')=='ARRIVED',timeout=25)
        assert math.hypot(pose[0]-1,pose[1])<=.1 and abs(math.atan2(math.sin(pose[2]-math.pi/2),math.cos(pose[2]-math.pi/2)))<.04
        results.append({'case':'A to B 1m, align 90deg, source time exceeded without expiry','passed':True,'pose':list(pose)})
        travelled=current['travelled_m'];assert .85<travelled<1.2,'must integrate twist, not doubled raw pose'
        step(1.)
        goal(0,0,0)
        until(lambda:current.get('state')=='ARRIVED',timeout=35)
        assert math.hypot(pose[0],pose[1])<=.1
        results.append({'case':'return original A, turn before forward movement, final yaw','passed':True,'pose':list(pose)})
        # Stale scene must stop, but may recover once fresh evidence returns.
        goal(1,0,0);until(lambda:current.get('state')=='MOVING')
        publish_scene=False;step(.85)
        assert abs(command[0])+abs(command[1])<1e-8
        assert current['state']=='WAITING_FOR_FEEDBACK'
        publish_scene=True;until(lambda:current.get('state')=='MOVING')
        results.append({'case':'scene loss stops; fresh scene resumes same task','passed':True})
        old_revision=current['trajectory_id'];clock=current['execution_time_s']
        packet.revision=99;packet.spline.traj_id=99;packet.valid=False
        step(.85)
        assert current['state']=='MOVING' and current['trajectory_id']==old_revision
        assert current['execution_time_s']>clock
        results.append({'case':'rejected replacement refreshes scene without revoking valid held segment','passed':True})
        packet.revision=old_revision;packet.spline.traj_id=old_revision;packet.valid=True
        goal(2.,pose[1],0.);until(lambda:current.get('state')=='MOVING')
        packet.obstacles=obstacles([Point(x=pose[0]+.57,y=pose[1],z=.4)])
        before=list(pose);step(.6)
        assert abs(command[0])<1e-8 and math.hypot(pose[0]-before[0],pose[1]-before[1])<.03
        results.append({'case':'new obstacle in braking corridor stops corrected command','passed':True})
        packet.obstacles=obstacles([]);until(lambda:current.get('state')=='MOVING')
        # Equivalent bypass revisions must not evict a safe current segment.
        packet.revision=2;packet.spline.traj_id=2
        packet.spline.pos_pts[1].y+=.12;packet.spline.pos_pts[2].y+=.12
        step(.25);assert current['trajectory_id']==1
        packet.revision=3;packet.spline.traj_id=3
        packet.spline.pos_pts[1].y-=.24;packet.spline.pos_pts[2].y-=.24
        step(.25);assert current['trajectory_id']==1
        results.append({'case':'alternating left/right revisions retain safe active geometry','passed':True})
        goal(pose[0]-1.,pose[1],0.)
        packet.obstacles=obstacles([Point(x=pose[0]+.40,y=pose[1]+.35,z=.4)])
        step(.8)
        assert current['state']=='ALIGN' and 'occupied'in current['reason']
        assert abs(command[0])+abs(command[1])<1e-8
        results.append({'case':'blocked rotation requests SCAN recovery without blind rotation','passed':True})
        packet.obstacles=obstacles([]);until(lambda:current.get('state')=='ROTATING')
        # External traffic, including a zero, latches manual priority without fighting it.
        external.publish(Twist());step(.3);assert current['state']=='MANUAL_OVERRIDE'
        count=len(outputs);step(.4);assert len(outputs)==count,'controller must yield all output'
        results.append({'case':'manual zero command latches ownership; no autonomous output','passed':True})
        step(1.1)
        pubs['/n01/navigation/enable'].publish(Bool(data=False));step(.1)
        pubs['/n01/navigation/enable'].publish(Bool(data=True));until(lambda:current.get('state')=='ROTATING')
        emergency.publish(Bool(data=True));step(.3)
        assert current['state']=='ESTOP' and abs(command[0])+abs(command[1])<1e-8
        results.append({'case':'explicit manual resume and latched emergency stop','passed':True})
        report={'environment':'SYNTHETIC isolated ROS domain 215, real C++ controller and SDK, no hardware',
            'cases':results,'states':dict(Counter(s['state']for s in statuses)),
            'control_max_ms':max(s['control_max_ms']for s in statuses),'statuses':len(statuses),
            'manual_ownership_verified':True,'raw_odometry_pose_scale':2.}
        report_path=Path(os.environ.get('N01_CONTROLLER_REPORT',str(tmp_path/'result.json')))
        report_path.parent.mkdir(parents=True,exist_ok=True);report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    finally:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        stream.close();node.destroy_node();rclpy.shutdown()


def test_real_monitor_service_and_typed_handoff():
    """Real ROS monitor/service codec with declared synthetic scene, no cmd output."""
    assert os.environ.get('ROS_DOMAIN_ID')=='215'
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import Point,Point32,PolygonStamped,PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState,PointCloud2,PointField
    from std_msgs.msg import String
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterValue
    from scan_planner_msgs.msg import Bspline,NavigationTrajectory
    from n01_live_navigation.trajectory_monitor import Hive2TrajectoryMonitor
    from n01_live_navigation.runtime_chassis_model import FIELDS,LIMIT_FIELDS
    from n01_live_navigation.hive2_kinematics import controller_geometry
    rclpy.init();source=rclpy.create_node('synthetic_admission_sources');base=controller_geometry()
    values={key:float(getattr(base,field))for key,field in FIELDS.items()}
    values.update({'chassis.steer.max_speed':1.57,'chassis.drive.max_speed':20.,'chassis.omega_per_linear':base.max_angular_speed/base.max_linear_speed})
    def parameters(request,response):
        response.values=[ParameterValue(type=3,double_value=values[name])for name in request.names];return response
    source.create_service(GetParameters,'/chassis_controller/get_parameters',parameters)
    monitor=Hive2TrajectoryMonitor();executor=SingleThreadedExecutor();executor.add_node(source);executor.add_node(monitor)
    pubs={}
    for topic,typ in [('/body_pose',Odometry),('/joint_states',JointState),('/planning/bspline',Bspline),
        ('/grid_map/occupancy',PointCloud2),('/n01/ground_reference/status',String),('/n01/live_navigation/goal_gated',PoseStamped)]:
        pubs[topic]=source.create_publisher(typ,topic,10)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    polygon_pub=source.create_publisher(PolygonStamped,'/n01/robot_footprint',qos)
    window_pub=source.create_publisher(String,'/grid_map/static_overlay_status',qos)
    received=[];source.create_subscription(NavigationTrajectory,'/n01/navigation/trajectory',received.append,10)
    stamp=source.get_clock().now().to_msg();goal=PoseStamped();goal.header.frame_id='map';goal.header.stamp=stamp;goal.pose.position.x=1.
    bs=Bspline(order=3,traj_id=1,goal_id=stamp.sec*10**9+stamp.nanosec,start_time=stamp,
        knots=[0.,0.,0.,0.,20.,20.,20.,20.],pos_pts=[Point(x=x,z=.775)for x in (0.,.33,.67,1.)])
    try:
        deadline=time.monotonic()+7
        while time.monotonic()<deadline and not any(p.valid for p in received):
            now=source.get_clock().now().to_msg()
            pose=Odometry();pose.header.frame_id='map';pose.header.stamp=now;pose.child_frame_id='base_link';pose.pose.pose.position.z=.775;pose.pose.pose.orientation.w=1.
            pubs['/body_pose'].publish(pose)
            j=JointState();j.header.stamp=now
            j.name=[f'{leg}_leg_pitch1'for leg in ('fl','fr','rl','rr')]+[f'{leg}_leg_pitch2'for leg in ('fl','fr','rl','rr')]+[f'{leg}_ankle_yaw'for leg in ('fl','fr','rl','rr')]
            j.position=[math.pi/4]*4+[-5*math.pi/12]*4+[0.]*4;pubs['/joint_states'].publish(j)
            polygon=PolygonStamped();polygon.header.stamp=now;polygon.header.frame_id='base_link'
            polygon.polygon.points=[Point32(x=x,y=y)for x,y in [(-.5,-.2),(.5,-.2),(.5,.2),(-.5,.2)]];polygon_pub.publish(polygon)
            cloud=PointCloud2();cloud.header.frame_id='map';cloud.header.stamp=now;cloud.height=1;cloud.width=1
            cloud.fields=[PointField(name=name,offset=i*4,datatype=7,count=1)for i,name in enumerate(('x','y','z'))]
            cloud.point_step=12;cloud.row_step=12;cloud.data=struct.pack('<fff',2.,2.,.4);pubs['/grid_map/occupancy'].publish(cloud)
            window_pub.publish(String(data=json.dumps(dict(ready=True,source_frame='map',window_min_xyz=[-5,-5,0],window_max_xyz=[5,5,2]))))
            pubs['/n01/ground_reference/status'].publish(String(data=json.dumps(dict(state='ground_projection_ready',plane=dict(a=0.,b=0.,c=0.,residual=0.)))))
            pubs['/n01/live_navigation/goal_gated'].publish(goal);pubs['/planning/bspline'].publish(bs)
            executor.spin_once(timeout_sec=.01)
            for _ in range(12):executor.spin_once(timeout_sec=.002)
        packets=[p for p in received if p.valid]
        assert packets,f'no admission; model={monitor.native_model_sync.error}; review={monitor.last_error}'
        p=packets[-1]
        assert p.goal_id==bs.goal_id and p.spline.goal_id==bs.goal_id and p.map_id
        assert p.obstacles.header.frame_id=='map' and len(p.obstacles.data)==12
        assert p.max_steering_rate==1.57 and p.max_wheel_rate==20. and len(p.chassis_geometry)==9
        assert len(p.samples)>3 and not p.rotation_observed
    finally:
        executor.shutdown();monitor.destroy_node();source.destroy_node();rclpy.shutdown()
