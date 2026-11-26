"""Real planner, declared synthetic map/pose in isolated domain 216. No cmd_vel."""
import os
import pytest
pytestmark=pytest.mark.skipif(os.getenv('N01_PLANNER_INTEGRATION')!='1',reason='explicit isolated planner test only')


def test_negative_map_goal_and_immediate_global_failure(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID')=='216'
    import json,time,signal,subprocess,struct
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped,PolygonStamped,Point32
    from nav_msgs.msg import Path,Odometry,OccupancyGrid
    from sensor_msgs.msg import PointCloud2,PointField
    from std_msgs.msg import String
    rclpy.init();n=rclpy.create_node('synthetic_planner_test')
    q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    model=n.create_publisher(PolygonStamped,'/n01/robot_footprint',q)
    maps=n.create_publisher(PointCloud2,'/grid_map/static_occupancy',q)
    body=n.create_publisher(Odometry,'/body_pose',qos_profile_sensor_data)
    cloudpub=n.create_publisher(PointCloud2,'/cloud',qos_profile_sensor_data)
    sensor=n.create_publisher(Odometry,'/sensor_pose',qos_profile_sensor_data)
    goals=n.create_publisher(PoseStamped,'/move_base_simple/goal',10)
    paths=[];statuses=[];previews=[];references=[]
    n.create_subscription(OccupancyGrid,"/grid_map/global_grid",references.append,q)
    n.create_subscription(OccupancyGrid,"/planning/search_grid",previews.append,q)
    n.create_subscription(Path,'/planning/global_path',lambda m:paths.append(m),q)
    n.create_subscription(String,'/planning/status',lambda m:statuses.append(json.loads(m.data)),q)
    stream=(tmp_path/'planner.log').open('w')
    config=get_package_share_directory('scan_planner')+'/config/planner.yaml'
    process=subprocess.Popen(['ros2','run','scan_planner','scan_planner_node','--ros-args','--params-file',config,
        '-p','grid_map.frame_id:=map','-p','grid_map.static_cloud_enabled:=true',
        '-p','grid_map.reference_updates_enabled:=false','-p','grid_map.fused_2d_enabled:=true',
        '-p','grid_map.static_observation_refresh:=true','-p','grid_map.static_prior_as_cost:=false',
        '-p','grid_map.ground_height:=-0.8'],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    def cloud(points,stamp):
        m=PointCloud2();m.header.frame_id='map';m.header.stamp=stamp;m.height=1;m.width=len(points)
        m.fields=[PointField(name=v,offset=i*4,datatype=7,count=1)for i,v in enumerate(('x','y','z'))]
        m.point_step=12;m.row_step=len(points)*12;m.data=b''.join(struct.pack('<fff',*p)for p in points);return m
    goal=PoseStamped();goal.header.frame_id='map';goal.header.stamp=n.get_clock().now().to_msg();goal.pose.position.x=.6;goal.pose.position.y=.2;goal.pose.position.z=-.8;goal.pose.orientation.w=1.
    def tick(block=False, send_goal=True):
        stamp=n.get_clock().now().to_msg()
        p=Odometry();p.header.frame_id='map';p.header.stamp=stamp;p.child_frame_id='base_link';p.pose.pose.position.z=-.13;p.pose.pose.orientation.w=1.
        body.publish(p)
        polygon=PolygonStamped();polygon.header.frame_id='base_link';polygon.header.stamp=stamp
        polygon.polygon.points=[Point32(x=x,y=y)for x,y in [(-.5,-.2),(.5,-.2),(.5,.2),(-.5,.2)]];model.publish(polygon)
        maps.publish(cloud([(-2.,-2.,.6),(2.,2.,.6)],stamp))
        if block:sensor.publish(p);cloudpub.publish(cloud([(.6,.2,-.13)],stamp))
        if send_goal:goals.publish(goal)
        end=time.monotonic()+.08
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.005)
        assert process.poll()is None,(tmp_path/'planner.log').read_text()
    try:
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and not (previews and references):tick(send_goal=False)
        assert previews and references and not paths
        original_grid = bytes((v+256)%256 for v in references[0].data)
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and not paths:tick()
        assert paths,'negative-z RViz map goal did not produce a global reference'
        assert abs(paths[-1].poses[-1].pose.position.x-.6)<1e-6
        assert abs(paths[-1].poses[-1].pose.position.z+.13)<1e-6,'goal height must use physical body reference'
        # Keep the old goal during obstacle delivery, then request a new ID.
        for _ in range(4):tick(True)
        goal.header.stamp=n.get_clock().now().to_msg();wanted=goal.header.stamp.sec*10**9+goal.header.stamp.nanosec
        deadline=time.monotonic()+3
        while time.monotonic()<deadline and not any(s.get('goal_id')==wanted and s.get('phase')=='GLOBAL_FAILED'for s in statuses):tick(True)
        match=[s for s in statuses if s.get('goal_id')==wanted and s.get('phase')=='GLOBAL_FAILED']
        assert match and match[-1]['reason'] in {'goal_occupied', 'goal_footprint_occupied'},statuses
        assert all(bytes((v+256)%256 for v in m.data)==original_grid for m in references)
    finally:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        stream.close();n.destroy_node();rclpy.shutdown()
