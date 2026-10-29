"""RViz-only visualization of the REAL isolated historical Bag G1-G3 chain.

No actuator, no TF publication, no robot SDK. Marker and Path publishers only.
G3 POST-HOC outputs are labeled and paced; recorded clock is never presented
as a contemporary robot motion command or a safety clearance.
"""
from collections import OrderedDict, deque
import json
import math
import os

if (os.environ.get("ROS_DOMAIN_ID")!="88"
        or os.environ.get("ROS_LOCALHOST_ONLY")!="1"
        or os.environ.get("N01_HISTORICAL_BAG_ONLY")!="1"):
    raise RuntimeError("N01 demo markers require isolated historical Bag replay")

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .demo_geometry import preview_tracks, predict_xy, reference_result


def make_point(x,y,z):
    p=Point()
    p.x=float(x)
    p.y=float(y)
    p.z=float(z)
    return p


class OfflineDemoMarkers(Node):
    def __init__(self):
        super().__init__("n01_offline_demo_markers")
        self.track_cache=OrderedDict()
        self.queued_scenes=deque(maxlen=128)
        self.last_scene=None
        self.live_pose=None
        self.latest_tracks=None
        self.frames_displayed=0
        self.recorded_g3=0
        self.nominal_pub=self.create_publisher(
            Path,"/n01/offline/demo_nominal_path",10)
        self.candidate_pub=self.create_publisher(
            Path,"/n01/offline/demo_candidate_path",10)
        self.marker_pub=self.create_publisher(
            MarkerArray,"/n01/offline/demo_markers",10)
        self.create_subscription(Odometry,"/body_pose",self.on_body,
                                 qos_profile_sensor_data)
        self.create_subscription(String,"/n01/offline/dynamic_tracks",
                                 self.on_tracks,10)
        self.create_subscription(String,"/n01/offline/g3_solver_report",
                                 self.on_g3,10)
        self.create_timer(.35,self.tick)
        self.get_logger().warning(
            "RViz display only: all tracks unverified, POST-HOC historical "
            "path results are NOT contemporaneous control or safety evidence.")

    def on_body(self,msg):
        if msg.header.frame_id.lstrip("/")=="map":
            p=msg.pose.pose.position
            if all(math.isfinite(v) for v in (p.x,p.y,p.z)):
                self.live_pose=(p.x,p.y,p.z)

    def on_tracks(self,msg):
        try:
            row=json.loads(msg.data)
            if (not isinstance(row,dict) or
                    row.get("schema")!="n01.offline.tracks.v1" or
                    row.get("status")!="PROCESSED"):
                return
            stamp=row.get("stamp_ns")
            if type(stamp) is not int or stamp<=0:
                return
            clean=preview_tracks(row)
            self.track_cache[stamp]=clean
            self.track_cache.move_to_end(stamp)
            while len(self.track_cache)>1024:
                self.track_cache.popitem(last=False)
            self.latest_tracks=clean
        except (ValueError,TypeError):
            pass

    def on_g3(self,msg):
        try:
            scene=reference_result(json.loads(msg.data))
        except (ValueError,TypeError):
            return
        if scene is not None:
            self.queued_scenes.append(scene)
            self.recorded_g3+=1

    def path_msg(self,xy,z,now):
        msg=Path()
        msg.header.frame_id="map"
        msg.header.stamp=now
        for x,y in xy:
            pose=PoseStamped()
            pose.header=msg.header
            pose.pose.position.x=x
            pose.pose.position.y=y
            pose.pose.position.z=z
            pose.pose.orientation.w=1.
            msg.poses.append(pose)
        return msg

    def marker(self,ns,mid,kind,now,xyz=(0.,0.,.9),
               rgb=(1.,1.,1.),scale=(.2,.2,.2),alpha=1.,
               text="",points=()):
        m=Marker()
        m.header.frame_id="map"
        m.header.stamp=now
        m.ns=ns
        m.id=mid
        m.type=kind
        m.action=Marker.ADD
        m.pose.position.x=float(xyz[0])
        m.pose.position.y=float(xyz[1])
        m.pose.position.z=float(xyz[2])
        m.pose.orientation.w=1.
        m.scale.x,m.scale.y,m.scale.z=tuple(float(v) for v in scale)
        m.color.r,m.color.g,m.color.b=tuple(float(v) for v in rgb)
        m.color.a=float(alpha)
        m.text=text
        m.points=[make_point(*p) for p in points]
        return m

    def tick(self):
        now=self.get_clock().now().to_msg()
        if self.queued_scenes:
            self.last_scene=self.queued_scenes.popleft()
            self.frames_displayed+=1
        scene=self.last_scene
        if scene is not None:
            self.nominal_pub.publish(self.path_msg(
                scene["nominal"],scene["z"],now))
            self.candidate_pub.publish(self.path_msg(
                scene["candidate"],scene["z"],now))
        markers=MarkerArray()
        remove=Marker()
        remove.action=Marker.DELETEALL
        markers.markers.append(remove)
        # Coordinates of G3 body and tracks MUST originate from matching
        # recorded stamp. If no matching records exist, don't mix other times.
        if scene:
            bx,by=scene["candidate"][0]
            bz=scene["z"]
            tracks=self.track_cache.get(scene["stamp"],[])
        elif self.live_pose:
            bx,by,bz=self.live_pose
            tracks=self.latest_tracks or []
        else:
            bx=by=0.
            bz=.9
            tracks=[]
        markers.markers.append(self.marker(
            "robot_historical",0,Marker.CUBE,now,
            (bx,by,bz),(.2,.75,1.),(.65,.45,.35),.8))
        markers.markers.append(self.marker(
            "robot_label",0,Marker.TEXT_VIEW_FACING,now,
            (bx,by,bz+.6),(1.,1.,1.),(0.,0.,.27),
            text="HISTORICAL POSE - NO CONTROL"))
        for i,track in enumerate(tracks[:20]):
            x,y=track["xy"]
            vx,vy=track["velocity"]
            ex,ey=track["extent"]
            markers.markers.append(self.marker(
                "moving_hypothesis",i,Marker.CUBE,now,
                (x,y,bz), (1.,.45,.1), (ex,ey,.55),.5))
            markers.markers.append(self.marker(
                "track_id",i,Marker.TEXT_VIEW_FACING,now,
                (x,y,bz+.65), (1.,.75,.35), (0.,0.,.24),
                text="UNVERIFIED #" + track["id"]))
            markers.markers.append(self.marker(
                "velocity_arrow",i,Marker.ARROW,now,
                rgb=(1.,.2,.2),scale=(.055,.12,.12),
                points=[(x,y,bz+.35),(x+vx,y+vy,bz+.35)]))
            predicted=predict_xy(track["xy"],track["velocity"])
            markers.markers.append(self.marker(
                "predicted_motion",i,Marker.LINE_STRIP,now,
                rgb=(1.,.75,.2),scale=(.06,0.,0.),
                points=[(px,py,bz+.2) for px,py in predicted]))
        if scene:
            state="POST-HOC HISTORICAL COMPARISON" if scene["posthoc"] else (
                "REPLAY ORDER (NOT REAL-TIME CONTROL)")
            moving=str(scene["dynamic_constraints"])
            label=(state+" | G3 CBF candidates="+moving+
                   " | recorded stamp "+str(scene["stamp"])+
                   " | "+scene["status"])
        else:
            label="WAITING FOR G3 DATA - OFFLINE ONLY"
        markers.markers.append(self.marker(
            "status",0,Marker.TEXT_VIEW_FACING,now,
            (bx,by,bz+1.4),(.5,1.,.5),(0.,0.,.27),
            text=label[:200]))
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node=OfflineDemoMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=="__main__":
    main()
