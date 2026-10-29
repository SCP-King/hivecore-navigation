"""ROS 2 read-only diagnostic of SCAN reference path vs obstacle hypotheses.

Every output explicitly disclaims safety authorization. No plan rewriting,
cmd_vel, action client, robot interface, Kalman filter, MPC or real CBF solver.
"""
import json
import os
import time

if os.environ.get("ROS_DOMAIN_ID")!="88" or os.environ.get("ROS_LOCALHOST_ONLY")!="1":
    raise RuntimeError("Path risk preview is isolated offline-only")

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String

from .path_risk_core import evaluate_path_preview


class OfflinePathRisk(Node):
    def __init__(self):
        super().__init__("n01_offline_path_risk")
        self.path=None
        self.path_received=None
        self.tracks=None
        self.tracks_received=None
        self.path_count=0
        self.track_count=0
        self.evaluable=0
        self.potential_conflicts=0
        self.last_status="WAITING_FOR_INPUTS"
        self.pub=self.create_publisher(String,"/n01/offline/path_risk",10)
        self.create_subscription(Path,"/nav/test/global_path",self.on_path,10)
        self.create_subscription(String,"/n01/offline/dynamic_tracks",
                                 self.on_tracks,10)
        self.create_timer(1.0,self.tick)

    def on_path(self,msg):
        self.path_count+=1
        if msg.header.frame_id.lstrip("/")!="map":
            self.path=None
            self.path_received=None
            return
        self.path=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        self.path_received=time.monotonic()

    def on_tracks(self,msg):
        self.track_count+=1
        try:
            tracks=json.loads(msg.data)
        except (ValueError,TypeError):
            self.tracks=None
            self.tracks_received=None
            return
        if (isinstance(tracks,dict) and
            tracks.get("schema")=="n01.offline.tracks.v1" and
            tracks.get("frame_id")=="map" and
            tracks.get("status")=="PROCESSED"):
            self.tracks=tracks.get("tracks")
            self.tracks_received=time.monotonic()
        else:
            self.tracks=None
            self.tracks_received=None

    def tick(self):
        now=time.monotonic()
        if self.path_received is None or now-self.path_received>15.:
            result=evaluate_path_preview([],[])
            result["status"]="NO_FRESH_REFERENCE_PATH"
        elif self.tracks_received is None or now-self.tracks_received>3.:
            result=evaluate_path_preview([],[])
            result["status"]="NO_FRESH_MOVING_HYPOTHESES_DATA"
        else:
            try:
                result=evaluate_path_preview(self.path,self.tracks)
            except (TypeError,ValueError,OverflowError):
                result=evaluate_path_preview([],[])
                result["status"]="INVALID_PREVIEW_INPUT"
        self.last_status=result["status"]
        if result["evaluated_pairs"]>0:
            self.evaluable+=1
        if result["status"]=="POTENTIAL_PROXIMITY_CONFLICT":
            self.potential_conflicts+=1
        result.update({
            "path_messages_seen":self.path_count,
            "tracking_reports_seen":self.track_count,
            "evaluated_previews":self.evaluable,
            "potential_conflict_previews":self.potential_conflicts,
            "scope":"OPEN_LOOP_OFFLINE_ONLY",
            "trusted_moving_objects":False,
            "offline_path_preview_alive":True,
        })
        self.pub.publish(String(data=json.dumps(result,ensure_ascii=False,
                                                sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node=OfflinePathRisk()
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
