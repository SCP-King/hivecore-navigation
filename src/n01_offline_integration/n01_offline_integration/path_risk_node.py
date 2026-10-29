"""G2.3: continuous recorded-time SCAN Path / moving-hypothesis diagnostic.

SCAN reference Path has a wall-time header. Recorded cloud and /body_pose
share historical scan stamps. Their exact stamp pairing is the ONLY timing
authority used here. A single Path is retained as unverified geometry, and
the robot's *recorded* map pose is projected onto it every matched frame.
NO control output, no motion authorization, not a true CBF or MPC solver.
"""
import json
import math
import os
import time

if os.environ.get("ROS_DOMAIN_ID")!="88" or os.environ.get("ROS_LOCALHOST_ONLY")!="1":
    raise RuntimeError("Path risk preview is isolated offline-only")

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

from .path_risk_core import evaluate_path_preview
from .replay_path_sync import ReplayPathSynchronizer


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


class OfflinePathRisk(Node):
    def __init__(self):
        super().__init__("n01_offline_path_risk")
        self.sync=ReplayPathSynchronizer()
        self.path_count=0
        self.track_count=0
        self.pose_count=0
        self.report_count=0
        self.evaluable=0
        self.potential_conflicts=0
        self.last_status="WAITING_FOR_INPUTS"
        self.last_matched_wall=None
        self.last_result=None
        self.min_barrier_proxy=None
        self.worst_track_id=None
        self.worst_at_scan_stamp_ns=None
        self.pub=self.create_publisher(String,"/n01/offline/path_risk",10)
        self.create_subscription(Path,"/nav/test/global_path",self.on_path,10)
        self.create_subscription(Odometry,"/body_pose",self.on_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(String,"/n01/offline/dynamic_tracks",
                                 self.on_tracks,10)
        self.create_timer(1.0,self.tick)
        self.get_logger().warning(
            "G2.3 exact replay-time matching and retained Path geometry; "
            "no robot commands; risk output is NOT safety-certified")

    def on_path(self,msg):
        self.path_count+=1
        if msg.header.frame_id.lstrip("/")!="map":
            self.sync.set_path([],stamp_ns(msg.header.stamp))
            return
        points=[(pose.pose.position.x,pose.pose.position.y) for pose in msg.poses]
        self.sync.set_path(points,stamp_ns(msg.header.stamp))

    def on_pose(self,msg):
        self.pose_count+=1
        if msg.header.frame_id.lstrip("/")!="map":
            return
        p=msg.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):
            return
        candidate=self.sync.put_body(stamp_ns(msg.header.stamp),(p.x,p.y))
        if candidate is not None:
            self._publish_matched(candidate)

    def on_tracks(self,msg):
        self.track_count+=1
        try:
            report=json.loads(msg.data)
        except (ValueError,TypeError):
            return
        candidate=self.sync.put_tracks(report)
        if candidate is not None:
            self._publish_matched(candidate)

    def _publish_matched(self,result):
        self.last_matched_wall=time.monotonic()
        self.last_result=result
        self.last_status=result["status"]
        if result["evaluated_pairs"]>0:
            self.evaluable+=1
            value=result["minimum_barrier_proxy_m2"]
            if value is not None and (
                    self.min_barrier_proxy is None or value<self.min_barrier_proxy):
                self.min_barrier_proxy=value
                self.worst_track_id=result.get("nearest_track_id")
                self.worst_at_scan_stamp_ns=result.get("recorded_scan_stamp_ns")
        if result["status"]=="POTENTIAL_PROXIMITY_CONFLICT":
            self.potential_conflicts+=1
        self._publish(result,frame_event=True)

    def _publish(self,result,frame_event):
        output=dict(result)
        self.report_count+=1
        output.update({
            "path_messages_seen":self.path_count,
            "pose_messages_seen":self.pose_count,
            "tracking_reports_seen":self.track_count,
            "diagnostic_reports_published":self.report_count,
            "evaluated_previews":self.evaluable,
            "potential_conflict_previews":self.potential_conflicts,
            "minimum_barrier_proxy_across_recorded_frames_m2":self.min_barrier_proxy,
            "worst_recorded_frame_track_id":self.worst_track_id,
            "worst_recorded_frame_stamp_ns":self.worst_at_scan_stamp_ns,
            "is_new_aligned_recorded_frame":bool(frame_event),
            "scope":"OPEN_LOOP_OFFLINE_ONLY",
            "trusted_moving_objects":False,
            "offline_path_preview_alive":True,
        })
        output.update(self.sync.evidence())
        self.pub.publish(String(data=json.dumps(output,ensure_ascii=False,
                                                sort_keys=True)))

    def tick(self):
        now=time.monotonic()
        if self.last_matched_wall is None or now-self.last_matched_wall>3.:
            # Preserve cumulative evidence while never pretending an old
            # prediction remains current or that a retained Path is safe.
            result=evaluate_path_preview([],[])
            result["status"]="NO_RECENT_EXACT_TIMESTAMP_PAIRED_FRAME"
            result["reference_provenance"]="HISTORICAL_PATH_GEOMETRY_UNVERIFIED"
            result["time_alignment"]="EXACT_RECORDED_BODY_POSE_AND_TRACK_STAMP_REQUIRED"
            result["path_header_is_historical_stamp"]=False
        else:
            result=dict(self.last_result)
            # Heartbeat is a repetition of the most recent recorded frame,
            # *not* a new independent collision prediction.
            result["status"]="LAST_RECORDED_FRAME_SNAPSHOT_NOT_NEW_EVALUATION"
            result["last_frame_status"]=self.last_status
        self._publish(result,frame_event=False)


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
