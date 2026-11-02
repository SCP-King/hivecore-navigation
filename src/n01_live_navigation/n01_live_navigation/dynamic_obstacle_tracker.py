"""Visualization-only unmatched-return tracker; native SCAN remains the planner obstacle source."""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .core import decode_pointcloud_xyz, cluster_xy_points


class DynamicObstacleTracker(Node):
    def __init__(self):
        super().__init__("n01_live_dynamic_obstacle_tracker")
        self.declare_parameter("cloud_topic", "/cloud")
        self.declare_parameter("static_topic", "/grid_map/static_occupancy")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("ground_z_map_m", -0.55)
        self.declare_parameter("body_height_m", 0.90)
        self.declare_parameter("voxel_size_m", 0.30)
        self.declare_parameter("cluster_cell_m", 0.35)
        self.declare_parameter("min_cluster_points", 6)
        self.declare_parameter("max_points", 100000)
        self.declare_parameter("track_timeout_s", 1.0)
        self.declare_parameter("association_distance_m", 0.8)
        get = lambda name: self.get_parameter(name).value
        self.map_frame = str(get("map_frame"))
        self.ground_z = float(get("ground_z_map_m"))
        self.body_height = float(get("body_height_m"))
        self.voxel = float(get("voxel_size_m"))
        self.cluster_cell = float(get("cluster_cell_m"))
        self.min_points = int(get("min_cluster_points"))
        self.max_points = int(get("max_points"))
        self.track_timeout = float(get("track_timeout_s"))
        self.association_distance = float(get("association_distance_m"))
        if not (
            self.map_frame == "map" and math.isfinite(self.ground_z)
            and 0.5 <= self.body_height <= 1.5
            and 0.1 <= self.voxel <= 1.0
            and 0.1 <= self.cluster_cell <= 1.0
            and 3 <= self.min_points <= 100
            and 100 <= self.max_points <= 300000
            and 0.2 <= self.track_timeout <= 5.0
            and 0.2 <= self.association_distance <= 3.0
        ):
            raise ValueError("dynamic obstacle tracker parameters outside bounds")

        self.static_voxels = set()
        self.static_ready = False
        self.tracks = {}
        self.next_track_id = 1
        self.cloud_count = 0
        self.cluster_count = 0
        self.last_candidate_count = 0
        self.last_static_match_count = 0
        self.last_error = ""

        self.create_subscription(
            PointCloud2, str(get("static_topic")), self.on_static, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, str(get("cloud_topic")), self.on_cloud, qos_profile_sensor_data)
        self.candidate_pub = self.create_publisher(
            PointCloud2, "/n01/live_navigation/dynamic_obstacle_candidates", qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/n01/live_navigation/dynamic_obstacle_tracks", 10)
        self.diag_pub = self.create_publisher(
            String, "/n01/live_navigation/dynamic_obstacle_status", 10)
        self.create_timer(0.2, self.expire_tracks)

    def _voxel_keys(self, points):
        keys = np.floor(np.asarray(points) / self.voxel).astype(np.int64)
        return [tuple(int(v) for v in row) for row in keys]

    def on_static(self, msg):
        if msg.header.frame_id.lstrip("/") != self.map_frame:
            self.last_error = "static_map_frame_mismatch"
            return
        try:
            points = decode_pointcloud_xyz(msg, 2000000, None)
            self.static_voxels = set(self._voxel_keys(points))
            self.static_ready = bool(self.static_voxels)
            self.last_error = ""
        except (ValueError, OverflowError) as exc:
            self.static_ready = False
            self.last_error = str(exc)

    @staticmethod
    def _cloud_message(points, header):
        pts = np.asarray(points, dtype="<f4").reshape((-1, 3))
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(pts.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        msg.data = pts.tobytes(order="C")
        return msg

    def _associate(self, clusters, now):
        assigned = set()
        active = {}
        for cluster in clusters:
            centroid = np.mean(cluster, axis=0)
            best_id, best_distance = None, self.association_distance
            for track_id, track in self.tracks.items():
                if track_id in assigned or now - track["last_seen"] > self.track_timeout:
                    continue
                distance = float(np.linalg.norm(centroid[:2] - track["centroid"][:2]))
                if distance < best_distance:
                    best_id, best_distance = track_id, distance
            if best_id is None:
                best_id = self.next_track_id
                self.next_track_id += 1
                velocity = [0.0, 0.0, 0.0]
            else:
                old = self.tracks[best_id]
                dt = now - old["last_seen"]
                if dt >= 0.02:
                    measured = (centroid - old["centroid"]) / dt
                    velocity = measured.tolist() if float(np.linalg.norm(measured)) <= 3.0 else [0.0, 0.0, 0.0]
                else:
                    velocity = old["velocity"]
            assigned.add(best_id)
            active[best_id] = {
                "centroid": centroid,
                "minimum": np.min(cluster, axis=0),
                "maximum": np.max(cluster, axis=0),
                "velocity": velocity,
                "last_seen": now,
                "points": int(cluster.shape[0]),
            }
        expired = {
            track_id for track_id, track in self.tracks.items()
            if now - track["last_seen"] > self.track_timeout
        }
        for track_id, track in self.tracks.items():
            if track_id not in assigned and track_id not in expired:
                active[track_id] = track
        self.tracks = active
        return expired

    def expire_tracks(self):
        now = time.monotonic()
        expired = {
            track_id for track_id, track in self.tracks.items()
            if now - track["last_seen"] > self.track_timeout
        }
        if expired:
            self.tracks = {
                track_id: track for track_id, track in self.tracks.items()
                if track_id not in expired
            }
            self._publish_markers(self.get_clock().now().to_msg(), expired)

    def _publish_markers(self, stamp, expired):
        array = MarkerArray()
        for track_id in sorted(expired):
            for marker_id, namespace in (
                (track_id, "n01_dynamic_candidates"),
                (track_id + 100000, "n01_dynamic_candidates_labels"),
            ):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = stamp
                marker.ns = namespace
                marker.id = marker_id
                marker.action = Marker.DELETE
                array.markers.append(marker)
        for track_id, track in sorted(self.tracks.items()):
            center = track["centroid"]
            extent = np.maximum(track["maximum"] - track["minimum"], 0.15)
            cube = Marker()
            cube.header.frame_id = self.map_frame
            cube.header.stamp = stamp
            cube.ns = "n01_dynamic_candidates"
            cube.id = track_id
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.orientation.w = 1.0
            cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = map(float, center)
            cube.scale.x, cube.scale.y, cube.scale.z = map(float, extent)
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = 1.0, 0.25, 0.05, 0.55
            array.markers.append(cube)

            label = Marker()
            label.header = cube.header
            label.ns = cube.ns + "_labels"
            label.id = track_id + 100000
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.orientation.w = 1.0
            label.pose.position.x = float(center[0])
            label.pose.position.y = float(center[1])
            label.pose.position.z = float(track["maximum"][2] + 0.25)
            label.scale.z = 0.20
            label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 0.9
            label.text = "candidate %d" % track_id
            array.markers.append(label)
        self.marker_pub.publish(array)

    def on_cloud(self, msg):
        self.cloud_count += 1
        if msg.header.frame_id.lstrip("/") != self.map_frame:
            self.last_error = "live_cloud_frame_mismatch"
            return
        if not self.static_ready:
            self.candidate_pub.publish(self._cloud_message(np.empty((0, 3)), msg.header))
            delete_all = MarkerArray()
            clear_marker = Marker()
            clear_marker.header.frame_id = self.map_frame
            clear_marker.header.stamp = msg.header.stamp
            clear_marker.action = Marker.DELETEALL
            delete_all.markers.append(clear_marker)
            self.marker_pub.publish(delete_all)
            self.tracks.clear()
            self.diag_pub.publish(String(data=json.dumps({
                "schema": "n01.dynamic_obstacle_status.v1",
                "status": "WAITING_STATIC_MAP",
                "visualization_only": True,
                "planner_input": "unfiltered /cloud remains connected directly to native SCAN",
            }, sort_keys=True)))
            return
        try:
            points = decode_pointcloud_xyz(msg, self.max_points, 20.0)
        except (ValueError, OverflowError) as exc:
            self.last_error = str(exc)
            return
        z_min = self.ground_z + 0.08
        z_max = self.ground_z + self.body_height + 0.30
        band = points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)]
        keys = self._voxel_keys(band)
        unmatched_mask = np.fromiter(
            (key not in self.static_voxels for key in keys),
            dtype=np.bool_, count=len(keys))
        candidates = band[unmatched_mask]
        self.last_static_match_count = int(len(band) - len(candidates))
        self.last_candidate_count = int(len(candidates))
        self.candidate_pub.publish(self._cloud_message(candidates, msg.header))

        clusters = cluster_xy_points(
            candidates, self.cluster_cell, self.min_points, max_clusters=64)
        now = time.monotonic()
        expired = self._associate(clusters, now)
        self.cluster_count = len(clusters)
        self._publish_markers(msg.header.stamp, expired)
        self.last_error = ""
        summary = {
            "schema": "n01.dynamic_obstacle_status.v1",
            "status": "CANDIDATES_AVAILABLE" if clusters else "NO_UNMATCHED_CLUSTERS",
            "cloud_frame": self.map_frame,
            "z_band_map_m": [z_min, z_max],
            "static_voxel_size_m": self.voxel,
            "candidate_voxel_match": "exact_coarse_voxel_match",
            "static_matched_points": self.last_static_match_count,
            "unmatched_candidate_points": self.last_candidate_count,
            "cluster_count": self.cluster_count,
            "tracks": [
                {
                    "id": int(track_id),
                    "centroid_map": track["centroid"].tolist(),
                    "velocity_map_mps": track["velocity"],
                    "points": track["points"],
                }
                for track_id, track in sorted(self.tracks.items())
            ],
            "visualization_only": True,
            "planner_input": "raw /cloud remains connected directly to native SCAN",
            "actuator_output": "DISABLED",
            "safety_authorization": "NEVER",
        }
        self.diag_pub.publish(String(data=json.dumps(summary, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
