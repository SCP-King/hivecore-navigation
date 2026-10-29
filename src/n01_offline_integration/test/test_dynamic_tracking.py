"""Deterministic offline obstacle/trajectory-prediction tests (no ROS required)."""
import math
import pathlib
import sys
import unittest

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from n01_offline_integration.dynamic_tracking_core import (
    Tracker, clusters_from_points,
)


def cloud_cluster(center_x, center_y, size=.20, z=.7):
    # nine occupied cells plus repeats so small voxel components are detectable
    return [(center_x+dx,center_y+dy,z)
            for dx in (-size,0,size)
            for dy in (-size,0,size) for _ in range(3)]


def detect(points):
    return clusters_from_points(
        points, body_xy=(0,0,.9),cell_size=.25,radius=9.,
        min_cells=2,max_cells=60,min_points=6)


class DynamicTrackingTests(unittest.TestCase):
    def test_stationary_obstacle_not_promoted(self):
        tracker=Tracker(min_hits=3)
        for i in range(6):
            rows=detect(cloud_cluster(2.0,1.0))
            self.assertGreater(len(rows),0)
            state=tracker.update((i+1)*250_000_000, rows)
        self.assertEqual(state["moving_hypotheses"],0)
        self.assertTrue(all(t["classification"]!="MOVING_HYPOTHESIS_UNVERIFIED"
                            for t in state["tracks"]))

    def test_moving_target_constant_velocity_horizon(self):
        tracker=Tracker(min_hits=3,motion_threshold_mps=.18)
        for i in range(6):
            rows=detect(cloud_cluster(2.+i*.2,0.))
            self.assertTrue(rows)
            result=tracker.update((i+1)*250_000_000,rows)
        self.assertGreater(result["moving_hypotheses"],0)
        moving=[t for t in result["tracks"]
                if t["classification"]=="MOVING_HYPOTHESIS_UNVERIFIED"]
        self.assertEqual(len(moving),1)
        self.assertGreater(moving[0]["velocity_xy_mps"][0],.3)
        self.assertEqual(len(moving[0]["predicted_positions"]),3)
        self.assertGreater(moving[0]["predicted_positions"][-1]["xyz_map_m"][0],
                           moving[0]["position_map_m"][0])

    def test_ego_movement_in_map_frame_not_dynamic(self):
        tracker=Tracker(min_hits=3)
        for i in range(6):
            # A static obstacle stays at world (2,1) while the robot moves.
            points=cloud_cluster(2.0,1.0)
            rows=clusters_from_points(points,(i*.2,0.,.9),cell_size=.25,
                                     min_cells=2,min_points=6)
            result=tracker.update((i+1)*250_000_000,rows)
        self.assertEqual(result["moving_hypotheses"],0)

    def test_backward_timestamp_fails_closed(self):
        t=Tracker()
        c=detect(cloud_cluster(2,1))
        t.update(2_000_000_000,c)
        out=t.update(1_000_000_000,c)
        self.assertEqual(out["reason"],"nonmonotonic_timestamp_reset")
        self.assertEqual(out["moving_hypotheses"],0)
        self.assertEqual(out["clock_resets"],1)

    def test_lost_target_not_extrapolated_as_live(self):
        t=Tracker(max_gap_s=.5)
        rows=detect(cloud_cluster(2,1))
        for i in range(4):
            t.update((i+1)*250_000_000,rows)
        out=t.update(2_000_000_000,[])
        self.assertFalse(out["tracks"])

    def test_invalid_cloud_geometry_rejected(self):
        with self.assertRaises(ValueError):
            clusters_from_points([(1,2,.8)],(0,0,.9),cell_size=.001)
        self.assertEqual(detect([(float("nan"),1,.9)]),[])

    def test_output_schema_not_controller(self):
        result=Tracker().update(1_000_000_000,detect(cloud_cluster(2,1)))
        self.assertEqual(result["frame_id"],"map")
        self.assertFalse(result["kalman_prediction_implemented"])
        self.assertFalse(result["collision_safety_verified"])
        self.assertFalse(result["motion_commands_enabled"])


if __name__=="__main__":
    unittest.main()
