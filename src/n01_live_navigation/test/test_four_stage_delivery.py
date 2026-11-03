import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


class FourStageDeliveryTests(unittest.TestCase):
    def test_safety_monitor_is_fail_closed(self):
        source = (ROOT / "src/n01_live_navigation/n01_live_navigation/safety_supervisor.py").read_text()
        self.assertIn("evaluate_safety_gate(", source)
        self.assertIn('"/n01/live_navigation/safety_status"', source)
        self.assertIn('"operator_hold": True', source)
        self.assertIn('"emergency_stop_chain_verified": False', source)
        self.assertIn('"actuator_output": "DISABLED"', source)
        self.assertIn('"safety_authorization": "NEVER"', source)

    def test_read_only_live_preflight_audits_time_sync_and_never_publishes(self):
        source = (ROOT / "scripts/n01_live_stack_preflight.py").read_text()
        for topic in (
            '"/odom"', '"/robot_pose"', '"/body_pose"',
            '"/livox/lidar/geo_filtered"', '"/cloud"', '"/sensor_pose"',
            '"/grid_map/static_overlay_ready"',
            '"/n01/live_navigation/system_health"',
        ):
            self.assertIn(topic, source)
        self.assertIn("cloud_sensor_pose_exact_matches", source)
        self.assertIn("receive_rate_hz", source)
        self.assertIn("duplicate_node_names", source)
        self.assertIn("refresh_topic_subscriptions", source)
        self.assertIn("multiple_publisher_endpoints", source)
        self.assertIn("robot_pose_stamp_stale_despite_recent_delivery", source)
        self.assertIn('"read_only": True', source)
        self.assertIn('"execution_authorized": False', source)
        self.assertNotIn("create_publisher(", source)
        self.assertNotIn("create_client(", source)

    def test_four_stage_runbook_exists(self):
        source = (ROOT / "docs/n01_four_stage_navigation_delivery.md").read_text()
        for marker in ("## A.", "## B.", "## C.", "## D."):
            self.assertIn(marker, source)

    def test_dynamic_tracker_uses_sensor_time_and_publishes_invalid_status(self):
        source = (ROOT / "src/n01_live_navigation/n01_live_navigation/dynamic_obstacle_tracker.py").read_text()
        self.assertIn("estimate_velocity(", source)
        self.assertIn("cloud_stamp_ns", source)
        self.assertIn('"status": "INVALID_CLOUD_FRAME"', source)
        self.assertIn('"status": "INVALID_CLOUD"', source)
        self.assertIn('"velocity_valid"', source)

    def test_safety_gate_consumes_dynamic_freshness_and_candidate_state(self):
        source = (ROOT / "src/n01_live_navigation/n01_live_navigation/safety_supervisor.py").read_text()
        self.assertIn("dynamic_obstacle_fresh=dynamic_fresh", source)
        self.assertIn('dynamic_obstacle_status=self.dynamic_status.get("status", "UNKNOWN")', source)
        core = (ROOT / "src/n01_live_navigation/n01_live_navigation/core.py").read_text()
        self.assertIn("dynamic_obstacle_status_missing_or_stale", core)
        self.assertIn("dynamic_obstacle_candidates_require_review", core)

    def test_runtime_health_reports_planner_and_scene_diagnostic_freshness(self):
        source = (ROOT / "src/n01_live_navigation/n01_live_navigation/runtime_supervisor.py").read_text()
        self.assertIn('"scene_diagnostics_ready": scene_diagnostics_ready', source)
        self.assertIn('"/grid_map/static_overlay_status"', source)
        self.assertIn('"static_overlay_diagnostic"', source)
        self.assertIn('"planner_candidate_status"', source)
        self.assertIn('"dynamic_tracker_review_required"', source)
        self.assertIn("stamped_data_is_fresh(", source)
        self.assertIn('"dynamic_tracker_stamp_age_s"', source)
        trajectory = (ROOT / "src/n01_live_navigation/n01_live_navigation/trajectory_monitor.py").read_text()
        self.assertIn('"trajectory_active": trajectory_active', trajectory)
        self.assertIn("trajectory_window_is_active(", trajectory)
        self.assertIn('"TRAJECTORY_NOT_ACTIVE_OR_STALE"', trajectory)
        self.assertIn('"joint_feedback_stamp_age_s": joint_stamp_age', trajectory)
        self.assertIn('"body_pose_frame_match": body_pose_frame_match', trajectory)
        self.assertIn('"WAITING_FOR_BSPLINE"', trajectory)
        self.assertIn('"STALE_BSPLINE"', trajectory)
        self.assertIn("Marker.DELETEALL", trajectory)
        self.assertIn("max_bspline_receive_age_s", trajectory)
        bridge = (ROOT / "src/n01_live_navigation/n01_live_navigation/live_scan_bridge.py").read_text()
        self.assertIn("pose_samples_equivalent(", bridge)
        self.assertIn("robot_pose_publisher_count", bridge)
        self.assertIn("robot_pose_stamp_progress_age_s", bridge)
        self.assertIn("drop_pose_duplicate_conflict", bridge)
        self.assertIn("if conflict_was_latest:", bridge)
        self.assertIn("self.pose_rx_monotonic = 0.0", bridge)
        self.assertIn('"global_robot_pose_missing_or_stale"', source)
        self.assertIn('"global_robot_pose_anchor_outside_scan_time_budget"', source)
        self.assertIn('"bridge_summary": bridge_summary', source)
        self.assertIn('"NAVIGATION BLOCKED "', source)


if __name__ == "__main__":
    unittest.main()
