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
        self.assertIn('"read_only": True', source)
        self.assertIn('"execution_authorized": False', source)
        self.assertNotIn("create_publisher(", source)
        self.assertNotIn("create_client(", source)

    def test_four_stage_runbook_exists(self):
        source = (ROOT / "docs/n01_four_stage_navigation_delivery.md").read_text()
        for marker in ("## A.", "## B.", "## C.", "## D."):
            self.assertIn(marker, source)


if __name__ == "__main__":
    unittest.main()
