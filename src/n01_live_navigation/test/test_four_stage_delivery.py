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

    def test_four_stage_runbook_exists(self):
        source = (ROOT / "docs/n01_four_stage_navigation_delivery.md").read_text()
        for marker in ("## A.", "## B.", "## C.", "## D."):
            self.assertIn(marker, source)


if __name__ == "__main__":
    unittest.main()
