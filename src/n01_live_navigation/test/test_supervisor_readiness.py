"""Isolated ROS-domain regression: no real robot publishers or subscribers."""
import json,time
import rclpy
from std_msgs.msg import String
from n01_live_navigation.runtime_supervisor import LiveRuntimeSupervisor


def test_supervisor_stage_consistency_and_bridge_heartbeat_expiry(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','215')
    rclpy.init()
    n=LiveRuntimeSupervisor()
    try:
        b=dict(status='LIVE_SCAN_INPUTS_READY',localization_status_fresh=True,
            robot_pose_receive_fresh=True,pose_scan_anchor_available=True,
            odom_receive_fresh=True,motion_compensation_ready=True,
            sensor_extrinsic_ready=True,cloud_output_fresh=True)
        n.on_bridge(String(data=json.dumps(b)))
        n.body_stamp_ns=n.get_clock().now().nanoseconds;n.body_rx_monotonic=time.monotonic()
        n.static_ready=True;n.robot_feasibility_rx_monotonic=time.monotonic()
        n.robot_feasibility=dict(status='ROTATION_STAGE_READY',candidate_review_ready=True)
        n.publish_health()
        assert n.last_health['navigation_ready']
        assert n.last_health['stages']['C_hive2_candidate_kinematics']['ready_for_review']
        n.bridge_rx_monotonic-=2.
        n.publish_health()
        assert not n.last_health['navigation_ready']
        assert n.last_health['blockers']==['bridge_status_missing_or_stale']
    finally:
        n.destroy_node();rclpy.shutdown()
