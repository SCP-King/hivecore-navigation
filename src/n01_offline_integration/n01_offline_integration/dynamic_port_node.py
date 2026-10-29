"""Non-actuating dynamic-avoidance interface placeholder.

Consumes SCAN reference paths and bridge state; advertises explicitly
NOT_IMPLEMENTED/DISARMED. No dynamic obstacle estimator, MPC/CBF solver
or command publisher is installed by this scaffold.
"""
import json
import os
if os.environ.get("ROS_DOMAIN_ID") != "88" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise RuntimeError("Dynamic port is offline-only")
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Path
from std_msgs.msg import String


class DynamicPort(Node):
    def __init__(self):
        super().__init__("n01_dynamic_avoidance_interface")
        self.path_count=0
        self.last_ready="NOT_READY"
        self.status=self.create_publisher(String,"/n01/offline/dynamic_port_status",10)
        self.create_subscription(Path,"/nav/test/global_path",self.on_path,10)
        self.create_subscription(String,"/n01/offline/readiness",self.on_ready,10)
        self.create_timer(1.0,self.tick)

    def on_path(self,msg):
        if msg.poses:
            self.path_count+=1

    def on_ready(self,msg):
        try:
            self.last_ready=json.loads(msg.data).get("readiness","NOT_READY")
        except ValueError:
            self.last_ready="NOT_READY"

    def tick(self):
        result={"dynamic_detection":"NOT_IMPLEMENTED",
                "kalman_prediction":"NOT_IMPLEMENTED",
                "mpc_cbf_solver":"NOT_IMPLEMENTED",
                "actuator_output":"DISABLED",
                "command_publisher_created":False,
                "reference_path_messages_seen":self.path_count,
                "upstream_readiness":self.last_ready}
        self.status.publish(String(data=json.dumps(result,sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node=DynamicPort()
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
