"""Publishes castor_interfaces/Heartbeat once a second for one component.

Every component's launch file runs one of these, so the system supervisor can
tell which containers are alive without knowing anything about their insides.
"""

import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from castor_interfaces.msg import Heartbeat


class HeartbeatNode(Node):

    def __init__(self):
        super().__init__("heartbeat")
        self.component = self.declare_parameter("component", "unknown").value
        self.robot_id = self.declare_parameter("robot_id", 0).value
        period = self.declare_parameter("period_s", 1.0).value
        self.revision = os.environ.get("CASTOR_IMAGE_REVISION", "dev")
        self.seq = 0
        self.pub = self.create_publisher(Heartbeat, "heartbeat", 10)
        self.create_timer(period, self.tick)

    def tick(self):
        msg = Heartbeat()
        msg.stamp = self.get_clock().now().to_msg()
        msg.component = self.component
        msg.robot_id = self.robot_id
        msg.image_revision = self.revision
        msg.sequence = self.seq
        self.seq += 1
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
