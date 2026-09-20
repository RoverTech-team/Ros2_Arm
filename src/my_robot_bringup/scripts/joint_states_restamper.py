#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatesRestamper(Node):
    def __init__(self):
        super().__init__("joint_states_restamper")
        self._sub = self.create_subscription(JointState, "joint_states", self._callback, 10)
        self._pub = self.create_publisher(JointState, "joint_states_ros", 10)

    def _callback(self, msg: JointState):
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = JointStatesRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
