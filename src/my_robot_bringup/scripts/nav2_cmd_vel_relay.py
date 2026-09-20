#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class Nav2CmdVelRelay(Node):
    def __init__(self):
        super().__init__("nav2_cmd_vel_relay")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/diff_drive_controller/cmd_vel")
        self.declare_parameter("output_stamped", True)
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("max_linear_x", 0.25)
        self.declare_parameter("max_angular_z", 0.50)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("publish_zero_on_timeout", True)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.output_stamped = bool(self.get_parameter("output_stamped").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.publish_zero_on_timeout = bool(self.get_parameter("publish_zero_on_timeout").value)

        msg_type = TwistStamped if self.output_stamped else Twist
        self.publisher = self.create_publisher(msg_type, self.output_topic, 10)
        self.subscription = self.create_subscription(Twist, self.input_topic, self.on_cmd_vel, 10)

        self.last_cmd_time = None
        self.zero_sent = True
        self.timer = self.create_timer(0.05, self.on_timer)

        output_type = "TwistStamped" if self.output_stamped else "Twist"
        self.get_logger().info(
            f"Relaying {self.input_topic} Twist to {self.output_topic} {output_type}; "
            f"limits: linear.x +/-{self.max_linear_x:.2f} m/s, "
            f"angular.z +/-{self.max_angular_z:.2f} rad/s"
        )

    def on_cmd_vel(self, msg: Twist):
        safe = self.clamp_twist(msg)
        self.publish(safe)
        self.last_cmd_time = self.get_clock().now()
        self.zero_sent = self.is_zero(safe)

    def on_timer(self):
        if not self.publish_zero_on_timeout or self.last_cmd_time is None or self.zero_sent:
            return

        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.command_timeout:
            self.publish(Twist())
            self.zero_sent = True
            self.get_logger().warn("Command timeout reached; published zero velocity.")

    def clamp_twist(self, msg: Twist) -> Twist:
        safe = Twist()
        safe.linear.x = self.clamp(msg.linear.x, -self.max_linear_x, self.max_linear_x)
        safe.angular.z = self.clamp(msg.angular.z, -self.max_angular_z, self.max_angular_z)

        if not math.isfinite(safe.linear.x):
            safe.linear.x = 0.0
        if not math.isfinite(safe.angular.z):
            safe.angular.z = 0.0

        return safe

    def publish(self, twist: Twist):
        if self.output_stamped:
            stamped = TwistStamped()
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.header.frame_id = self.frame_id
            stamped.twist = twist
            self.publisher.publish(stamped)
        else:
            self.publisher.publish(twist)

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def is_zero(msg: Twist) -> bool:
        return abs(msg.linear.x) < 1e-6 and abs(msg.angular.z) < 1e-6


def main():
    rclpy.init()
    node = Nav2CmdVelRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
